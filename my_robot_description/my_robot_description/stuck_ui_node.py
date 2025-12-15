#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import threading
import os

class StuckUiNode(Node):
    """
    "ใบหน้า" (ฉบับ "ฉลาด" - คืนชีพ Wait Mode Loop)
    - นี่คือเวอร์ชันที่ถูกต้อง ที่มี "while loop"
    - (ฉบับแก้ไข Bug คำผิด และเว้นวรรค)
    """
    def __init__(self):
        super().__init__('stuck_ui_node')
        self.get_logger().info('Stuck UI Node (The Smart Face) is starting...')

        self.ui_request_sub = self.create_subscription(
            String, '/request_ui_popup', self.popup_callback, 10
        )
        self.cancel_sub = self.create_subscription(
            Bool, '/cancel_ui_alert', self.cancel_alert_callback, 10
        )
        self.decision_pub = self.create_publisher(
            String, '/ui_decision', 10
        )
        
        self.popup_lock = threading.Lock() 
        self.countdown_process = None
        self.choice_process = None
        self.is_cancelled = False 
        self.COUNTDOWN_TIME = 10 

    def popup_callback(self, msg: String):
        if not self.popup_lock.acquire(blocking=False):
            self.get_logger().warn('Popup request ignored, UI is already active.')
            return
            
        self.get_logger().info('Popup request received (Lock Acquired). Starting Smart Zenity thread...')
        self.is_cancelled = False 
        
        thread = threading.Thread(target=self.run_smart_zenity, args=(msg.data,), daemon=True)
        thread.start()

    def cancel_alert_callback(self, msg: Bool):
        if not msg.data: return
        
        if self.popup_lock.locked():
            self.get_logger().warn("Cancel signal received! Killing active Zenity dialogs...")
            self.is_cancelled = True 
            try:
                if self.countdown_process and self.countdown_process.poll() is None:
                    self.countdown_process.kill()
                if self.choice_process and self.choice_process.poll() is None:
                    self.choice_process.kill()
            except Exception as e:
                self.get_logger().error(f"Error while killing Zenity: {e}")


    def run_smart_zenity(self, trigger_reason: str):
        """
        ฟังก์ชันที่รันใน thread แยก
        (ฉบับแก้ไข Bug คำผิด และเว้นวรรค)
        """
        # ‼️ [แก้ไข] 1. ไม่ Hardcode DISPLAY=:0
        # ใช้ env ของระบบที่ main() ตรวจสอบให้แล้ว
        env = dict(os.environ)
        
        final_choice_mapped = "ui_cancelled" 
        
        self.countdown_process = None
        self.choice_process = None
        
        try:
            # --- ✨ STAGE 1: COUNTDOWN (10 วินาที) ✨ ---
            self.get_logger().info(f'UI Stage 1: Running {self.COUNTDOWN_TIME}s Smart Countdown...')
            
            if "NAV2_ABORTED" in trigger_reason:
                initial_text = "Navigation FAILED! Pausing..."
            else: # (SONAR_TRIGGER)
                initial_text = "Obstacle Detected! Pausing..."

            countdown_cmd = ['zenity', '--progress', '--width=350', '--title=Robot Paused!', 
                             f'--text={initial_text}', '--percentage=0', '--auto-close']
            
            self.countdown_process = subprocess.Popen(
                countdown_cmd, stdin=subprocess.PIPE, text=True, bufsize=1, stderr=subprocess.PIPE,
                env=env # ‼️ [แก้ไข] 2. ส่ง env เข้าไป
            )
            
            for i in range(self.COUNTDOWN_TIME):
                if not rclpy.ok() or self.is_cancelled: break
                time.sleep(1.0) 
                if self.is_cancelled: break 
                
                remaining_time = self.COUNTDOWN_TIME - (i + 1)
                percentage = int(((i + 1) / self.COUNTDOWN_TIME) * 100)
                update_text = f"# {initial_text}\n# Waiting... {remaining_time}s remaining.\n{percentage}"
                
                try:
                    if self.countdown_process.poll() is None and self.countdown_process.stdin and not self.countdown_process.stdin.closed:
                        self.countdown_process.stdin.write(update_text + '\n')
                        self.countdown_process.stdin.flush()
                    else:
                        if not self.is_cancelled: self.get_logger().warn("Countdown closed by user.")
                        self.is_cancelled = True 
                        break
                except (BrokenPipeError, OSError): 
                    self.is_cancelled = True
                    break
            
            if self.countdown_process and self.countdown_process.poll() is None:
                try: self.countdown_process.stdin.close()
                except Exception: pass
                try: self.countdown_process.wait(timeout=0.1)
                except subprocess.TimeoutExpired: self.countdown_process.kill(); self.countdown_process.wait()
            self.countdown_process = None 
            
            if self.is_cancelled:
                self.get_logger().info('UI Stage 1 was cancelled. Exiting thread.')
                final_choice_mapped = "ui_cancelled"
                return 

            # --- ✨ STAGE 2: 3-OPTION DIALOG ✨ ---
            self.get_logger().info('UI Stage 2: Showing "Normal" (3 options)...')
            
            # ‼️ [แก้ไข] 3. แก้คำผิด "cheack point" -> "Checkpoint"
            options_command = [
                'zenity', '--list', '--width=500', '--height=350', 
                '--title=ROBOT RECOVERY',
                '--text=Choose recovery action:', '--radiolist',
                '--column=Choice', '--column=Action',
                'FALSE', 'Return to Checkpoint', # <-- แก้ไข
                'TRUE', 'Wait', 
                'FALSE', 'Return to HOME'
            ]
            
            self.choice_process = subprocess.Popen(
                options_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                env=env # ‼️ [แก้ไข] 2. ส่ง env เข้าไป
            )
            stdout, stderr = self.choice_process.communicate(timeout=60.0) 
            choice_stage_2 = stdout.strip() if stdout else ""
            self.choice_process = None
            
            if self.is_cancelled:
                self.get_logger().info('UI Stage 2 was cancelled. Exiting thread.')
                final_choice_mapped = "ui_cancelled"
                return 
                
            
            # --- ✨ STAGE 3: WAIT MODE (ถ้าจำเป็น) ✨ ---
            if choice_stage_2 == 'Wait':
                self.get_logger().info('UI Stage 3: Entering "Wait Mode" (2 options)...')
                
                # ‼️ [แก้ไข] 4. แก้คำผิด และ "ลบเว้นวรรค" ที่ต่อท้าย
                options_command_wait = [
                    'zenity', '--list', 
                    '--width=500', '--height=350', 
                    '--title=ROBOT PAUSED (Wait Mode)',
                    '--text=Obstacle present. Robot is paused.\nChoose action:', '--radiolist',
                    '--column=Choice', '--column=Action',
                    'TRUE', 'Return to Checkpoint', # <-- แก้ไข (ลบเว้นวรรค)
                    'FALSE', 'Return to Home (0,0)'
                ]
                
                self.choice_process = subprocess.Popen(
                    options_command_wait, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                    env=env # ‼️ [แก้ไข] 2. ส่ง env เข้าไป
                )
                
                user_made_choice = False
                while rclpy.ok() and not self.is_cancelled:
                    poll_result = self.choice_process.poll()
                    if poll_result is not None:
                        user_made_choice = True
                        break
                    time.sleep(0.25)
                
                if user_made_choice:
                    stdout, _ = self.choice_process.communicate()
                    choice = stdout.strip()
                    if choice == 'Return to Home (0,0)': final_choice_mapped = "go_home"
                    # ‼️ [แก้ไข] 5. แก้ไขคำให้ตรงกัน (Bug ที่คุยกัน)
                    elif choice == 'Return to Checkpoint': final_choice_mapped = "go_checkpoint"
                    else: final_choice_mapped = "ui_cancelled" 
                else:
                    final_choice_mapped = "ui_cancelled" 
                
                self.choice_process = None
            
            # ‼️ [แก้ไข] 6. แก้ไขคำให้ตรงกัน
            elif choice_stage_2 == 'Return to HOME':
                final_choice_mapped = "go_home"
            elif choice_stage_2 == 'Return to Checkpoint': # <-- แก้ไข
                final_choice_mapped = "go_checkpoint"
            else:
                self.get_logger().warn('No choice made in Stage 2. Defaulting to cancel.')
                final_choice_mapped = "ui_cancelled"

        except subprocess.TimeoutExpired:
             self.get_logger().warn(f'Zenity dialog timed out. Defaulting to cancel.'); 
             final_choice_mapped = "ui_cancelled"
        except Exception as e:
            self.get_logger().error(f"Error in Zenity Stages: {e}")
            final_choice_mapped = "ui_cancelled"
        
        finally:
            # --- ✨ STAGE 4: PUBLISH & RELEASE ✨ ---
            decision_msg = String()
            decision_msg.data = final_choice_mapped
            self.decision_pub.publish(decision_msg)
            self.get_logger().info(f"Published final decision: {final_choice_mapped}")
            
            if self.countdown_process and self.countdown_process.poll() is None:
                self.countdown_process.kill()
            if self.choice_process and self.choice_process.poll() is None:
                self.choice_process.kill()
                
            self.countdown_process = None
            self.choice_process = None
            
            self.get_logger().info('UI Thread finished. Releasing lock.')
            self.popup_lock.release()

# (ฟังก์ชัน main เหมือนเดิม)
def main(args=None):
    rclpy.init(args=args)
    if 'DISPLAY' not in os.environ:
        print("ERROR: $DISPLAY environment variable is not set.", file=sys.stderr)
        print("       Cannot run Zenity UI node without a display.", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    node = StuckUiNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()