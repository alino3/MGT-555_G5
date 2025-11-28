import serial
import time
import threading

class Arduino3Tester:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        print(f"Connected to Arduino 3 on {port}")
        
    def send_command(self, command):
        """Send command to Arduino and wait for response"""
        print(f"Sending: {command}")
        self.ser.write((command + '\n').encode())
        
        # Read response
        response = []
        start_time = time.time()
        while time.time() - start_time < 5:  # 5 second timeout
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if line:
                    print(f"Arduino: {line}")
                    if "completed" in line or "ERROR" in line or "Ready" in line:
                        break
                response.append(line)
        
        return response
    
    def move_motor(self, steps, direction):
        """Move the Z-axis motor"""
        command = f"M {steps} {direction}"
        self.send_command(command)
    
    def set_servo_phi(self, angle):
        """Set Phi servo angle"""
        command = f"S P {angle}"
        self.send_command(command)
    
    def set_servo_gripper(self, angle):
        """Set Gripper servo angle"""
        command = f"S G {angle}"
        self.send_command(command)
    
    def enable_motor(self, enable=True):
        """Enable or disable the motor"""
        command = f"E {1 if enable else 0}"
        self.send_command(command)
    
    def run_test(self):
        """Run the test sequence"""
        self.send_command("T")
    
    def interactive_mode(self):
        """Start interactive mode"""
        print("\n=== INTERACTIVE MODE ===")
        print("Type commands directly (or 'quit' to exit):")
        
        while True:
            try:
                user_input = input("> ").strip()
                if user_input.lower() == 'quit':
                    break
                elif user_input:
                    self.send_command(user_input)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def stress_test_motor(self, cycles=10, steps=1000):
        """Run stress test on the motor"""
        print(f"\n=== STRESS TEST: {cycles} cycles of {steps} steps ===")
        
        for i in range(cycles):
            print(f"Cycle {i+1}/{cycles}")
            
            # Forward
            self.move_motor(steps, 1)
            time.sleep(0.5)
            
            # Backward
            self.move_motor(steps, 0)
            time.sleep(0.5)
        
        print("Stress test completed")
    
    def close(self):
        """Close serial connection"""
        self.ser.close()
        print("Connection closed")

# Usage examples
if __name__ == "__main__":
    # Update this to your Arduino 3's COM port
    PORT = "COM5"  # Change to your actual port (COM3, COM4, etc.)
    
    tester = None
    try:
        tester = Arduino3Tester(PORT)
        
        # Test 1: Basic functionality
        print("\n1. Testing basic commands...")
        tester.enable_motor(True)
        tester.move_motor(500, 1)   # 500 steps forward
        tester.move_motor(500, 0)   # 500 steps backward
        
        # Test 2: Servos
        print("\n2. Testing servos...")
        tester.set_servo_phi(0)
        tester.set_servo_gripper(0)
        time.sleep(1)
        tester.set_servo_phi(180)
        tester.set_servo_gripper(180)
        time.sleep(1)
        tester.set_servo_phi(90)
        tester.set_servo_gripper(90)
        
        # Test 3: Run built-in test
        print("\n3. Running built-in test sequence...")
        tester.run_test()
        
        # Test 4: Stress test (optional)
        print("\n4. Running stress test...")
        tester.stress_test_motor(cycles=3, steps=800)
        
        # Test 5: Interactive mode
        print("\n5. Starting interactive mode...")
        tester.interactive_mode()
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if tester:
            tester.close()