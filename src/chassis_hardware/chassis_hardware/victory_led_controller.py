#!/usr/bin/env python3

"""
Victory LED Controller Node

This node subscribes to the /victory topic (Bool) and controls GPIO pins to make LEDs blink
when a victory signal is received.

Features:
- Subscribes to /victory topic (std_msgs/Bool)
- Controls GPIO pin 31 for LED blinking using BOARD mode
- Uses RPi.GPIO library with fallback for development environments
- true = continuous blinking, false = LED off
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
import threading
import subprocess

# Try to import RPi.GPIO, fallback to mock for development
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    print("RPi.GPIO imported successfully")
except ImportError:
    print("RPi.GPIO not available, using mock GPIO for development")
    GPIO_AVAILABLE = False
    
    # Mock GPIO class for development
    class MockGPIO:
        BOARD = "BOARD"
        OUT = "OUT"
        HIGH = True
        LOW = False
        
        @staticmethod
        def setmode(mode):
            print(f"Mock GPIO: setmode({mode})")
            
        @staticmethod
        def setup(pin, mode):
            print(f"Mock GPIO: setup(pin={pin}, mode={mode})")
            
        @staticmethod
        def output(pin, value):
            print(f"Mock GPIO: output(pin={pin}, value={value})")
            
        @staticmethod
        def cleanup():
            print("Mock GPIO: cleanup()")
    
    GPIO = MockGPIO()

class VictoryLEDController(Node):
    def __init__(self):
        super().__init__('victory_led_controller')
        
        # GPIO configuration - using your verified settings
        self.led_pin = 31  # GPIO pin (BOARD mode)
        
        # Initialize GPIO
        self.setup_gpio()
        
        # Victory state and blinking control
        self.should_blink = False
        self.blink_thread = None
        self.stop_blinking = threading.Event()
        
        # Subscribe to victory topic (Bool)
        self.victory_subscriber = self.create_subscription(
            Bool, '/victory', self.victory_callback, 10)
        
        self.get_logger().info('Victory LED Controller started')
        self.get_logger().info(f'LED pin: {self.led_pin} (BOARD mode)')
        if not GPIO_AVAILABLE:
            self.get_logger().warn('Running in development mode - GPIO commands will be mocked')
    
    def setup_gpio(self):
        """Initialize GPIO settings using your verified configuration"""
        try:
            # Initialize LED GPIO using busybox devmem
            self.initialize_led_gpio()
            
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.led_pin, GPIO.OUT)
            GPIO.output(self.led_pin, GPIO.LOW)  # Start with LED off
            self.get_logger().info(f'GPIO pin {self.led_pin} (BOARD mode) configured successfully')
        except Exception as e:
            self.get_logger().error(f'GPIO setup error: {e}')
    
    def initialize_led_gpio(self):
        """Initialize LED GPIO using busybox devmem command"""
        try:
            cmd = ['sudo', 'busybox', 'devmem', '0x2430070', 'w', '0x004']
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.get_logger().info('LED GPIO initialized with busybox devmem')
            else:
                self.get_logger().warn(f'GPIO initialization warning: {result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().error('GPIO initialization timeout')
        except Exception as e:
            self.get_logger().error(f'GPIO initialization error: {e}')
    
    def victory_callback(self, msg):
        """Handle victory state messages"""
        new_victory_state = msg.data
        
        if new_victory_state != self.should_blink:
            self.should_blink = new_victory_state
            self.get_logger().info(f'Victory state changed to: {self.should_blink}')
            
            if self.should_blink:
                # Start continuous blinking
                self.start_victory_blink()
            else:
                # Stop blinking
                self.stop_victory_blink()
    
    def start_victory_blink(self):
        """Start continuous LED blinking in a separate thread"""
        # Stop any existing blinking
        self.stop_victory_blink()
        
        # Reset the stop event
        self.stop_blinking.clear()
        
        # Start new blinking thread
        self.blink_thread = threading.Thread(target=self.blink_led_worker)
        self.blink_thread.daemon = True
        self.blink_thread.start()
        
        self.get_logger().info('Started continuous victory LED blinking')
    
    def stop_victory_blink(self):
        """Stop LED blinking"""
        if self.blink_thread and self.blink_thread.is_alive():
            self.stop_blinking.set()
            self.blink_thread.join(timeout=1.0)
            
        # Turn off LED
        try:
            self.initialize_led_gpio()
            GPIO.output(self.led_pin, GPIO.LOW)
        except Exception as e:
            self.get_logger().error(f'Error turning off LED: {e}')
        
        self.get_logger().info('Stopped victory LED blinking')
    
    def blink_led_worker(self):
        """Worker function for continuous LED blinking - using your verified timing"""
        try:
            curr_value = GPIO.HIGH
            
            while not self.stop_blinking.is_set():
                # Initialize GPIO before each operation
                self.initialize_led_gpio()
                
                # Output current value
                GPIO.output(self.led_pin, curr_value)
                self.get_logger().debug(f"pin {self.led_pin} now is {curr_value}")
                
                # Wait for 0.07 seconds (your verified timing)
                if self.stop_blinking.wait(timeout=0.07):
                    break  # Stop event was set
                
                # Toggle LED state (using your verified XOR method)
                curr_value ^= GPIO.HIGH
            
            # Ensure LED is off when stopping
            self.initialize_led_gpio()
            GPIO.output(self.led_pin, GPIO.LOW)
            
        except Exception as e:
            self.get_logger().error(f'Error in LED blinking: {e}')
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.stop_victory_blink()
        
        try:
            GPIO.cleanup()
            self.get_logger().info('GPIO cleanup completed')
        except Exception as e:
            self.get_logger().error(f'GPIO cleanup error: {e}')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = VictoryLEDController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Victory LED Controller stopped by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
