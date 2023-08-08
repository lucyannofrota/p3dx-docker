import Jetson.GPIO as GPIO
import time

from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node

# Pin Definitons:
but_pin = 22  # Board pin 22
led_pin = 18  # Board pin 22

class E_STOP(Node):

    start = 0
    stop = 0

    input_state = 0

    btn_state = True

    motors_enable = True

    def __init__(self):
        super().__init__('ROS2_E_STOP')
        self.disable_cli = self.create_client(Empty, '/disable_motors')
        while not self.disable_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /disable_motors not available, waiting again...')
        self.get_logger().info('/disable_motors service client initialized')
        self.enable_cli = self.create_client(Empty, '/enable_motors')
        while not self.enable_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /enable_motors not available, waiting again...')
        self.get_logger().info('/enable_motors service client initialized')
        self.req = Empty.Request()

    def send_disable_request(self):
        self.future = self.disable_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.motors_enable = False
        self.get_logger().warn('Motors State: Disable!')
        return self.future.result()

    def send_enable_request(self):
        self.future = self.enable_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.motors_enable = True
        self.get_logger().warn('Motors State: Enable!')
        return self.future.result()
    
    def btn_callback(self,channel):
        if(self.btn_state):
            self.get_logger().warn("Button Pressed!")
            self.btn_state = False
            self.start = time.time()
        else:
            self.get_logger().warn("Button Released!")
            self.btn_state = True
            self.stop = time.time()

            if(self.stop-self.start < 2):
                self.send_disable_request()
                self.get_logger().warn("\tDisable Motors")
                motors_enable = False
            else:
                self.send_enable_request()
                self.get_logger().warn("\tEnable Motors")
                motors_enable = True
            
            if motors_enable:
                GPIO.output(led_pin, GPIO.HIGH)
            else:
                GPIO.output(led_pin, GPIO.LOW)



def main(args=None):
    rclpy.init(args=args)

    e_stop = E_STOP()

    # Pin Setup:
    GPIO.cleanup()  # cleanup all GPIOs
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    GPIO.setup(led_pin, GPIO.OUT)  # LED pin set as output
    GPIO.setup(but_pin, GPIO.IN)  # button pin set as input

    # Initial state for LEDs:
    GPIO.output(led_pin, GPIO.LOW)

    GPIO.add_event_detect(but_pin, GPIO.BOTH, callback=e_stop.btn_callback, bouncetime=10, polltime=0.1)

    try:
        while True:
            e_stop.get_logger().info("Waiting for button event...")
            time.sleep(100)

    finally:
        e_stop.get_logger().warn("Ending program!")
        e_stop.get_logger().warn("Disabling Motors!")
        GPIO.output(led_pin, GPIO.LOW)
        e_stop.send_disable_request()
        GPIO.cleanup()  # cleanup all GPIOs
        e_stop.destroy_node()
        rclpy.shutdown()
    



if __name__ == '__main__':
    main()