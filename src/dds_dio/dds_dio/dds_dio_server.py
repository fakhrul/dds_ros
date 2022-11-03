import rclpy
from rclpy.node import Node  # Handles the creation of nodes

import serial
import time
from example_interfaces.srv import SetBool

class DdsDioService(Node):
    def __init__(self):
        super().__init__('dds_dio_service')

        self.srv = self.create_service(SetBool, 'set_buzzer', self.set_buzzer_callback)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)


    def set_buzzer_callback(self, request, response):
        self.get_logger().info('Incoming request\ndata: %s' % (request.data))

        if request.data == True:
            self.ser.write(b"B")
        else:
            self.ser.write(b"b")
        
        response.message = "success"
        response.success = True
        return response


def main(args=None):

    rclpy.init(args=args)
    dds_dio_service = DdsDioService()
    rclpy.spin(dds_dio_service)
    rclpy.shutdown()

if __name__=='__main__':
    main()