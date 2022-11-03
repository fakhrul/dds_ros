import sys
import rclpy
from rclpy.node import Node  # Handles the creation of nodes
import serial
import time
from example_interfaces.srv import SetBool

class DdsDioClientAsync(Node):
    def __init__(self):
        super().__init__('dds_dio_client_async')
        self.cli = self.create_client(SetBool, 'set_buzzer')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self):
        if sys.argv[1] == 'True':
            self.req.data = True
        else:
            self.req.data = False
        self.future = self.cli.call_async(self.req)


def main(args=None):

    rclpy.init(args=args)
    dds_dio_client = DdsDioClientAsync()
    dds_dio_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(dds_dio_client)
        # See if the service has replied
        if dds_dio_client.future.done():
            try:
                response = dds_dio_client.future.result()
            except Exception as e:
                dds_dio_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                dds_dio_client.get_logger().info(
                    'Result of SetBool: for %s = %s' %
                    (dds_dio_client.req.data, response.message))
            break
    dds_dio_client.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()