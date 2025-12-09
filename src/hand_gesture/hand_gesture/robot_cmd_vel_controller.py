import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


from robomaster import robot


class RobotCmdVelController(Node):
    def __init__(self):
        super().__init__("robot_cmd_vel_controller")

        self.get_logger().info("✅ Robot /cmd_vel Controller Node Started")

       
        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        
        self.ep_robot = robot.Robot()
        try:
            self.ep_robot.initialize(conn_type="sta")
            self.get_logger().info("✅ RoboMaster Connected")
        except Exception as e:
            self.get_logger().error(f"❌ RoboMaster connection failed: {e}")
            return

        self.chassis = self.ep_robot.chassis

   
    def cmd_vel_callback(self, msg: Twist):

        speed = msg.linear.x
        turn = msg.angular.z

      
        self.get_logger().info(
            f"🚗 Speed: {speed:.2f} | Turn: {turn:.2f}"
        )

        
        self.chassis.drive_speed(
            x=speed,      # forward/backward
            y=0.0,        # no sideways motion
            z=turn        # rotation
        )


def main():
    rclpy.init()
    node = RobotCmdVelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
