import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

from robomaster import robot


class RobotDriveAndGripperController(Node):
    def __init__(self):
        super().__init__("robot_drive_and_gripper_controller")

        self.get_logger().info("✅ Robot Drive + Gripper Controller Node Started")

        # ============================
        # ✅ SUBSCRIBERS
        # ============================

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        self.gripper_sub = self.create_subscription(
            String,
            "/gripper_cmd",
            self.gripper_callback,
            10
        )

        # ============================
        # ✅ INITIALIZE ROBOT
        # ============================

        self.ep_robot = robot.Robot()
        try:
            self.ep_robot.initialize(conn_type="sta")
            self.get_logger().info("✅ RoboMaster Connected")
        except Exception as e:
            self.get_logger().error(f"❌ RoboMaster connection failed: {e}")
            return

        self.chassis = self.ep_robot.chassis
        self.gripper = self.ep_robot.gripper

        # ============================
        # ✅ SAFETY VARIABLES
        # ============================

        self.last_cmd_time = time.time()
        self.last_gripper_state = "HOLD"

        # ✅ Auto-stop watchdog (every 0.1s)
        self.safety_timer = self.create_timer(0.1, self.safety_check)

    # ============================
    # ✅ DRIVE CALLBACK
    # ============================

    def cmd_vel_callback(self, msg: Twist):
        speed = msg.linear.x
        turn = msg.angular.z

        self.last_cmd_time = time.time()

        self.chassis.drive_speed(
            x=speed,
            y=0.0,
            z=turn
        )

        self.get_logger().info(
            f"🚗 Drive → Speed: {speed:.2f} | Turn: {turn:.2f}"
        )

    # ============================
    # ✅ SAFETY AUTO-STOP
    # ============================

    def safety_check(self):
        if time.time() - self.last_cmd_time > 0.5:
            self.chassis.drive_speed(0.0, 0.0, 0.0)

    # ============================
    # ✅ GRIPPER CALLBACK
    # ============================

    def gripper_callback(self, msg: String):
        gripper_state = msg.data

        if gripper_state == self.last_gripper_state:
            return

        self.last_gripper_state = gripper_state
        self.get_logger().info(f"🖐️ Gripper Command → {gripper_state}")

        # ✅ ONLY OPEN & CLOSE ARE SUPPORTED

        if gripper_state == "OPEN":
            self.gripper.open(power=50)
            self.get_logger().info("✅ Gripper OPEN")

        elif gripper_state == "CLOSE":
            self.gripper.close(power=50)
            self.get_logger().info("✅ Gripper CLOSE")

        elif gripper_state in ["UP", "DOWN"]:
            self.get_logger().warn(
                "⚠️ UP / DOWN ignored — Standard RoboMaster EP gripper has NO lift motor"
            )

        elif gripper_state == "HOLD":
            pass

        else:
            self.get_logger().warn(f"⚠️ Unknown gripper command: {gripper_state}")


def main():
    rclpy.init()
    node = RobotDriveAndGripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
