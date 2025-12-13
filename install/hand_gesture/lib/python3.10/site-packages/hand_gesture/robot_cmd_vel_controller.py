#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import threading
from robomaster import robot


ARM_STEP_X = 20   # mm per command
ARM_STEP_Y = 20   # mm per command


class RobotDriveArmGripperController(Node):
    def __init__(self):
        super().__init__("robot_drive_arm_gripper_controller")

        self.get_logger().info("✅ Robot Drive + Robotic Arm + Gripper Controller Started")

        # ============================
        # SUBSCRIBERS
        # ============================

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.arm_sub = self.create_subscription(
            Twist, "/arm_cmd", self.arm_callback, 10
        )

        self.gripper_sub = self.create_subscription(
            String, "/gripper_cmd", self.gripper_callback, 10
        )

        # ============================
        # INIT ROBOT
        # ============================

        self.ep = robot.Robot()
        self.ep.initialize(conn_type="sta")

        self.chassis = self.ep.chassis
        self.arm = self.ep.robotic_arm
        self.gripper = self.ep.gripper

        self.arm.start()
        self.arm_busy = False

        self.last_drive_time = time.time()

        self.drive_watchdog = self.create_timer(0.1, self.drive_safety)

    # ============================
    # DRIVE
    # ============================

    def cmd_vel_callback(self, msg: Twist):
        self.last_drive_time = time.time()
        z=msg.angular.z
        if msg.linear.x != 0 :
            z = 0
            
        self.chassis.drive_speed(
            x=msg.linear.x,
            y=0.0,
            z=z
        )

    def drive_safety(self):
        if time.time() - self.last_drive_time > 0.5:
            self.chassis.drive_speed(0.0, 0.0, 0.0)

    # ============================
    # ROBOTIC ARM
    # ============================

    def arm_callback(self, msg: Twist):
        if self.arm_busy:
            return

        dx = msg.angular.x
        dy = msg.angular.y
        if dx == 0.0 and dy == 0.0:
            return

        self.arm_busy = True
        print("outside")
        threading.Thread(target=self._do_arm_move, args=(dx, dy), daemon=True).start()

    def _do_arm_move(self, dx, dy):
        try:
            print("inside")
            print(dx,dy)
            ok = self.arm.move(dx, dy).wait_for_completed(1)   
            print(ok)           # starts action
            if not ok:
                self.get_logger().warn("Arm move timed out")
        except Exception as e:
            self.get_logger().error(f"Arm move failed: {e}")
        finally:
            self.arm_busy = False

    # ============================
    # GRIPPER
    # ============================

    def gripper_callback(self, msg: String):
        if msg.data == "OPEN":
            self.gripper.open(power=50)

        elif msg.data == "CLOSE":
            self.gripper.close(power=50)

        elif msg.data == "HOLD":
            self.gripper.pause()


def main():
    rclpy.init()
    node = RobotDriveArmGripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
