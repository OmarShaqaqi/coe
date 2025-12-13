#!/usr/bin/env python3
import cv2
import mediapipe as mp
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class GestureControlNode(Node):
    def __init__(self):
        super().__init__("gesture_control_node")

        self.get_logger().info("✅ Gesture Control Node Started")
        self.get_logger().info("RIGHT Hand -> DRIVE")
        self.get_logger().info("LEFT  Hand -> ARM (GIMBAL) + GRIPPER")

        # =========================
        # ✅ PUBLISHERS
        # =========================
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gripper_pub = self.create_publisher(String, "/gripper_cmd", 10)
        self.gimbal_pub = self.create_publisher(Twist, "/arm_cmd", 10)

        # =========================
        # ✅ ARM THROTTLE (publish every 2 seconds max)
        # =========================
        self.last_arm_time = 0.0
        self.arm_interval = 1.0  # seconds
        self.last_pitch = 0.0
        self.last_yaw = 0.0

        # =========================
        # ✅ CAMERA
        # =========================
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam.set(cv2.CAP_PROP_FPS, 30)

        if not self.cam.isOpened():
            self.get_logger().error("❌ Camera not opened")

        # =========================
        # ✅ MEDIAPIPE
        # =========================
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            model_complexity=0,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        # =========================
        # ✅ LOOP (30 Hz)
        # =========================
        self.timer = self.create_timer(0.033, self.process_frame)

    def process_frame(self):
        ret, frame = self.cam.read()
        if not ret:
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        # Defaults
        speed = 0.0
        turn = 0.0
        pitch_speed = 0.0
        yaw_speed = 0.0
        gripper_state = "HOLD"

        if results.multi_hand_landmarks and results.multi_handedness:
            for i, hand in enumerate(results.multi_hand_landmarks):
                lm = hand.landmark
                hand_type = results.multi_handedness[i].classification[0].label

                # Fingers (index, middle, ring, pinky)
                tips = [8, 12, 16, 20]
                fingers = [lm[t].y < lm[t - 2].y for t in tips]
                index_up, middle_up, ring_up, pinky_up = fingers
                finger_count = sum(fingers)

                # =========================
                # RIGHT HAND → DRIVE
                # =========================
                if hand_type == "Right":
                    if finger_count == 0:
                        speed = 0.0
                        turn = 0.0
                    elif finger_count >= 4:
                        speed = 0.5
                    if finger_count == 3:
                        speed = -0.5
                    elif index_up and lm[8].x > lm[6].x:
                        turn = -30.0
                    elif index_up and lm[8].x < lm[6].x:
                        turn = 30.0

                # =========================
                # LEFT HAND → ARM + GRIPPER
                # =========================
                elif hand_type == "Left":

                    # ✊ Fist → CLOSE
                    if finger_count == 0:
                        gripper_state = "CLOSE"

                    # ✋ Open hand → OPEN
                    elif finger_count >= 4:
                        gripper_state = "OPEN"

                    # ☝ One finger UP → Y+
                    elif finger_count == 1 and index_up:
                        pitch_speed = +40.0

                    # 👇 One finger DOWN → Y-
                    elif finger_count == 1 and not index_up:
                        pitch_speed = -40.0

                    elif finger_count == 2 and (not index_up) and (not middle_up):
                        pitch_speed = -40.0

                    # ✌️ Two fingers UP → X+
                    elif finger_count == 2 and index_up and middle_up:
                        yaw_speed = +60.0

                    # ✌️ Three fingers → X-
                    elif finger_count == 3:
                        yaw_speed = -60.0

                self.mp_draw.draw_landmarks(frame, hand, self.mp_hands.HAND_CONNECTIONS)

        # Store latest desired arm command (do NOT publish at 30 Hz)
        self.last_pitch = pitch_speed
        self.last_yaw = yaw_speed

        # =========================
        # PUBLISH DRIVE (30 Hz)
        # =========================
        cmd = Twist()
        cmd.linear.x = speed
        if speed != 0 :
            turn = 0.0
        cmd.angular.z = turn
        self.cmd_vel_pub.publish(cmd)

        # =========================
        # PUBLISH GRIPPER (30 Hz; you can optimize to publish only on change later)
        # =========================
        grip = String()
        grip.data = gripper_state
        self.gripper_pub.publish(grip)

        # =========================
        # PUBLISH ARM/GIMBAL (THROTTLED: every 2 seconds max)
        # =========================
        now = time.time()
        if (now - self.last_arm_time) >= self.arm_interval:
            # Only publish if there's an actual command
            if self.last_pitch != 0.0 or self.last_yaw != 0.0:
                gimbal = Twist()
                gimbal.angular.y = self.last_pitch
                gimbal.angular.x = self.last_yaw
                self.gimbal_pub.publish(gimbal)

                self.get_logger().info(
                    f"✅ Arm cmd sent: pitch={self.last_pitch}, yaw={self.last_yaw}"
                )
            self.last_arm_time = now

        # =========================
        # HUD
        # =========================
        remaining = max(0.0, self.arm_interval - (now - self.last_arm_time))
        cv2.putText(frame, f"Gimbal Y: {pitch_speed}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Gimbal X: {yaw_speed}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(frame, f"Gripper: {gripper_state}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, f"Arm cooldown: {remaining:.1f}s", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, f"x: {speed:.2f}  z: {turn:.2f}", (10, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("Gesture Control", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cam.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = GestureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
