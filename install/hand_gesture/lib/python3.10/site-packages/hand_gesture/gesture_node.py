#!/usr/bin/env python3
import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


FORWARD_SPEED = 0.30
TURN_SPEED = 0.8


class GestureControlNode(Node):
    def __init__(self):
        super().__init__("gesture_control_node")

        self.get_logger().info("✅ Gesture Control Node Started")
        self.get_logger().info("RIGHT Hand -> DRIVE (OPEN=FORWARD, FIST=STOP, POINT LEFT/RIGHT)")
        self.get_logger().info("LEFT  Hand -> GRIPPER (OPEN / CLOSE / UP / DOWN)")

        # =========================
        # ✅ PUBLISHERS
        # =========================
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gripper_pub = self.create_publisher(String, "/gripper_cmd", 10)

        # =========================
        # ✅ CAMERA SOURCE
        # =========================
        # For laptop / USB camera:
        #self.cam = cv2.VideoCapture(0)

        # For DroidCam IP (uncomment + change IP if needed):
        self.cam = cv2.VideoCapture("http://192.168.43.101:4747/video")

        if not self.cam.isOpened():
            self.get_logger().error("❌ Could not open camera!")

        # =========================
        # ✅ MEDIAPIPE INIT
        # =========================
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_draw = mp.solutions.drawing_utils

        # =========================
        # ✅ TIMER LOOP (20 Hz)
        # =========================
        self.timer = self.create_timer(0.05, self.process_frame)

    def process_frame(self):
        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().warn("❌ Camera frame not received")
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        # Control variables (defaults)
        speed = 0.0
        turn = 0.0
        gripper_state = "HOLD"
        movement_text = "STOP"

        if results.multi_hand_landmarks and results.multi_handedness:
            for i, hand in enumerate(results.multi_hand_landmarks):
                lm = hand.landmark
                hand_type = results.multi_handedness[i].classification[0].label  # "Left" or "Right"

                # Fingertips (excluding thumb)
                tips = [8, 12, 16, 20]
                fingers = []

                for tip in tips:
                    fingers.append(lm[tip].y < lm[tip - 2].y)

                index_up, middle_up, ring_up, pinky_up = fingers
                finger_count = sum(fingers)

                # ==========================================
                # ✅ RIGHT HAND → ROBOT DRIVE
                # ==========================================
                if hand_type == "Right":
                    # ✊ FIST → STOP (no fingers)
                    if finger_count == 0:
                        speed = 0.0
                        turn = 0.0

                    # ✋ OPEN HAND → FORWARD (all fingers up)
                    elif finger_count >= 4:
                        speed = 0.5
                        turn = 0.0

                    # 👉 POINT RIGHT: index up AND tip to right of its base
                    elif index_up and lm[8].x > lm[6].x:
                        speed = 0.0
                        turn = -8.0

                    # 👈 POINT LEFT: index up AND tip to left of its base
                    elif index_up and lm[8].x < lm[6].x:
                        speed = 0.0
                        turn = 8.0

                # ==========================================
                # ✅ LEFT HAND → GRIPPER CONTROL
                # ==========================================
                elif hand_type == "Left":
                    # ✊ Fist → CLOSE
                    if finger_count == 0:
                        gripper_state = "CLOSE"

                    # ✋ Open palm → OPEN
                    elif finger_count >= 4:
                        gripper_state = "OPEN"

                    # ☝ Index UP → GRIPPER UP
                    elif index_up and not middle_up:
                        gripper_state = "UP"

                    # 👇 Index DOWN (no fingers counted as up, but you can refine if needed)
                    elif not index_up and finger_count == 0:
                        gripper_state = "DOWN"

                    else:
                        gripper_state = "HOLD"

                # Draw landmarks
                self.mp_draw.draw_landmarks(frame, hand, self.mp_hands.HAND_CONNECTIONS)

        # =========================
        # ✅ MOVEMENT LABEL
        # =========================
        if speed == 0.0:
            movement_text = "STOP"
        else:
            if turn > 0:
                movement_text = "FORWARD + LEFT"
            elif turn < 0:
                movement_text = "FORWARD + RIGHT"
            else:
                movement_text = "FORWARD"

        # =========================
        # ✅ PUBLISH /cmd_vel
        # =========================
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = turn
        self.cmd_vel_pub.publish(twist_msg)

        # =========================
        # ✅ PUBLISH /gripper_cmd
        # =========================
        grip_msg = String()
        grip_msg.data = gripper_state
        self.gripper_pub.publish(grip_msg)

        # =========================
        # ✅ DEBUG LOG
        # =========================
        self.get_logger().info(
            f"Speed: {speed:.2f} | Turn: {turn:.2f} | "
            f"Movement: {movement_text} | Gripper: {gripper_state}"
        )

        # =========================
        # ✅ HUD OVERLAY
        # =========================
        cv2.putText(frame, f"Speed: {speed:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.putText(frame, f"Turn: {turn:.2f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        cv2.putText(frame, f"Movement: {movement_text}", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.putText(frame, f"Gripper: {gripper_state}", (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        cv2.imshow("Gesture Control ROS2", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        # Cleanup camera & windows
        if self.cam.isOpened():
            self.cam.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
