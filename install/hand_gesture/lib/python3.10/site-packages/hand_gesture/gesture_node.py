import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp


class GestureCmdVelNode(Node):
    def __init__(self):
        super().__init__("gesture_cmd_vel_node")

        self.get_logger().info("✅ Gesture → /cmd_vel ROS2 Node Started")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        
        self.cam = cv2.VideoCapture("http://192.168.43.101:4747/video")

        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands()
        self.mp_draw = mp.solutions.drawing_utils

        #TIMER (20 Hz)
        self.timer = self.create_timer(0.05, self.process)

    def process(self):
        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().warn("❌ Camera frame not received")
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        speed = 0.0
        turn = 0.0
        movement_text = "STOP"

        if results.multi_hand_landmarks and results.multi_handedness:
            self.get_logger().info("Detected Hands")

            for i, hand in enumerate(results.multi_hand_landmarks):
                lm = hand.landmark
                hand_type = results.multi_handedness[i].classification[0].label

                tips = [4, 8, 12, 16, 20]
                fingers = []

                for tip in tips[1:]:
                    fingers.append(lm[tip].y < lm[tip - 2].y)

                finger_count = sum(fingers)


                #LEFT HAND → SPEED
                if hand_type == "Left":
                    if finger_count == 0:
                        speed = 0.0
                    elif finger_count == 1:
                        speed = 0.2
                    elif finger_count == 2:
                        speed = -0.2
                    else : 
                        speed = 0.0

    
                #RIGHT HAND → DIRECTION
                elif hand_type == "Right":
                    if finger_count == 1:
                        turn = 15.0
                    elif finger_count == 2:
                        turn = -15.0
                    else:
                        turn = 0.0

                self.mp_draw.draw_landmarks(frame, hand, mp.solutions.hands.HAND_CONNECTIONS)

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
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn
        self.pub.publish(msg)

        # =========================
        # ✅ DEBUG PRINT
        # =========================
        self.get_logger().info(
            f"Speed: {speed:.2f} | Turn: {turn:.2f} | Movement: {movement_text}"
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

        cv2.imshow("Gesture Control ROS2", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = GestureCmdVelNode()
    rclpy.spin(node)

    node.cam.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
