import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LawnMowerRobot(Node):

    def __init__(self):
        super().__init__("lawn_mower_robot_node")
        self.cmd_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # 상태 관리
        self.state = "forward"       # forward / turn / advance_short
        self.turn_direction = 1      # 좌/우 번갈아
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

        # 속도/거리 설정
        self.forward_speed = 0.25      # m/s
        self.turn_speed = 0.6          # rad/s
        self.short_advance_dist = 0.2  # m
        self.turn_angle = 1.5708       # 90도(rad)

        # 전방 장애물 거리
        self.front_min = 5.0

    def scan_callback(self, msg):
        # 전방 라이다 최소 거리 (0.05~5m 유효)
        front = [d for d in msg.ranges[200:400] if 0.05 < d < 5.0]
        self.front_min = min(front) if front else 5.0
        self.get_logger().info(f"Front distance: {self.front_min:.2f} m") 
        
    def timer_callback(self):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.state_start_time

        OBSTACLE_DIST = 0.5  # 장애물 감지 기준

        if self.state == "forward":
            if self.front_min < OBSTACLE_DIST:
                self.state = "turn"
                self.state_start_time = now
            else:
                cmd.linear.x = self.forward_speed

        elif self.state == "turn":
            cmd.angular.z = self.turn_direction * self.turn_speed
            turn_time = self.turn_angle / self.turn_speed
            if elapsed >= turn_time:
                self.state = "advance_short"
                self.state_start_time = now

        elif self.state == "advance_short":
            cmd.linear.x = self.forward_speed
            advance_time = self.short_advance_dist / self.forward_speed
            if elapsed >= advance_time:
                self.turn_direction *= -1  # 다음 번 회전 방향 반전
                self.state = "forward"
                self.state_start_time = now

        self.cmd_publisher_.publish(cmd)
        self.get_logger().info(
            f"상태: {self.state}, 전진 속도: {cmd.linear.x:.2f} m/s, 각속도: {cmd.angular.z:.2f} rad/s"
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        robot = LawnMowerRobot()
        rclpy.spin(robot)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
