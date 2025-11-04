#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class GapFollow(Node):
    def __init__(self):
        super().__init__('gap_follow_node')

        # Topics
        self.lidarscan_topic = '/scan'
        self.drive_topic = '/drive'
        self.best_point_marker_topic = '/best_point_marker'
        self.bubble_marker_topic = '/bubble_point_marker'

        # Pub/Sub
        self.lidar_sub = self.create_subscription(
            LaserScan, self.lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        self.best_point_marker_pub = self.create_publisher(
            Marker, self.best_point_marker_topic, 10)
        self.bubble_marker_pub = self.create_publisher(
            Marker, self.bubble_marker_topic, 10)

        # ---------------- Params (튜닝 포인트) ----------------
        self.disparity_threshold = 0.5            # 인접 빔 거리차
        self.min_bubble_radius = 0.30             # 버블 반경 하한 (m)
        self.max_bubble_radius = 0.60             # 버블 반경 상한 (m)
        self.bubble_distance_threshold = 10.0     # 멀수록 큰 버블

        self.min_gap_distance = 0.9               # gap 후보 최소거리 (m) (기존 1.0→0.9로 약간 완화)
        self.min_gap_size = 18                    # gap 최소 길이(인덱스) (기존 22→18로 완화)

        self.lidar_cap = 30.0                     # Inf 대체 상한
        self.fov_deg_min = -42.0                  # 사용 FOV (도)
        self.fov_deg_max =  42.0

        # 속도/조향 제한 + 스무딩
        self.max_speed = 7.2
        self.min_speed = 4.0
        self.max_steer_deg = 20.0                 # 조향 제한
        self.angle_gain_speed = 2.0               # 각도에 따른 감속량
        self.alpha_v = 0.5                        # 속도 EMA
        self.alpha_s = 0.5                        # 조향 EMA
        self.max_dv = 0.6                         # 사이클당 속도 변화 제한 (m/s)
        self.max_ds_deg = 6.0                     # 사이클당 조향 변화 제한 (deg)

        # State
        self.prev_speed = 0.0
        self.prev_steer = 0.0

        self.get_logger().info("GapFollow Node Initialized Successfully")

    # ---------------- Marker ----------------
    def publish_best_point_marker(self, best_point_idx, best_point_distance, angle_min, angle_increment):
        angle = angle_min + best_point_idx * angle_increment
        x = best_point_distance * np.cos(angle)
        y = best_point_distance * np.sin(angle)

        marker = Marker()
        marker.header.frame_id = "ego_racecar/laser"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "best_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        self.best_point_marker_pub.publish(marker)

    def publish_closest_bubble_marker(self, bubble_distance, bubble_angle):
        x = bubble_distance * np.cos(bubble_angle)
        y = bubble_distance * np.sin(bubble_angle)

        # 거리 기반 동적 반경
        scale_factor = self.calculate_scale_factor(bubble_distance)
        dynamic_bubble_radius = self.min_bubble_radius + \
            (self.max_bubble_radius - self.min_bubble_radius) * scale_factor

        marker = Marker()
        marker.header.frame_id = "ego_racecar/laser"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bubble_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = dynamic_bubble_radius * 2.0
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)
        self.bubble_marker_pub.publish(marker)

    # ---------------- Helpers ----------------
    def calculate_scale_factor(self, distance):
        """0~1 스케일: 가까울수록 작게, 멀수록 크게"""
        if distance <= self.min_bubble_radius:
            return 0.0
        if distance >= self.bubble_distance_threshold:
            return 1.0
        return (distance - self.min_bubble_radius) / (self.bubble_distance_threshold - self.min_bubble_radius)

    def get_angle(self, index, angle_min, angle_increment):
        return angle_min + index * angle_increment

    def _fov_indices(self, data, deg_min, deg_max):
        n = len(data.ranges)
        i0 = int((np.radians(deg_min) - data.angle_min) / data.angle_increment)
        i1 = int((np.radians(deg_max) - data.angle_min) / data.angle_increment)
        i0 = max(0, min(n - 1, i0))
        i1 = max(0, min(n - 1, i1))
        if i0 > i1:
            i0, i1 = i1, i0
        return i0, i1

    # ---------------- Disparity / Bubble / Gap ----------------
    def find_disparities_valid(self, ranges, threshold, start_idx, end_idx):
        """무시구간(0.0) 제외하고 유효 FOV에서만 disparity 계산"""
        disparities = []
        for i in range(max(start_idx, 0), min(end_idx, len(ranges) - 2) + 1):
            a, b = ranges[i], ranges[i + 1]
            if a == 0.0 or b == 0.0:
                continue
            if abs(b - a) > threshold:
                disparities.append(i if (b - a) > 0 else i + 1)
        return disparities

    def create_safety_bubble(self, ranges, disparities, angle_min, angle_increment):
        """가장 가까운 disparity 지점을 중심으로 버블 1개 생성"""
        proc_ranges = ranges.copy()
        if not disparities:
            return proc_ranges, None, None

        valid_disparities = [i for i in disparities if ranges[i] > 0.0]
        if not valid_disparities:
            return proc_ranges, None, None

        bubble_index = min(valid_disparities, key=lambda i: ranges[i])
        bubble_distance = ranges[bubble_index]
        bubble_angle = self.get_angle(bubble_index, angle_min, angle_increment)

        # 거리 기반 반경
        bubble_radius = self.calculate_bubble_radius(bubble_distance)

        # 각도 스프레드
        if bubble_distance > self.min_bubble_radius:
            theta = 2.0 * np.arcsin(min(1.0, bubble_radius / max(bubble_distance, 1e-6)))
        else:
            theta = np.pi
        bubble_size = int(theta / angle_increment / 2.0)

        s = max(0, bubble_index - bubble_size)
        e = min(len(ranges) - 1, bubble_index + bubble_size)
        proc_ranges[s:e + 1] = 0.0

        return proc_ranges, bubble_distance, bubble_angle

    def calculate_bubble_radius(self, bubble_distance):
        if bubble_distance >= self.bubble_distance_threshold:
            bubble_radius = self.max_bubble_radius
        else:
            bubble_radius = ((bubble_distance / self.bubble_distance_threshold) *
                             (self.max_bubble_radius - self.min_bubble_radius) + self.min_bubble_radius)
        return float(np.clip(bubble_radius, self.min_bubble_radius, self.max_bubble_radius))

    def set_nearest_point_bubble(self, ranges, angle_increment):
        """정면 최단거리 포인트에도 항상 버블 1개 생성(정면 들이대기 방지)"""
        safe = np.where(ranges == 0.0, np.inf, ranges)
        if not np.any(np.isfinite(safe)):
            return ranges  # 유효 없음
        idx = int(np.argmin(safe))
        dist = float(safe[idx])

        # 작은 각 근사: theta ≈ 2*atan(R/(2D))
        R = self.min_bubble_radius
        theta = 2.0 * np.arctan(R / max(2.0 * dist, 1e-6))
        bubble_size = max(1, int(theta / angle_increment))

        s = max(0, idx - bubble_size)
        e = min(len(ranges) - 1, idx + bubble_size)
        ranges[s:e + 1] = 0.0
        return ranges

    def find_max_gap(self, proc_ranges, disparities):
        """0.0/디스패리티 제외하고 가장 '길고 멀리' 열린 gap 선택"""
        exclusion_indices = set(np.where(proc_ranges == 0.0)[0])
        exclusion_indices.update(disparities)
        valid_indices = [i for i in range(len(proc_ranges))
                         if i not in exclusion_indices and proc_ranges[i] >= self.min_gap_distance]
        if not valid_indices:
            return None, None

        gaps = np.split(valid_indices, np.where(np.diff(valid_indices) > 1)[0] + 1)
        filtered = [g for g in gaps if len(g) >= self.min_gap_size]
        if not filtered:
            return None, None

        # 평균거리 최대 gap 선택
        max_gap = max(filtered, key=lambda g: np.mean(proc_ranges[g]))
        return int(max_gap[0]), int(max_gap[-1])

    # ---------------- 메인 콜백 ----------------
    def scan_callback(self, data: LaserScan):
        ranges = np.array(data.ranges, dtype=np.float32)
        ranges[np.isinf(ranges)] = self.lidar_cap
        ranges[np.isnan(ranges)] = 0.0

        # ① FOV 안전 클램프
        angle_start, angle_end = self._fov_indices(data, self.fov_deg_min, self.fov_deg_max)
        # FOV 밖 무시
        ranges[:angle_start] = 0.0
        ranges[angle_end + 1:] = 0.0

        # ① (계속) 무시구간 제외한 유효 FOV에서만 disparity
        disparities = self.find_disparities_valid(ranges, self.disparity_threshold, angle_start, angle_end)

        # disparity 기반 버블
        proc_ranges, bubble_distance, bubble_angle = self.create_safety_bubble(
            ranges, disparities, data.angle_min, data.angle_increment)

        # ② 최단거리 포인트에도 버블 추가
        proc_ranges = self.set_nearest_point_bubble(proc_ranges, data.angle_increment)

        # 버블 마커 (있을 때만)
        if bubble_distance is not None and bubble_angle is not None and np.isfinite(bubble_distance):
            self.publish_closest_bubble_marker(bubble_distance, bubble_angle)

        # ③ gap 탐색
        start_index, end_index = self.find_max_gap(proc_ranges, disparities)
        if start_index is None or end_index is None or end_index < start_index:
            self.get_logger().info("No valid gaps available for navigation.")
            # 제동
            self._reactive_control(steering_angle=0.0, speed_hint=self.min_speed)
            return

        # ③ best point = (중앙 ⊕ 거리^2 가중 중앙)
        gap_indices = np.arange(start_index, end_index + 1)
        gap_ranges = proc_ranges[gap_indices]
        w = np.clip(gap_ranges, 0.0, None) ** 2
        if np.sum(w) > 1e-6:
            weighted_idx = int(np.average(gap_indices, weights=w))
        else:
            weighted_idx = int((start_index + end_index) // 2)
        mid_idx = int((start_index + end_index) // 2)
        best_point = int(np.clip(int(0.5 * weighted_idx + 0.5 * mid_idx), start_index, end_index))
        best_point_distance = float(proc_ranges[best_point])

        # 마커 표시
        self.publish_best_point_marker(best_point, best_point_distance, data.angle_min, data.angle_increment)

        # 목표 조향각
        best_point_angle = data.angle_min + best_point * data.angle_increment

        # ④ 속도/조향 완만화 + 레이트 제한 포함
        speed_cmd = self.calculate_speed(best_point_angle, best_point_distance)
        self._reactive_control(steering_angle=best_point_angle, speed_hint=speed_cmd)

    # ---------------- 속도/제어 ----------------
    def calculate_speed(self, steering_angle, distance):
        """각도가 클수록 감속. 거리 기반 상향은 과감히 배제(안정성 우선)."""
        steer_lim = np.radians(self.max_steer_deg)
        angle_factor = min(1.0, abs(steering_angle) / max(steer_lim, 1e-6))
        angle_speed = self.max_speed - self.angle_gain_speed * angle_factor
        return float(np.clip(angle_speed, self.min_speed, self.max_speed))

    def _reactive_control(self, steering_angle, speed_hint):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/base_link'

        # 조향 제한
        steer_lim = np.radians(self.max_steer_deg)
        steer_cmd = float(np.clip(steering_angle, -steer_lim, steer_lim))
        speed_cmd = float(np.clip(speed_hint, self.min_speed, self.max_speed))

        # EMA
        speed_f = self.alpha_v * speed_cmd + (1.0 - self.alpha_v) * self.prev_speed
        steer_f = self.alpha_s * steer_cmd + (1.0 - self.alpha_s) * self.prev_steer

        # 레이트 제한
        dv = np.clip(speed_f - self.prev_speed, -self.max_dv, self.max_dv)
        ds = np.clip(steer_f - self.prev_steer, -np.radians(self.max_ds_deg), np.radians(self.max_ds_deg))
        speed_out = self.prev_speed + dv
        steer_out = self.prev_steer + ds

        msg.drive.steering_angle = float(steer_out)
        msg.drive.speed = float(speed_out)

        self.drive_pub.publish(msg)
        self.prev_speed = speed_out
        self.prev_steer = steer_out

        self.get_logger().info(f"Steering Angle: {np.degrees(steer_out):.2f} deg, Speed: {speed_out:.2f} m/s")


def main(args=None):
    rclpy.init(args=args)
    print("Reactive Gap Follower (with 4 safety patches) Initialized")
    node = GapFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()