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
        self.lidar_sub = self.create_subscription(LaserScan, self.lidarscan_topic, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.best_point_marker_pub = self.create_publisher(Marker, self.best_point_marker_topic, 10)
        self.bubble_marker_pub = self.create_publisher(Marker, self.bubble_marker_topic, 10)

        # Constants
        self.PI = 3.1415927
        self.LIDAR_RANGE_CAP = 10.0

        # Speed logic 
        self.max_speed = 8.5    # m/s (상황에 따라 조정)
        self.min_speed = 4.5     # m/s
        self.max_steer_deg = 20.0  # 물리적 조향 한계(안전 클리핑)

        # Gap/Disparity/Fine-gap parameters 
        self.disparity_threshold = 2.5     # 디스패리티 탐지 임계
        self.extend_num = 1                # 디스패리티 확장 폭
        self.fine_threshold = 2.0          # fine gap: 최소 거리 임계
        self.fine_min_length = 5           # fine gap: 최소 인덱스 길이
        self.fine_min_range = 2.5          # fine gap: 길이 기반 필터
        self.fine_min_width = 2.0 # 0.5          # fine gap: 실제 폭(m) 조건

        # FOV 선택: 주행 의사결정 범위(-85~85deg), 버블 전용(-45~45deg)
        self.deg_min = -60
        self.deg_max = 60
        self.deg_min_bubble = -45
        self.deg_max_bubble = 45

        # Safety bubble 
        self.fixed_bubble_radius = 0.35  # 버블 반경(m) – 마스킹 계산용
        self.min_bubble_radius = 0.02     # RViz 표시용 동적 반경 최소치
        self.max_bubble_radius = 0.4     # RViz 표시용 동적 반경 최대치
        self.bubble_distance_threshold = 10.0  # 동적 표시 스케일 범위

        self.get_logger().info("Merged GapFollow Node Initialized")

    # ---------- Utilities ----------
    def preprocess_lidar(self, arr):
        arr = arr.copy()
        arr[np.isinf(arr)] = self.LIDAR_RANGE_CAP * 3  # 먼 곳은 크게
        arr[np.isnan(arr)] = 0.0
        arr[arr > self.LIDAR_RANGE_CAP] = self.LIDAR_RANGE_CAP
        return arr

    def find_n_extend_disparity(self, ranges, disparity_threshold, extend_num):
        # 1) disparity 찾기
        ranges = ranges.copy()
        disps = []
        for i in range(1, len(ranges)):
            if abs(ranges[i] - ranges[i - 1]) > disparity_threshold:
                disps.append((i, ranges[i]))
        # 2) 확장
        for index, _ in disps:
            if ranges[index] > ranges[index - 1]:
                base = ranges[index - 1]
                for j in range(extend_num):
                    k = index + j
                    if k < len(ranges):
                        ranges[k] = base
            else:
                base = ranges[index]
                for j in range(extend_num):
                    k = index - j - 1
                    if k >= 0:
                        ranges[k] = base
        return ranges

    def set_safety_bubble(self, closest_idx, proc_ranges, angle_increment, closest_dist):
        # 고정 반경을 각도로 변환해 마스킹
        if closest_dist <= 0.0:
            return proc_ranges
        bubble_angle = 2 * np.arctan(self.fixed_bubble_radius / (2 * max(closest_dist, 1e-3)))
        bubble_index = max(1, int(bubble_angle / angle_increment))
        s = max(0, closest_idx - bubble_index)
        e = min(len(proc_ranges) - 1, closest_idx + bubble_index)
        proc_ranges[s:e + 1] = 0.0
        return proc_ranges

    def find_max_gap(self, ranges):
        masked = np.ma.masked_where(ranges <= 0.0, ranges)
        slices = np.ma.notmasked_contiguous(masked)
        if not slices:
            return 0, len(ranges) - 1
        largest = max(slices, key=lambda sl: sl.stop - sl.start)
        return largest.start, largest.stop

    def find_fine_gap(self, ranges, threshold, min_length, min_range, min_width, angle_increment):
        gaps, s, cnt = [], None, 0
        for i in range(len(ranges)):
            if ranges[i] > threshold:
                if s is None:
                    s = i
                cnt += 1
            else:
                if cnt >= min_length:
                    gaps.append(list(range(s, i)))
                s, cnt = None, 0
        if cnt >= min_length:
            gaps.append(list(range(s, len(ranges))))
        if not gaps:
            return None, None

        valid = []
        for g in gaps:
            g_s, g_e = g[0], g[-1]
            r1, r2 = ranges[g_s], ranges[g_e]
            theta = (g_e - g_s) * angle_increment
            gap_width = np.sqrt(r1**2 + r2**2 - 2*r1*r2*np.cos(theta))
            if gap_width >= min_width:
                valid.append(g)
        if not valid:
            return None, None

        long_gaps = [g for g in valid if len(g) >= min_range]
        if not long_gaps:
            opt = max(valid, key=lambda g: len(g))
        else:
            opt = max(long_gaps, key=lambda g: max(ranges[idx] for idx in g))
        return (opt[0], opt[-1]) if opt else (None, None)

    def find_best_point_avg(self, start_idx, end_idx, ranges, window_size=20):
        sub = ranges[start_idx:end_idx]
        if len(sub) == 0:
            return (start_idx + end_idx) // 2
        kernel = np.ones(window_size) / window_size
        smoothed = np.convolve(sub, kernel, mode='same')
        return smoothed.argmax() + start_idx

    def calculate_scale_factor(self, distance):
        # RViz 마커 크기 스케일
        if distance <= self.min_bubble_radius:
            return 0.0
        if distance >= self.bubble_distance_threshold:
            return 1.0
        return (distance - self.min_bubble_radius) / (self.bubble_distance_threshold - self.min_bubble_radius)

    # ---------- Markers ----------
    def publish_best_point_marker(self, best_idx, best_dist, angle_min, angle_increment):
        ang = angle_min + best_idx * angle_increment
        x = best_dist * np.cos(ang)
        y = best_dist * np.sin(ang)
        m = Marker()
        m.header.frame_id = "ego_racecar/laser"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "best_point"; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        self.best_point_marker_pub.publish(m)

    def publish_bubble_marker(self, idx, dist, angle_min, angle_increment):
        ang = angle_min + idx * angle_increment
        x = dist * np.cos(ang)
        y = dist * np.sin(ang)
        # 표시용 동적 반경
        scale = self.calculate_scale_factor(dist)
        dyn_r = self.min_bubble_radius + (self.max_bubble_radius - self.min_bubble_radius) * scale
        m = Marker()
        m.header.frame_id = "ego_racecar/laser"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "bubble_point"; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = dyn_r * 2.0
        m.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)
        self.bubble_marker_pub.publish(m)

    # ---------- Speed / Control ----------
    def calculate_speed(self, steering_angle_rad, distance):
        # 조향 큰 경우 감속
        angle_speed = self.max_speed - abs(steering_angle_rad * 13.0) # 11.0)
        angle_speed = np.clip(angle_speed, self.min_speed, self.max_speed)
        # 베스트 포인트까지 거리 기반 보정
        if distance > 3.0: # 4.0:
            dist_speed = max(self.max_speed, distance / 0.95) # 0.95
        else:
            dist_speed = min(self.max_speed * 1.5, distance / 0.95) # 1.5
        return float(min(angle_speed, dist_speed))

    def publish_drive(self, steering_angle_rad, speed):
        # 물리적 조향각 한계 클리핑
        max_rad = np.radians(self.max_steer_deg)
        steering_angle_rad = float(np.clip(steering_angle_rad, -max_rad, max_rad))
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/base_link'
        msg.drive.steering_angle = steering_angle_rad
        msg.drive.speed = float(np.clip(speed, 0.0, self.max_speed * 1.5))
        self.drive_pub.publish(msg)
        self.get_logger().info(f"Steering: {np.degrees(steering_angle_rad):.2f} deg | Speed: {msg.drive.speed:.2f} m/s")

    # ---------- Main Callback ----------
    def lidar_callback(self, data: LaserScan):
        ranges_full = np.array(data.ranges, dtype=float)

        # FOV 인덱스 계산
        def deg_to_idx(deg):
            return int((np.radians(deg) - data.angle_min) / data.angle_increment)

        min_idx = deg_to_idx(self.deg_min)
        max_idx = deg_to_idx(self.deg_max)
        min_idx_bub = deg_to_idx(self.deg_min_bubble)
        max_idx_bub = deg_to_idx(self.deg_max_bubble)

        # 범위 외 마스킹
        ranges_full[:min_idx] = 0.0
        ranges_full[max_idx:] = 0.0

        # 전처리
        proc = self.preprocess_lidar(ranges_full[min_idx:max_idx])
        proc_bub = self.preprocess_lidar(ranges_full[min_idx_bub:max_idx_bub])

        # 디스패리티 확장
        proc = self.find_n_extend_disparity(proc, self.disparity_threshold, self.extend_num)

        # 가장 가까운 점 (버블 기준은 중앙 좁은 FOV에서)
        if len(proc) == 0 or len(proc_bub) == 0:
            self.publish_drive(0.0, 0.0)
            return
        closest_idx_bub_local = int(np.argmin(proc_bub))
        closest_dist_bub = float(proc_bub[closest_idx_bub_local])
        closest_idx_bub_global = closest_idx_bub_local + (min_idx_bub - min_idx)

        # 안전 버블 마스킹
        proc = self.set_safety_bubble(closest_idx_bub_global, proc, data.angle_increment, closest_dist_bub)

        # 최대 갭 + fine gap 
        start_m, end_m = self.find_max_gap(proc)
        start_f, end_f = self.find_fine_gap(proc, self.fine_threshold, self.fine_min_length,
                                            self.fine_min_range, self.fine_min_width, data.angle_increment)

        # 베스트 포인트 결정: fine 우선, 없으면 평균 윈도우 최대
        if start_f is not None and end_f is not None:
            best_idx_local = (start_f + end_f) // 2
        else:
            best_idx_local = self.find_best_point_avg(start_m, end_m, proc, window_size=20)

        # 전역 인덱스 & 각도
        best_idx_global = best_idx_local + min_idx
        best_angle = data.angle_min + best_idx_global * data.angle_increment
        best_dist = float(proc[best_idx_local]) if 0 <= best_idx_local < len(proc) else 0.0

        # RViz 마커
        self.publish_best_point_marker(best_idx_global, best_dist, data.angle_min, data.angle_increment)
        self.publish_bubble_marker(closest_idx_bub_local + min_idx_bub, closest_dist_bub, data.angle_min, data.angle_increment)

        # 속도 계산 (2번)
        speed = self.calculate_speed(best_angle, best_dist)
        self.publish_drive(best_angle, speed)


def main(args=None):
    rclpy.init(args=args)
    node = GapFollow()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()