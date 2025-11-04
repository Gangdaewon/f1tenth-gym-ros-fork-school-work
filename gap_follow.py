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
       
        # Topics for publishing and subscribing
        self.lidarscan_topic = '/scan'
        self.drive_topic = '/drive'
        self.best_point_marker_topic = '/best_point_marker'
        self.bubble_marker_topic = '/bubble_point_marker'
       
        # Create subscribers and publishers
        self.lidar_sub = self.create_subscription(
            LaserScan, self.lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        self.best_point_marker_pub = self.create_publisher(
            Marker, self.best_point_marker_topic, 10)
        self.bubble_marker_pub = self.create_publisher(
            Marker, self.bubble_marker_topic, 10)
     
        # Define additional variables
        self.disparity_threshold = 0.5         # 라이다 인덱스 i와 i+1의 거리차가 0.5m 이상이면 disparity로 간주
        
        # Multi-bubble parameters (레이싱 최적화)
        self.bubble_creation_distance = 3.0    # 이 거리 이내 모든 장애물에 버블 생성 (meters)
        self.min_bubble_radius = 0.1          # Minimum bubble radius (meters) - 증가
        self.max_bubble_radius = 0.4           # Maximum bubble radius (meters) - 증가
        self.bubble_distance_threshold = 5.0  # Distance at which bubble is maximum size
        self.speed_bubble_factor = 0.5         # 속도에 따른 버블 크기 증가 계수
        
        # Gap finding parameters
        self.min_gap_distance = 1.5            # 라이다 값 거리가 1.5m 이하들은 버림(너무 가까워서 조향각이 안 나옴)
        self.min_gap_size = 15                 # Gap과 Gap 사이가 라이다 인덱스 22개 이하면 버림 그 구간 무시(너무 좁음)
        
        # Steering control parameters (강건한 제어)
        self.prev_steering_angle = 0.0         # 이전 조향각 (스무딩용)
        self.steering_smoothing_factor = 0.3   # 조향각 스무딩 계수 (0=최대 스무딩, 1=스무딩 없음)
        self.max_steering_rate = 0.5           # 최대 조향 각속도 (rad/iteration)
        
        # Speed control parameters (강건한 속도 제어)
        self.max_speed = 6.0                   # 최대 속도 (m/s)
        self.min_speed = 2.5                   # 최소 속도 (m/s)
        self.prev_speed = 0.0                  # 이전 속도 (가속도 제한용)
        self.max_acceleration = 3.0            # 최대 가속도 (m/s²)
        self.max_deceleration = 4.0            # 최대 감속도 (m/s²)
        self.current_speed = 0.0               # 현재 속도 추정값
       
        self.get_logger().info("GapFollow Node Initialized Successfully")

    def publish_best_point_marker(self, best_point_idx, best_point_distance, angle_min, angle_increment):
        """Publish a visualization marker for the best point in RViz"""
        # Calculate angle of best point
        best_point_angle = angle_min + best_point_idx * angle_increment

        # Convert polar coordinates to Cartesian for visualization
        x = best_point_distance * np.cos(best_point_angle)
        y = best_point_distance * np.sin(best_point_angle)

        # Create the Marker message
        marker = Marker()
        marker.header.frame_id = "ego_racecar/laser"  # Set the reference frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "best_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # LiDAR is in 2D plane, so z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scale the marker (make it a small sphere)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color of the marker
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green, opaque

        # Publish the Marker
        self.best_point_marker_pub.publish(marker)
       
    def publish_bubble_markers(self, bubble_info_list):
        """Publish visualization markers for all bubbles in RViz"""
        if not bubble_info_list:
            return
            
        for i, bubble_info in enumerate(bubble_info_list):
            bubble_distance = bubble_info['distance']
            bubble_angle = bubble_info['angle']
            
            # Convert polar coordinates to Cartesian for visualization
            x = bubble_distance * np.cos(bubble_angle)
            y = bubble_distance * np.sin(bubble_angle)

            # Calculate scale factor based on distance
            scale_factor = self.calculate_scale_factor(bubble_distance)

            # Adjust bubble size based on scaling factor
            dynamic_bubble_radius = self.min_bubble_radius + (self.max_bubble_radius - self.min_bubble_radius) * scale_factor

            # Create the Marker message
            marker = Marker()
            marker.header.frame_id = "ego_racecar/laser"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "bubble_points"
            marker.id = i  # Unique ID for each bubble
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Scale the marker (make it a sphere with dynamic radius)
            marker.scale.x = dynamic_bubble_radius * 2
            marker.scale.y = dynamic_bubble_radius * 2
            marker.scale.z = dynamic_bubble_radius * 2

            # Set the color of the marker (gradient from red to blue based on distance)
            color_ratio = min(1.0, bubble_distance / 5.0)
            marker.color = ColorRGBA(r=1.0 - color_ratio, g=0.0, b=color_ratio, a=0.6)

            # Publish the Marker
            self.bubble_marker_pub.publish(marker)
           
    def calculate_scale_factor(self, distance):
        """Calculate scale factor based on distance (0 to 1)"""
        if distance <= self.min_bubble_radius:
            return 0.0
        elif distance >= self.bubble_distance_threshold:
            return 1.0
        else:
            return (distance - self.min_bubble_radius) / (self.bubble_distance_threshold - self.min_bubble_radius)
   
    def find_disparities(self, ranges, threshold):
        """Find disparities (sudden changes) in the LiDAR distance data"""
        disparities = []
        for i in range(len(ranges) - 1):
            diff = ranges[i + 1] - ranges[i]
            if abs(diff) > threshold:
                disparities.append(i if diff > 0 else i + 1)
        return disparities
   
    def create_safety_bubbles(self, ranges, disparities, angle_min, angle_increment):
        """
        Create safety bubbles around multiple obstacles within creation distance
        - Creates bubbles for all obstacles closer than bubble_creation_distance
        - Bubble size adapts based on distance and current speed
        - Returns processed ranges and list of bubble information
        """
        proc_ranges = ranges.copy()
        bubble_info_list = []
        
        if not disparities:
            self.get_logger().debug("No disparities detected.")
            return proc_ranges, bubble_info_list

        # Filter disparities: valid distance and within creation distance threshold
        close_disparities = [i for i in disparities 
                            if ranges[i] > 0.0 and ranges[i] < self.bubble_creation_distance]
        
        if not close_disparities:
            self.get_logger().debug(f"No disparities within {self.bubble_creation_distance}m threshold.")
            return proc_ranges, bubble_info_list

        # Create bubbles for all close disparities (다중 버블)
        for bubble_index in close_disparities:
            bubble_distance = ranges[bubble_index]

            # Calculate bubble radius based on distance and speed
            bubble_radius = self.calculate_dynamic_bubble_radius(bubble_distance, self.current_speed)

            # Calculate bubble angle
            bubble_angle = self.get_angle(bubble_index, angle_min, angle_increment)

            # Calculate angular spread based on bubble_radius and distance
            if bubble_distance > self.min_bubble_radius:
                # Prevent domain error in arcsin by ensuring R/D <= 1
                theta = 2 * np.arcsin(min(1.0, bubble_radius / bubble_distance))
            else:
                theta = np.pi  # Very close obstacle, cover all

            bubble_size = int(theta / angle_increment / 2)  # Number of indices on one side

            start_index = max(0, bubble_index - bubble_size)
            end_index = min(len(ranges) - 1, bubble_index + bubble_size)
            proc_ranges[start_index:end_index + 1] = 0.0  # Ignore bubble area

            self.get_logger().debug(
                f"Bubble {len(bubble_info_list)}: Dist={bubble_distance:.2f}m, "
                f"Radius={bubble_radius:.2f}m, Angle={np.degrees(bubble_angle):.1f}°"
            )
            
            # Store bubble information
            bubble_info_list.append({
                'distance': bubble_distance,
                'angle': bubble_angle,
                'radius': bubble_radius,
                'index': bubble_index
            })

        return proc_ranges, bubble_info_list
    
    def calculate_dynamic_bubble_radius(self, bubble_distance, current_speed):
        """
        Calculate bubble radius based on distance and current speed
        - Higher speed = larger bubble for safety margin
        - Closer obstacle = appropriate sizing
        """
        # Base radius calculation (distance-based)
        if bubble_distance >= self.bubble_distance_threshold:
            base_radius = self.max_bubble_radius
        else:
            base_radius = ((bubble_distance / self.bubble_distance_threshold) *
                          (self.max_bubble_radius - self.min_bubble_radius) + self.min_bubble_radius)
        
        # Speed factor: increase bubble size at high speed
        speed_ratio = current_speed / self.max_speed  # 0.0 ~ 1.0
        speed_multiplier = 1.0 + (self.speed_bubble_factor * speed_ratio)
        
        # Apply speed factor
        bubble_radius = base_radius * speed_multiplier
        
        # Clip to safe range
        bubble_radius = np.clip(bubble_radius, self.min_bubble_radius, self.max_bubble_radius * 1.5)
        
        return bubble_radius
   
    def calculate_bubble_radius(self, bubble_distance):
        """Calculate the bubble radius based on the distance to the obstacle (legacy method)"""
        # Bubble radius increases when far from obstacle and decreases when close
        if bubble_distance >= self.bubble_distance_threshold:
            bubble_radius = self.max_bubble_radius
        else:
            bubble_radius = ((bubble_distance / self.bubble_distance_threshold) *
                             (self.max_bubble_radius - self.min_bubble_radius) + self.min_bubble_radius)
        bubble_radius = np.clip(bubble_radius, self.min_bubble_radius, self.max_bubble_radius)
        return bubble_radius
   
    def find_max_gap(self, proc_ranges, disparities):
        """
        Find the largest gap in the processed ranges, excluding disparity regions
        """
        exclusion_indices = set(np.where(proc_ranges == 0.0)[0])
        exclusion_indices.update(disparities)
        valid_indices = [i for i in range(len(proc_ranges)) if i not in exclusion_indices and proc_ranges[i] >= self.min_gap_distance]

        if not valid_indices:
            self.get_logger().debug("No valid gaps found.")
            return None, None

        gaps = np.split(valid_indices, np.where(np.diff(valid_indices) > 1)[0] + 1)
        filtered_gaps = [gap for gap in gaps if len(gap) >= self.min_gap_size]

        if not filtered_gaps:
            self.get_logger().debug("No gaps meet the size criteria.")
            return None, None

        # Select the gap with maximum average distance
        max_gap = max(filtered_gaps, key=lambda gap: np.mean(proc_ranges[gap]))
        start_index, end_index = max_gap[0], max_gap[-1]

        self.get_logger().debug(f"Max gap found: Start={start_index}, End={end_index}, Size={len(max_gap)}")
        return start_index, end_index
   
   
    def scan_callback(self, data):
        """Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message"""
        ranges = np.array(data.ranges)
        # Preprocess LiDAR scan ranges                
        ranges[np.isinf(ranges)] = 30.0  # Set infinite values to a max distance
        ranges[np.isnan(ranges)] = 0.0   # Set NaN values to zero
        angle_start = int((np.radians(-42.5) - data.angle_min) / data.angle_increment)
        angle_end = int((np.radians(42.5) - data.angle_min) / data.angle_increment)
        ranges[:angle_start] = 0.0
        ranges[angle_end:] = 0.0
       
        # Find disparities
        disparities = self.find_disparities(ranges, self.disparity_threshold)
        self.get_logger().debug(f"Detected Disparities: {disparities}")    

        # Create multiple safety bubbles (거리 및 속도 기반)
        proc_ranges, bubble_info_list = self.create_safety_bubbles(
            ranges, disparities, data.angle_min, data.angle_increment)

        # Publish all bubbles as Markers
        self.publish_bubble_markers(bubble_info_list)
        
        self.get_logger().debug(f"Created {len(bubble_info_list)} safety bubbles")

        # Find the largest valid gap
        start_index, end_index = self.find_max_gap(proc_ranges, disparities)
        if start_index is not None and end_index is not None:
            # Calculate gap size for speed control
            gap_size = end_index - start_index + 1
            
            # Find the best point (lidar index) as the middle of the gap
            best_point = (start_index + end_index) // 2
            best_point_distance = proc_ranges[best_point]

            # Publish the best point as a Marker
            self.publish_best_point_marker(
                best_point, best_point_distance, data.angle_min, data.angle_increment)

            # Calculate the steering angle towards the best point
            best_point_angle = data.angle_min + best_point * data.angle_increment
            
            # Apply steering smoothing for stable control
            smoothed_steering_angle = self.apply_steering_smoothing(best_point_angle)

            # Calculate robust speed based on multiple factors
            speed = self.calculate_robust_speed(
                smoothed_steering_angle, 
                best_point_distance,
                gap_size
            )

            # Publish the drive command
            self.reactive_control(smoothed_steering_angle, speed)
            
            self.get_logger().debug(
                f"Gap size: {gap_size}, Best point dist: {best_point_distance:.2f}m, "
                f"Raw angle: {np.degrees(best_point_angle):.1f}°, "
                f"Smoothed angle: {np.degrees(smoothed_steering_angle):.1f}°, "
                f"Speed: {speed:.2f}m/s"
            )
        else:
            self.get_logger().info("No valid gaps available for navigation.")
            # If no valid gaps, gradually slow down (emergency stop)
            emergency_speed = max(0.0, self.prev_speed - self.max_deceleration * 0.1)
            self.prev_speed = emergency_speed
            self.current_speed = emergency_speed
            self.reactive_control(0.0, emergency_speed)
       
       
    def calculate_speed(self, steering_angle, distance):
        """Calculate the vehicle's speed based on steering angle and distance"""
        max_speed = 6.0  # Maximum speed (m/s)  # 7.2
        min_speed = 4.0 # Minimum speed (m/s)   # 4.5

        # Adjust speed based on steering angle (slower when steering sharply)
        angle_speed = max_speed - abs(steering_angle * 11 ) # * 2.0
        angle_speed = np.clip(angle_speed, min_speed, max_speed)

        # Adjust speed based on distance to the best point (slower when closer)
        if distance > 4.0:
            distance_speed = max(max_speed, distance / 0.95)
        else:
            distance_speed = min(max_speed * 1.5, distance / 0.95)
        speed = min(angle_speed, distance_speed)

        return speed
    
    def apply_steering_smoothing(self, target_steering_angle):
        """
        Apply exponential smoothing to steering angle to reduce oscillations
        - Prevents sudden steering changes
        - Maintains responsiveness for safety
        """
        # Exponential moving average
        smoothed_angle = (self.steering_smoothing_factor * target_steering_angle + 
                         (1.0 - self.steering_smoothing_factor) * self.prev_steering_angle)
        
        # Apply steering rate limit (angular velocity constraint)
        angle_diff = smoothed_angle - self.prev_steering_angle
        if abs(angle_diff) > self.max_steering_rate:
            smoothed_angle = self.prev_steering_angle + np.sign(angle_diff) * self.max_steering_rate
        
        # Update previous value
        self.prev_steering_angle = smoothed_angle
        
        return smoothed_angle
    
    def calculate_robust_speed(self, steering_angle, best_point_distance, gap_size):
        """
        Calculate safe speed based on:
        - Steering angle (curvature)
        - Distance to target
        - Gap width
        - Current speed (for acceleration limits)
        """
        # 1. Speed based on steering angle (curvature-based)
        # Sharper turns = lower speed
        abs_steering = abs(steering_angle)
        if abs_steering < 0.1:  # Nearly straight
            angle_speed = self.max_speed
        elif abs_steering < 0.3:  # Gentle curve
            angle_speed = self.max_speed * 0.85
        elif abs_steering < 0.5:  # Moderate curve
            angle_speed = self.max_speed * 0.7
        else:  # Sharp turn
            angle_speed = self.max_speed * 0.55
        
        # 2. Speed based on distance to best point
        # Closer obstacles = lower speed
        if best_point_distance > 5.0:
            distance_speed = self.max_speed
        elif best_point_distance > 3.0:
            distance_speed = self.max_speed * 0.85
        elif best_point_distance > 2.0:
            distance_speed = self.max_speed * 0.7
        else:
            distance_speed = self.min_speed
        
        # 3. Speed based on gap width
        # Narrow gaps = lower speed for safety
        if gap_size > 50:  # Wide gap
            gap_speed = self.max_speed
        elif gap_size > 30:  # Medium gap
            gap_speed = self.max_speed * 0.8
        else:  # Narrow gap
            gap_speed = self.max_speed * 0.65
        
        # Take minimum of all constraints
        target_speed = min(angle_speed, distance_speed, gap_speed)
        target_speed = np.clip(target_speed, self.min_speed, self.max_speed)
        
        # 4. Apply acceleration/deceleration limits
        speed_diff = target_speed - self.prev_speed
        
        # Assume ~10Hz update rate (0.1s)
        dt = 0.1
        
        if speed_diff > 0:  # Accelerating
            max_speed_change = self.max_acceleration * dt
        else:  # Decelerating
            max_speed_change = self.max_deceleration * dt
        
        if abs(speed_diff) > max_speed_change:
            final_speed = self.prev_speed + np.sign(speed_diff) * max_speed_change
        else:
            final_speed = target_speed
        
        # Update previous speed
        self.prev_speed = final_speed
        self.current_speed = final_speed  # Update current speed estimate
        
        return final_speed

    def reactive_control(self, steering_angle, speed):
        """Calculate and publish the steering angle and speed based on the best gap angle"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'ego_racecar/base_link'

        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed

        # Publish the message
        self.drive_pub.publish(drive_msg)
        self.get_logger().info(
            f"Steering Angle: {np.degrees(steering_angle):.2f} deg, Speed: {speed:.2f} m/s")


    def get_angle(self, index, angle_min, angle_increment):
        """Convert index to angle"""
        return angle_min + index * angle_increment


def main(args=None):
    rclpy.init(args=args)
    print("Reactive Gap Follower Node Initialized")
    reactive_node = GapFollow()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
