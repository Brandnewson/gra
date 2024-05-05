#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection3DArray
from ultralytics_ros.msg import ConeDetection  

# A dictionary to map id values to human-readable names
CONE_TYPES = {
    0: "blue_cones",
    1: "large_orange_cones",
    2: "orange_cones",
    3: "unknown_cones",
    4: "yellow_cones"
}

class ConeSorter:
    def __init__(self):
        rospy.init_node('cone_sorter', anonymous=False)
        
        self.cone_lists = {name: [] for name in CONE_TYPES.values()}  
        
        self.sub = rospy.Subscriber("/transformed_yolo_3d_result", Detection3DArray, self.callback)
        self.pub = rospy.Publisher("/cone_list", ConeDetection, queue_size=10)

        # Initialize EKF dictionary for cones
        self.cone_filters = {}

    def is_new_cone(self, new_cone, existing_cones):
        for cone in existing_cones:
            if np.linalg.norm(np.array([cone.x, cone.y]) - np.array([new_cone.x, new_cone.y])) < 1.0:   # Avoid redundant cones. Value is cone proximity threshold in meters
                return False
        return True

    def callback(self, data):
        for detection in data.detections:
            # Skip detections with invalid size criteria
            size = detection.bbox.size
            if np.isinf(size.x) or np.isinf(size.y) or np.isinf(size.z):
                rospy.loginfo("Skipping detection with infinite size.")
                continue
            if size.x > 0.5 or size.y > 0.5 or size.z > 0.5:
                rospy.loginfo("Skipping detection with size larger than 0.5.")
                continue

            cone_type = CONE_TYPES[detection.results[0].id]  # Map the id to the human-readable name
            new_cone = Point(detection.bbox.center.position.x, detection.bbox.center.position.y, 0)  # Z value is not used

            # Update or initialize EKF for the cone
            if cone_type not in self.cone_filters:
                self.initialize_filter(cone_type)

            # Extract measurement from detection
            measurement = np.array([new_cone.x, new_cone.y])

            # Update EKF with measurement
            self.update_filter(cone_type, measurement)

        self.publish_cones()
    def initialize_filter(self, cone_type):
        # Initialize EKF for the cone
        ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)  # 2D position and velocity
        # Initialize state and covariance matrix
        ekf.x = np.array([0, 0, 0, 0])  # Initial state (position and velocity)
        ekf.P = np.eye(4)  # Initial covariance matrix
        # Set process and measurement noise covariance matrices
        ekf.Q *= 0.01  # Process noise covariance
        ekf.R *= 0.1  # Measurement noise covariance
        self.cone_filters[cone_type] = ekf

    def update_filter(self, cone_type, measurement):
        # Get the corresponding EKF for the cone
        ekf = self.cone_filters[cone_type]
        
        # Prediction step: Predict next state based on motion model
        ekf.predict()
        
        # Correction step: Update state based on measurement
        ekf.update(measurement)

        # Extract updated position from EKF state
        updated_position = ekf.x[:2]
        # Use updated position for further processing or publishing
        self.cone_lists[cone_type].append(Point(updated_position[0], updated_position[1], 0))


    def publish_cones(self):
        cone_list_msg = ConeDetection()
        cone_list_msg.header.stamp = rospy.Time.now()

        for cone_type, points in self.cone_lists.items():
            setattr(cone_list_msg, cone_type, points)
        
        self.pub.publish(cone_list_msg)

if __name__ == '__main__':
    cone_sorter = ConeSorter()
    rospy.spin()
