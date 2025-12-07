#!/usr/bin/env python3
# Every line is commented; node publishes distance at 100 Hz.

import rclpy  # ROS 2 Python API
from rclpy.node import Node  # Base class
from geometry_msgs.msg import PoseStamped  # To read sprinter pose
from std_msgs.msg import Float32, String  # Distance and FSM state
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster  # Static TF
from geometry_msgs.msg import TransformStamped  # For static TF
import random

class DistanceSensor(Node):
    def __init__(self):
        super().__init__("distance_sensor")
        self.sensor_x = -1.0  # Sensor befindet sich 1m hinter der Startlinie, Startlinie = x
        self.sensor_y = 0.0
        self.sensor_z = 1.0
        self.latest_x = 0.0
        self.sub = self.create_subscription(
            PoseStamped, "sprint/sprinter_pose", self.on_pose, 10
        )
        self.pub = self.create_publisher(
            Float32, "sprint/distance", 10
        )
        self.state_pub = self.create_publisher(
            String, "sprint/state", 10
        )
        self.timer = self.create_timer(0.01, self.tick)  # Messergebnisse werden in 100 Hz abgerufen

        # Filter: store last N values for moving average
        self.filter_N = 5
        self.filter_buffer = []

        # Publish a static TF for world->sensor_base
        self.static_br = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = "world"
        st.child_frame_id = "sensor_base"
        st.transform.translation.x = float(self.sensor_x)
        st.transform.translation.y = float(self.sensor_y)
        st.transform.translation.z = float(self.sensor_z)
        st.transform.rotation.w = 1.0
        self.static_br.sendTransform(st)

    def on_pose(self, msg):
        self.latest_x = float(msg.pose.position.x)

    def tick(self):
        error = random.uniform(-0.03, 0.03) #Messgenauikeit von +3cm und -3cm
        d = max(0.0, self.latest_x - self.sensor_x + error)

        # Filter um die Messergebnisse zu glätten
        self.filter_buffer.append(d)
        if len(self.filter_buffer) > self.filter_N:
            self.filter_buffer.pop(0)
        filtered = sum(self.filter_buffer) / len(self.filter_buffer)

        out = Float32()
        out.data = filtered
        self.pub.publish(out)

def main():
    rclpy.init()
    n = DistanceSensor()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()