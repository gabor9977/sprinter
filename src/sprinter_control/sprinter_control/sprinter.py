#!/usr/bin/env python3
# Every line is commented for clarity yet kept compact.

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for nodes
from geometry_msgs.msg import PoseStamped, TransformStamped  # Pose and TF
from tf2_ros import TransformBroadcaster  # TF broadcaster
from std_msgs.msg import Float32, String  # Speed and FSM state
import math, time

class Sprinter(Node):
    def __init__(self):
        super().__init__("sprinter")
        self.br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, "sprint/sprinter_pose", 10)
        self.speed_pub = self.create_publisher(Float32, "sprint/sprinter_speed", 10)
        self.state_pub = self.create_publisher(String, "sprint/state", 10)

        self.cum_t = [0.00, 1.90, 2.88, 3.80, 4.63, 5.46, 6.29] #Zeiten für Strecken
        self.cum_x = [0, 10, 20, 30, 40, 50, 60] #Strecken
        self.seg_v = [
            (self.cum_x[i + 1] - self.cum_x[i]) / (self.cum_t[i + 1] - self.cum_t[i])
            for i in range(len(self.cum_x) - 1)
        ]

        self.run_count = 0 # Startet bei 0 Rudnen
        self.max_runs = 5 # Anzahl Runden welcher der Läufer rennt
        self.waiting = True
        self.t0 = None
        self.done = False
        self.wait_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.state = "Idle"
        self.timer = self.create_timer(0.01, self.step)
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def x_of_t(self, t):
        if t <= self.cum_t[0]:
            return 0.0
        if t >= self.cum_t[-1]:
            return self.cum_x[-1]
        for k in range(len(self.cum_t) - 1):
            if self.cum_t[k] <= t < self.cum_t[k + 1]:
                return self.cum_x[k] + self.seg_v[k] * (t - self.cum_t[k])
        return self.cum_x[-1]

    def v_of_t(self, t):
        if t <= self.cum_t[0] or t >= self.cum_t[-1]:
            return 0.0
        for k in range(len(self.cum_t) - 1):
            if self.cum_t[k] <= t < self.cum_t[k + 1]:
                return self.seg_v[k]
        return 0.0

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.waiting:
            if self.state != "Countdown":
                self.state = "Countdown"
                self.publish_state()
                self.get_logger().info(f"Run {self.run_count + 1} starting in 5 seconds...")
            if now - self.wait_start_time >= 5.0: #Läufer wartet 5s bis er anfängt zum springen
                self.t0 = now
                self.done = False
                self.waiting = False
                self.state = "Running"
                self.publish_state()
            else:
                return

        t = now - self.t0
        x = self.x_of_t(t)
        v = self.v_of_t(t)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "world" #öffnet die Simulation in der Umgebung World
        tf.child_frame_id = "sprinter_base" #Gesamter Sprint findet auf einer Fläche statt, welche im Code Sprinter_base definiert wurde
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.5
        tf.transform.rotation.x = tf.transform.rotation.y = tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.br.sendTransform(tf)

        p = PoseStamped()
        p.header.stamp = tf.header.stamp
        p.header.frame_id = "world" #öffnet die Umgebung im Format World
        p.pose.position.x = tf.transform.translation.x
        p.pose.position.y = 0.0
        p.pose.position.z = 0.5
        p.pose.orientation.w = 1.0
        self.pose_pub.publish(p)

        s = Float32()
        s.data = float(v)
        self.speed_pub.publish(s)

        if not self.done and t >= self.cum_t[-1]:
            self.done = True
            self.run_count += 1
            self.get_logger().info(f"Sprinter finished run {self.run_count} at t={t:.2f}s") #Sprinte ist Fertig
            self.state = "Finished"
            self.publish_state()
            if self.run_count >= self.max_runs:
                self.get_logger().info("All runs completed. Shutting down node.") #Nach allen Runden vom Sprinter, Programm stellt ab
                rclpy.shutdown()
            else:
                self.waiting = True
                self.wait_start_time = now
                self.state = "Ready"
                self.publish_state()

def main():
    rclpy.init()
    node = Sprinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()