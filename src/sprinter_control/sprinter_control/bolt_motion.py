import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class BoltMotion(Node):
    """
    Publishes TF world->base_link to move 0→60 m with Bolt-like timing.
    Resets to x=0 at the start of each new run; repeats 'repeats' times, then holds at 60 m.
    Logs each run start/finish and final summary.
    """

    def __init__(self):
        super().__init__("bolt_motion")
        self.br = TransformBroadcaster(self)
        self.declare_parameter("hip_z", 0.90) #Höhe Hüfte
        self.declare_parameter("repeats", 5) #Wiederholungen
        self.z = float(self.get_parameter("hip_z").value)
        self.repeats = int(self.get_parameter("repeats").value)

        # Bolt 0..60 m (t[s], x[m])
        self.path = [(0.00, 0.0), (2.89, 20.0), (4.64, 40.0), (6.31, 60.0)]
        self.cycle_T = self.path[-1][0]

        self.run_idx = 1
        self.done = False
        self.started = False
        self.run_start_time = None

        self.timer = self.create_timer(0.01, self.step) #Initalisiert timer in 100 ms schritten

    def step(self):
        now = self.get_clock().now()

        if not self.started:
            self.started = True
            self.run_start_time = now
            self.get_logger().info("Run 1 started at x=0.0 m (camera following).") #Kamera folgt dem Renner

        if self.done:
            x = 60.0  # hold at finish after final run / Renner steht an der Ziellinie (60m)
        else:
            t_in = (now - self.run_start_time).nanoseconds * 1e-9
            if t_in >= self.cycle_T:
                self.get_logger().info(
                    f"Run {self.run_idx} finished at x=60.0 m (t_in={t_in:.2f}s)."
                )
                self.run_idx += 1
                if self.run_idx > self.repeats:
                    self.done = True
                    self.get_logger().info(
                        f"All {self.repeats} runs complete. Holding at finish line (x=60.0 m)."
                    )
                    x = 60.0
                else:
                    self.get_logger().info(
                        f"Resetting to start. Run {self.run_idx} started at x=0.0 m." #Start am Angang (0m)
                    )
                    self.run_start_time = now
                    x = 0.0
            else:
                x = self.interpolate_cycle(t_in)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "world" #Frame im Simulator ist World
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = float(self.z)  # USE self.z
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.br.sendTransform(tf)

    def interpolate_cycle(self, t: float) -> float:
        if t <= self.path[0][0]:
            return self.path[0][1]
        for (t0, x0), (t1, x1) in zip(self.path, self.path[1:]):
            if t < t1:
                r = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return x0 + r * (x1 - x0)
        return self.path[-1][1]


def main():
    rclpy.init()
    node = BoltMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
