import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
 
class FinishLineMarkers(Node):
    """
    Publishes two BIG cylinder posts at the 60 m line, visible in RViz even if RViz starts late.
 
    Publishes on ALL common topics so you don't have to tweak RViz:
      - /finish_line                     (MarkerArray)
      - /visualization_marker_array      (MarkerArray)
      - /visualization_marker            (Marker)  [two markers sent individually]
 
    QoS is TRANSIENT_LOCAL + RELIABLE (latched).
    Frame is 'world' (configurable). Posts sit on the ground at z = scale/2.
    """
 
    def __init__(self):
        super().__init__('finish_line_markers')
 
        # ---- Parameters (tweak at runtime with `ros2 param set`) ----
        self.declare_parameter('frame_id',  'world')
        self.declare_parameter('finish_x',  60.0)
        self.declare_parameter('offset_y',  1.0)   # ±offset_y
        self.declare_parameter('scale',     0.3)   # x,y diameter and z height
        self.declare_parameter('r',         1.0)
        self.declare_parameter('g',         1.0)
        self.declare_parameter('b',         1.0)
        self.declare_parameter('a',         1.0)
        self.declare_parameter('period',    0.5)   # seconds
 
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.finish_x = float(self.get_parameter('finish_x').value)
        self.offset_y = float(self.get_parameter('offset_y').value)
        self.scale    = float(self.get_parameter('scale').value)
        self.color    = (
            float(self.get_parameter('r').value),
            float(self.get_parameter('g').value),
            float(self.get_parameter('b').value),
            float(self.get_parameter('a').value),
        )
        period = float(self.get_parameter('period').value)
 
        # ---- RViz-friendly QoS (latched) ----
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
 
        # ---- Publishers on multiple topics ----
        self.pub_finish = self.create_publisher(MarkerArray, '/finish_line', qos)
        self.pub_va     = self.create_publisher(MarkerArray, '/visualization_marker_array', qos)
        self.pub_vm     = self.create_publisher(Marker,      '/visualization_marker', qos)
 
        self.timer = self.create_timer(period, self.publish_markers)
        self._announced = False
 
    # Utility: build a single cylinder marker
    def _make_post(self, marker_id: int, y_pos: float) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'finish_line'
        m.id = marker_id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
 
        # Position: x at finish line; ± offset_y; sit on ground
        m.pose.position.x = self.finish_x
        m.pose.position.y = float(y_pos)
        m.pose.position.z = self.scale / 2.0
        m.pose.orientation.w = 1.0
 
        # Size: BIG
        m.scale.x = self.scale   # diameter X
        m.scale.y = self.scale   # diameter Y
        m.scale.z = self.scale * 5.0   # height Z
 
        # Color + visibility
        m.color.r, m.color.g, m.color.b, m.color.a = self.color
        m.frame_locked = True           # follow TF at render time
        m.lifetime = Duration(sec=0, nanosec=0)  # persist
 
        return m
 
    def publish_markers(self):
        left  = self._make_post(1, -self.offset_y)
        right = self._make_post(2,  self.offset_y)
 
        # MarkerArray on two topics
        ma = MarkerArray()
        ma.markers = [left, right]
        self.pub_finish.publish(ma)
        self.pub_va.publish(ma)
 
        # Also publish individually on visualization_marker
        self.pub_vm.publish(left)
        self.pub_vm.publish(right)
 
        if not self._announced:
            self.get_logger().info(
                f"Finish line posts latched on 3 topics "
                f"(frame='{self.frame_id}', x={self.finish_x:.1f}, y=±{self.offset_y:.1f}, scale={self.scale:.2f}). "
                f'Use RViz "MarkerArray" on /finish_line or /visualization_marker_array, '
                f'or "Marker" on /visualization_marker.'
            )
            self._announced = True
 
def main():
    rclpy.init()
    node = FinishLineMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()