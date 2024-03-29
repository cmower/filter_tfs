import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from filter_tfs.tf_listener import init_buffer, TransformListener
from filter_tfs.tf_filter import TransformFilter


class TransformHandler:

    def __init__(
        self,
        tf_broadcaster,
        parent_frame: str,
        child_frame: str,
        fraction_translation: float,
        fraction_rotation: float,
        max_observations: int,
    ):
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.tf_broadcaster = tf_broadcaster
        self.tf_listener = TransformListener(self.tf_buffer, parent_frame, child_frame)
        self.tf_filter = TransformFilter(
            fraction_translation, fraction_rotation, max_observations
        )

    def update(self):
        tf = TransformStamped(transform=self.tf_filter(self.tf_listener.get()))
        if tf.transform is None:
            return
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.child_frame + "_filtered"
        tf.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf)


class Node:

    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)
        self.hz = rospy.get_param("~hz", 100)
        self.fraction_translation = self._get_fraction_param("~fraction_translation")
        self.fraction_rotation = self._get_fraction_param("~fraction_rotation")
        self.max_observations = rospy.get_param("~max_observations", 10)
        assert (
            self.max_observations > 0
        ), "~max_observations must be an int greater than 0"
        self.tf_buffer = init_buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_handlers = []

    def _get_fraction_param(self, name):
        f = rospy.get_param(name)
        assert 0.0 <= f < 1.0, f"{name} must be in range [0, 1)"
        return f

    def add(
        self,
        parent_frame: str,
        child_frame: str,
    ) -> None:
        self.tf_handlers.append(
            TransformHandler(
                self.tf_broadcaster,
                parent_frame,
                child_frame,
                self.fraction_translation,
                self.fraction_rotation,
                self.max_observations,
            )
        )

    def start(self):
        dt = rospy.Duration(1.0 / float(self.hz))
        rospy.Timer(dt, self.timer_callback)

    def timer_callback(self, event):
        for tf_handler in self.tf_handlers:
            tf_handler.update()

    def spin(self):
        rospy.spin()


def main(node_cls):
    node = node_cls()
    node.start()
    node.spin()
