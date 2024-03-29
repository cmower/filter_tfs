import rospy
import tf2_ros


from geometry_msgs.msg import Transform


def init_buffer():
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    return tf_buffer


class TransformListener:

    def __init__(
        self,
        tf_buffer: tf2_ros.Buffer,
        parent_frame: str,
        child_frame: str,
    ) -> None:
        self.tf_buffer = tf_buffer
        self.parent_frame = parent_frame
        self.child_frame = child_frame

    def get(self) -> Transform:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, rospy.Time()
            )
            transform = tf.transform
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            transform = None
        return transform
