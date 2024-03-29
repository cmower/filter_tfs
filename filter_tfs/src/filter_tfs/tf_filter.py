import abc
from typing import Tuple

from geometry_msgs.msg import Transform

import numpy as np

from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.transform import Slerp


def transform2numpy(transform: Transform) -> Tuple[np.ndarray]:
    t = transform.translation
    r = transform.rotation
    tnp = np.array([t.x, t.y, t.z])
    rnp = np.array([r.x, r.y, r.z, r.w])
    return tnp, rnp


def numpy2transform(t: np.ndarray, r: np.ndarray) -> Transform:
    transform = Transform()
    transform.translation.x = t[0]
    transform.translation.y = t[1]
    transform.translation.z = t[2]


class Filter(abc.ABC):

    def __init__(self, alpha: float, maxn: int) -> None:
        self.alpha = alpha
        self.maxn = maxn
        self.filt = None
        self.n = 0

    def is_ready(self) -> bool:
        return self.filt is not None

    @abc.abstractmethod
    def apply(self, raw: np.ndarray) -> None:
        """Apply filter."""

    def __call__(self, raw: np.ndarray) -> np.ndarray:
        if self.filt is None or self.n < self.maxn:
            self.filt = raw
        self.n += 1
        self.apply(raw)
        return self.filt.copy()


class TranslationFilter(Filter):

    def apply(self, raw: np.ndarray) -> None:
        self.filt = (1.0 - self.alpha) * self.filt + self.alpha * raw


class RotationFilter(Filter):

    def apply(self, raw: np.ndarray):
        ts = [0.0, 1.0]
        rs = [Rot.from_quat(self.filt), Rot.from_quat(raw)]
        slerp = Slerp(ts, rs)
        self.filt = slerp(self.alpha).as_quat()


class TransformFilter:

    def __init__(self, fraction_translation, fraction_rotation, max_observations):
        self.translation_filter = TranslationFilter(
            fraction_translation,
            max_observations,
        )
        self.rotation_filter = RotationFilter(
            fraction_rotation,
            max_observations,
        )

    def is_ready(self):
        return self.translation_filter.is_ready() and self.rotation_filter.is_ready()

    def __call__(self, transform: Transform):
        if transform is None:
            return
        tr, rr = transform2numpy(transform)
        tf = self.translation_filter(tr)
        rf = self.rotation_filter(rr)
        return numpy2transform(tf, rf)
