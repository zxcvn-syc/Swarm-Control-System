"""Tracker package: BoT-SORT, DeepSORT-lite, DeepSORT-cascade, CMC, smoother, KF."""

from cvtrack.tracker.botsort import BoTSortTracker
from cvtrack.tracker.cmc import (
    CameraMotionCompensator,
    EccCompensator,
    SparseOFCompensator,
    affine_is_pure_camera_pan,
    make_cmc,
)
from cvtrack.tracker.deepsort import (
    DEEPSORT_APPEARANCE_GATE,
    DEEPSORT_MAHALANOBIS_GATE,
    DeepSortCascade,
    DeepSortLite,
)
from cvtrack.tracker.kalman import (
    BOTSORT_HIGH_CONF,
    BOTSORT_IOU_THRESH,
    BOTSORT_LOST_RELINK_FRAMES,
    BOTSORT_NEW_TRACK_CONF,
    CHI2_THRESHOLD,
    KalmanBoT,
    KalmanCV2D,
    predict_n_steps,
    predict_n_steps_with_covariance,
)
from cvtrack.tracker.metrics import (
    CHI2_INV_95_2DOF,
    CHI2_INV_95_4DOF,
    class_aware_iou_distance,
    gate_mahalanobis,
    idf1,
    iou,
    iou_matrix,
    mahalanobis_2d,
)
from cvtrack.tracker.smoother import rts_smooth_2d

__all__ = [
    "BOTSORT_HIGH_CONF",
    "BOTSORT_IOU_THRESH",
    "BOTSORT_LOST_RELINK_FRAMES",
    "BOTSORT_NEW_TRACK_CONF",
    "BoTSortTracker",
    "CHI2_INV_95_2DOF",
    "CHI2_INV_95_4DOF",
    "CHI2_THRESHOLD",
    "CameraMotionCompensator",
    "DEEPSORT_APPEARANCE_GATE",
    "DEEPSORT_MAHALANOBIS_GATE",
    "DeepSortCascade",
    "DeepSortLite",
    "EccCompensator",
    "KalmanBoT",
    "KalmanCV2D",
    "SparseOFCompensator",
    "affine_is_pure_camera_pan",
    "class_aware_iou_distance",
    "gate_mahalanobis",
    "idf1",
    "iou",
    "iou_matrix",
    "mahalanobis_2d",
    "make_cmc",
    "predict_n_steps",
    "predict_n_steps_with_covariance",
    "rts_smooth_2d",
]