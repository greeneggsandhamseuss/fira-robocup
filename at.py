#============================================================================

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from pupil_apriltags import Detector

#============================================================================

april_cm = 100.0   #convert tag transpose to cm 
# focal = [921,919]
# center = [460,351]
focal = [550,550]
center = [480,360]

#============================================================================

at_detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.8,
    debug=0,
)

#============================================================================

def get_tags(img):
    tags = at_detector.detect(
        cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
        estimate_tag_pose=True,
        camera_params=(focal[0], focal[1], center[0],center[1]),
        tag_size=0.11
    )

    res = []
    if len(tags) > 0:
        for tag in tags:
            r =  Rotation.from_matrix(tag.pose_R)
            angles = r.as_euler("zyx",degrees=True)
            T = tag.pose_t.reshape((3)).tolist()
            for i in range(3):
                T[i] = int(T[i] * april_cm)
            res.append([tag.tag_id, T[0], T[1], T[2], int(angles[1]), int(tag.center[0]), int(tag.center[1])])

    return res

#============================================================================
