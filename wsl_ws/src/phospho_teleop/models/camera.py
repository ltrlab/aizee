from typing import List

from pydantic import BaseModel, Field

from phospho_teleop.types import CameraTypes


class SingleCameraStatus(BaseModel):
    camera_id: int
    is_active: bool
    camera_type: CameraTypes = Field(
        description="Type of camera."
        + "\n`classic`: Standard camera detected by OpenCV."
        + "\n`stereo`: Stereoscopic camera. It has two lenses: left eye and right eye to give a 3D effect. The left half of the image is the left eye, and the right half is the right eye."
        + "\n`realsense`: Intel RealSense camera. It use infrared sensors to provide depth information. It requires a special driver."
        + "\n`dummy`: Dummy camera. Used for testing."
        + "\n`dummy_stereo`: Dummy stereoscopic camera. Used for testing."
        + "\n`unknown`: Unknown camera type."
    )
    width: int
    height: int
    fps: int


class AllCamerasStatus(BaseModel):
    """
    Description of the status of all cameras. Use this to know which camera to stream.
    """

    cameras_status: List[SingleCameraStatus] = Field(default_factory=list)
    is_stereo_camera_available: bool = Field(
        default=False, description="Whether a stereoscopic camera is available."
    )
    realsense_available: bool = Field(
        default=False, description="Whether a RealSense camera is available."
    )
    video_cameras_ids: List[int] = Field(
        default_factory=list,
        description="List of camera ids that are video cameras.",
    )
