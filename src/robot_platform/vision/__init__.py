"""Camera geometry for Minerva: the table's ArUco board, the pinhole camera with lens distortion, the arms' kinematic
chains, and the wrist cameras' calibration from still poses (vision.wrist_calibration).

Numpy and scipy only, so the gate tests every solver path without OpenCV; tag detection, which needs OpenCV, runs on
the robot (helm.calibration_still).
"""
