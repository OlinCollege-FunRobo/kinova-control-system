from backend.kinova import BaseApp
import numpy as np
from src.planning.kinematics import calc_inverse_kinematics
from src.planning.utils import EndEffector
import pyrealsense2 as rs
import cv2
import math

def camera_to_robot_base(point_camera):
    # Calibration vectors from solvePnP (Base -> Camera)
    rvec = np.array([[-2.336613872251611], [-2.038652935760678], [-0.0073696870535258]], dtype=np.float64)
    tvec = np.array([[-0.012022628847092487], [-0.38039863812963065], [1.3811058819390545]], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    P_camera = np.array(point_camera, dtype=np.float64).reshape(3, 1)
    R_inv = R.T
    P_base = R_inv @ (P_camera - tvec)
    return P_base.flatten()

def get_tag_base_position():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"Failed to start high res stream, falling back to 640x480: {e}")
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        
    align = rs.align(rs.stream.color)
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    print("Looking for ArUco tag ID 1...")
    tag_base_pos = None
    
    try:
        for _ in range(150):
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame: continue
            
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)
            
            if ids is not None:
                for i in range(len(ids)):
                    if ids[i][0] == 1:
                        c = corners[i][0]
                        center_x = int(np.mean(c[:, 0]))
                        center_y = int(np.mean(c[:, 1]))
                        depth = depth_frame.get_distance(center_x, center_y)
                        if depth > 0:
                            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
                            point_camera = rs.rs2_deproject_pixel_to_point(color_intrin, [center_x, center_y], depth)
                            tag_base_pos = camera_to_robot_base(point_camera)
                            break
                if tag_base_pos is not None:
                    break
    finally:
        pipeline.stop()
        
    return tag_base_pos


class Main(BaseApp):
        
    def start(self):
        self.home = False     
        self.HOME_POSITION = np.deg2rad(np.array([360, 340, 180, 214, 0 , 310, 90]))
        
        # Get ArUco tag position
        tag_pos = get_tag_base_position()
        
        if tag_pos is None:
            print("ERROR: Could not find ArUco tag ID 1. Defaulting to home position.")
            self.target_joint_angles = self.HOME_POSITION
        else:
            print(f"Found ArUco tag at robot base coordinates: {tag_pos}")
            # Apply offset to Z position
            tag_pos[2] += 0.23
            print(f"Target position: {tag_pos}")
            
            ee = EndEffector()
            ee.x, ee.y, ee.z = tag_pos[0], tag_pos[1], tag_pos[2]
            ee.rotx, ee.roty, ee.rotz = 0.0, math.pi, 0.0
            
            print("Calculating IK to target position...")
            ik_solution = calc_inverse_kinematics(ee, self.HOME_POSITION.tolist())
            print(f"IK Solution found: {ik_solution}")
            self.target_joint_angles = np.array(ik_solution)
        
    def loop(self):        
        if(self.home):
            self.kinova_robot.set_joint_angles(self.target_joint_angles, gripper_percentage=100)
            self.home = False
        else:
            self.kinova_robot.set_joint_angles(self.HOME_POSITION, gripper_percentage=0)
            self.home = True            

if __name__ == "__main__":
    simulate = False
    
    if(simulate is None):
        raise ValueError("Pick simulate or real world robot")
    
    if simulate:
        final_project = Main(simulate=True, urdf_path="visualizer/7dof/urdf/7dof.urdf")
        pass
    else:
        final_project = Main(is_suction=False)
    
    try:
        while True:
            pass
    except KeyboardInterrupt:
        final_project.shutdown()
