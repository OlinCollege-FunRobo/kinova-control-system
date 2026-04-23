"""
RealSense camera driver interface.
Initializes the depth and RGB camera streams, configures settings,
and retrieves aligned frames for perception processing.
"""
import pyrealsense2 as rs

class RealSenseDriver:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

    def get_frame(self):
        frame = self.pipeline.wait_for_frames()
        depth_frame = frame.get_depth_frame()
        color_frame = frame.get_color_frame()
        return depth_frame, color_frame

    def stop(self):
        self.pipeline.stop()

if __name__ == "__main__":
    driver = RealSenseDriver()
    depth_frame, color_frame = driver.get_frame()
    driver.stop()