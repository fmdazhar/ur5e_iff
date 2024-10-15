import numpy as np
import cv2
import threading
import queue
from collections import OrderedDict
from datetime import datetime
from typing import Dict

from video_capture import VideoCapture
from rs_capture import RSCapture

class ImageDisplayer(threading.Thread):
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.daemon = True  # Make this a daemon thread

    def run(self):
        while True:
            img_array = self.queue.get()  # Retrieve an image from the queue
            if img_array is None:  # None is our signal to exit
                break

            frame = np.concatenate(
                [v for k, v in img_array.items() if "full" not in k], axis=0
            )

            cv2.imshow("RealSense Cameras", frame)
            cv2.waitKey(1)

class FrankaCameraHandler:
    def __init__(self, config, fake_env=False, save_video=False):
        self.config = config
        self.save_video = save_video
        self.recording_frames = []

        if fake_env:
            return

        self.cap = None
        self.init_cameras(config.REALSENSE_CAMERAS)
        self.img_queue = queue.Queue()
        self.displayer = ImageDisplayer(self.img_queue)
        self.displayer.start()
        print("Initialized Camera Handler")

    def init_cameras(self, name_serial_dict=None):
        """Initialize the RealSense cameras."""
        if self.cap is not None:  # Close cameras if they are already open
            self.close_cameras()

        self.cap = OrderedDict()
        for cam_name, cam_serial in name_serial_dict.items():
            cap = VideoCapture(
                RSCapture(name=cam_name, serial_number=cam_serial, depth=False)
            )
            self.cap[cam_name] = cap

    def close_cameras(self):
        """Close all RealSense cameras."""
        try:
            for cap in self.cap.values():
                cap.close()
        except Exception as e:
            print(f"Failed to close cameras: {e}")

    def get_im(self) -> Dict[str, np.ndarray]:
        """Capture images from the RealSense cameras."""
        images = {}
        display_images = {}
        for key, cap in self.cap.items():
            try:
                rgb = cap.read()
                cropped_rgb = self.crop_image(key, rgb)
                resized = cv2.resize(
                    cropped_rgb, (128, 128)
                )
                images[key] = resized[..., ::-1]  # Convert BGR to RGB
                display_images[key] = resized
                display_images[key + "_full"] = cropped_rgb
            except queue.Empty:
                input(
                    f"{key} camera frozen. Check connection, then press Enter to relaunch..."
                )
                cap.close()
                self.init_cameras(self.config.REALSENSE_CAMERAS)
                return self.get_im()

        # For video recording
        self.recording_frames.append(
            np.concatenate([display_images[f"{k}_full"] for k in self.cap], axis=0)
        )
        self.img_queue.put(display_images)
        return images

    def crop_image(self, name, image) -> np.ndarray:
        """Crop RealSense images to a square."""
        if name in ["wrist_1", "wrist_2"]:
            return image[:, 80:560, :]
        else:
            raise ValueError(f"Camera {name} not recognized in cropping")

    def save_video_recording(self):
        """Save the recorded frames into a video file."""
        try:
            if len(self.recording_frames):
                video_writer = cv2.VideoWriter(
                    f'./videos/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.mp4',
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    10,
                    self.recording_frames[0].shape[:2][::-1],
                )
                for frame in self.recording_frames:
                    video_writer.write(frame)
                video_writer.release()
            self.recording_frames.clear()
        except Exception as e:
            print(f"Failed to save video: {e}")

    def reset_video_recording(self):
        """Reset the video recording frames."""
        self.recording_frames.clear()

# Example usage:
if __name__ == "__main__":
    class DefaultEnvConfig:
        """Default configuration for FrankaEnv."""
        REALSENSE_CAMERAS: Dict = {
            "wrist_1": "130322274175",
            "wrist_2": "127122270572",
        }

    config = DefaultEnvConfig()
    camera_handler = FrankaCameraHandler(config, fake_env=False, save_video=True)

    try:
        while True:
            images = camera_handler.get_im()
            # Perform any additional processing with the images
    except KeyboardInterrupt:
        camera_handler.close_cameras()
        camera_handler.save_video_recording()
        print("Camera handler stopped.")
