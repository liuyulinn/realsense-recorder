import os
import time

import cv2
import imageio.v2 as iio
import numpy as np

# from offline_tele.modules.nuwa_utils.mask_extractor import extract_mask_from_image
# from offline_tele.utils.path import PACKAGE_ASSET_DIR
# from offline_tele.utils.video_image import save_mp4
from PIL import Image
from real_robot.sensors.camera import Camera, CameraConfig
from termcolor import cprint


def save_mp4(save_path: str, frames: list[np.ndarray], fps: int = 60):
    videoWriter = iio.get_writer(
        save_path,
        format="ffmpeg",  # type: ignore
        mode="I",
        fps=fps,
        codec="libx264",
        pixelformat="yuv420p",
    )

    # fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    # out = cv2.VideoWriter(save_path, fourcc, fps, (width, height))

    for frame in frames:
        videoWriter.append_data(frame)
        if cv2.waitKey(10) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


class Recorder:
    def __init__(
        self,
        camera: Camera,
        save_dir: str = "video",
        # extract_mask: bool = True,
        # record_hand: bool = False,
    ) -> None:
        self.fps = camera.config["Color"][-1]  # type: ignore
        self.camera_intrinsics = camera.intrinsic_matrix

        # self.extract_mask = extract_mask

        assert (
            self.fps is not None and self.camera_intrinsics is not None
        ), "Camera is not initialized properly."
        print(f"Recording at {self.fps} fps")

        self.cam_id = camera.camera_cfg.uid

        self.sensor_rgb = []
        self.sensor_depth = []
        self.time_stamp = []

        # self.record_hand = record_hand
        self.time_stamp_hand = []
        self.hand_pos = []

        self.recording = False
        self.record_start_time = None

        os.makedirs(save_dir, exist_ok=True)
        self.save_dir = save_dir

    def change_status(self):
        if self.recording:
            self.stop()
        else:
            self.start()

    def start(self):
        cprint("Recording started.", "cyan")
        self.recording = True
        path = time.strftime("%Y%m%d-%H%M%S")
        self.save_path = f"{self.save_dir}/{path}_{self.cam_id}"
        os.makedirs(self.save_path, exist_ok=True)
        print(f"Recording states in {self.save_path}")

        self.record_start_time = time.time()
        self.time_stamp = []

        self.sensor_rgb = []
        self.sensor_depth = []

        # self.videoWriter = iio.get_writer(
        #     f"{self.save_path}/sensor_rgb.mp4",
        #     fps=self.fps,
        #     format="ffmpeg",  # type: ignore
        #     mode="I",
        #     codec="libx264",
        #     pixelformat="yuv420p",
        # )

    def stop(self):
        self.recording = False
        assert self.record_start_time is not None, "Recording is not started."
        record_time = time.time() - self.record_start_time

        # self.records["record_time"] = record_time
        # self.records["states"] = self.states
        cprint(f"Saving states to {self.save_path}...", "yellow")
        np.save(f"{self.save_path}/time.npy", np.stack(self.time_stamp))

        # save sensor data
        # if self._record_sensor:
        # assert self.sensor_rgb is not None and self.sensor_depth is not None
        print(len(self.sensor_rgb))
        save_mp4(
            f"{self.save_path}/sensor_rgb.mp4",
            self.sensor_rgb,
            fps=self.fps,
        )
        # cv2.destroyAllWindows()

        sensor_depth = np.stack(
            [
                (sensor_data * 1000).astype(np.uint16)
                for sensor_data in self.sensor_depth
            ],
            axis=0,
        )
        # sensor_depth = np.stack(self.sensor_depth, axis=0)

        np.save(f"{self.save_path}/sensor_depth.npy", sensor_depth)
        cv2.imwrite(f"{self.save_path}/frame_0.png", self.sensor_rgb[0])

        # self.n_count = len(os.listdir(self.save_dir))
        with open(os.path.join(self.save_path, "cam_K.txt"), "w") as f:
            for row in self.camera_intrinsics:
                f.write(" ".join(map(str, row)) + "\n")

        # if self.extract_mask:
        #     mask = extract_mask_from_image(Image.fromarray(self.sensor_rgb[0]))
        #     # np.save(f"{self.save_path}/mask.npy", mask)
        #     Image.fromarray(mask.astype("uint8") * 255).save(
        #         f"{self.save_path}/mask.png"
        #     )

        # if self.record_hand:
        #     np.save(f"{self.save_path}/time_hand.npy", np.stack(self.time_stamp_hand))
        #     np.save(f"{self.save_path}/hand_pos.npy", np.stack(self.hand_pos))

        # )
        cprint(f"States are saved in {self.save_path}.", "yellow")

        return record_time

    def realsense_step(self, rgb: np.ndarray, depth_image: np.ndarray) -> None:
        if not self.recording:
            return

        # self.state["timestamp"] = self.time
        assert self.record_start_time is not None
        self.time_stamp.append(time.time())  # - self.record_start_time)
        # self.time = time.time() - self.record_start_time
        self.sensor_rgb.append(rgb)
        # if self.sensor_rgb is None:
        # self.sensor_rgb = rgb
        # self.videoWriter.append_data(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

        self.sensor_depth.append(depth_image)
        # self.states.append(self.state)

        # self.state = {}

    def hand_step(self, hand_pos: list[np.ndarray]) -> None:
        # print("inside recorder", self.recording)
        if not self.recording:
            return

        # self.state["timestamp"] = self.time
        assert self.record_start_time is not None
        self.time_stamp_hand.append(time.time())

        # print("inside recorder")
        self.hand_pos.append(np.stack(hand_pos))
