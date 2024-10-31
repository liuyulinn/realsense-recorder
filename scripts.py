import argparse
import select
import sys
import termios
import time
import tty

import cv2
# from loguru import logger

# from offline_tele.utils.path import PACKAGE_ASSET_DIR
from real_robot.sensors.camera import Camera, CameraConfig
from real_robot.utils.camera import depth2xyz
from real_robot.utils.visualization import Visualizer
from termcolor import cprint

from recorder import Recorder

parser = argparse.ArgumentParser()
parser.add_argument("--record", "-r", type=int, default=1, help="Record the data")
args = parser.parse_args()

try:
    from pynput.keyboard import Key, KeyCode, Listener
except ImportError as e:
    print(e)

record = 0


def _on_key_press(key):
    global record
    if key == KeyCode.from_char(" "):
        record = 1
    else:
        record = 0


def keypoint_detection():
    # Obtain the image from the Realsense camera
    camera = Camera(
        CameraConfig(
            "cam1",
            # device_sn="146322070293",
            config={
                "Color": (848, 480, 60),
                "Depth": (848, 480, 60),
            },
            preset="Default",
        ),
    )
    realsense_visualizer = Visualizer(
        run_as_process=True, stream_camera=True, stream_robot=False
    )

    if args.record:
        recorder = Recorder(camera=camera)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)

    idx = 0
    try:
        cprint("Press space to start/stop recording.", "yellow")
        while True:
            cur_time = time.time()
            image = camera.get_images(True)
            rgb = image["rgb"]
            depth = image["depth"]

            if args.record:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == " ":
                        recorder.change_status()
                    elif key == "q":
                        cprint("Q key pressed, exiting...", "yellow")
                        break
                    else:
                        cprint(f"Detected key: {key}", "cyan")

                idx += 1
                recorder.realsense_step(rgb, depth)

            time.sleep(
                max(1 / camera.config["Color"][-1] - (time.time() - cur_time), 0)
            )

    except Exception as e:
        cprint(f"Exception occurred: {e}", "red")

    finally:
        # Ensure the terminal settings are restored even if an error occurs
        if args.record:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        cprint("Restored terminal settings.", "green")


def keypoint_detection_two_cam():
    # Initialize two cameras
    camera1 = Camera(
        CameraConfig(
            uid="cam1",
            device_sn="146322076186",
            config={
                "Color": (848, 480, 60),
                "Depth": (848, 480, 60),
            },
            preset="Default",
        ),
    )

    camera2 = Camera(
        CameraConfig(
            uid="cam2",
            device_sn="146322070293",
            config={
                "Color": (848, 480, 60),
                "Depth": (848, 480, 60),
            },
            preset="Default",
        ),
    )

    realsense_visualizer = Visualizer(
        run_as_process=True, stream_camera=True, stream_robot=False
    )

    if args.record:
        recorder1 = Recorder(camera=camera1)
        recorder2 = Recorder(camera=camera2)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)

    idx = 0
    try:
        cprint("Press space to start/stop recording.", "yellow")
        while True:
            cur_time = time.time()
            image1 = camera1.get_images(True)
            image2 = camera2.get_images(True)
            rgb1, depth1 = image1["rgb"], image1["depth"]
            rgb2, depth2 = image2["rgb"], image2["depth"]

            if args.record:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == " ":
                        recorder1.change_status()
                        recorder2.change_status()
                    elif key == "q":
                        cprint("Q key pressed, exiting...", "yellow")
                        break
                    else:
                        cprint(f"Detected key: {key}", "cyan")

                idx += 1
                recorder1.realsense_step(rgb1, depth1)
                recorder2.realsense_step(rgb2, depth2)

            time.sleep(
                max(1 / camera1.config["Color"][-1] - (time.time() - cur_time), 0)
            )

    except Exception as e:
        cprint(f"Exception occurred: {e}", "red")

    finally:
        # Ensure the terminal settings are restored even if an error occurs
        if args.record:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        cprint("Restored terminal settings.", "green")


if __name__ == "__main__":
    keypoint_detection()
