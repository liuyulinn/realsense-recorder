import time
from termcolor import cprint
from realse_reader import Observer
from recorder import Recorder
import argparse 
from tqdm import tqdm

parser = argparse.ArgumentParser()
parser.add_argument("--fps", type=int, default=30)
args = parser.parse_args()

try:
    from pynput.keyboard import Key, KeyCode, Listener
except ImportError as e:
    print(e)

pressed = ""


def _on_key_press(key):
    global pressed
    if key == KeyCode.from_char("a"):
        pressed = "a"
    elif key == KeyCode.from_char("q"):
        pressed = "q"
    else:
        pressed = ""


def main():
    observer = Observer(align_to="color")
    key_listener = Listener(on_press=_on_key_press)
    key_listener.start()

    recorder = Recorder()

    fps = args.fps
    camera_intrinsics = observer.K
    recorder.regist_camera_info(camera_intrinsics, fps)

    for i in tqdm(range(1000000)):
        cur_time = time.time()
        depth, rgb = observer.get_rgb_depth()

        global pressed
        if pressed == "a":
            recorder.change_status()
            pressed = ""
        elif pressed == "q":
            cprint("Q key pressed, exiting...", "yellow")
            break
        elif pressed != "": 
            cprint(f"key {pressed} pressed", "yellow")
        
        # idx += 1
        recorder.realsense_step(rgb, depth)

        time.sleep(
            max(1 / fps - (time.time() - cur_time), 0)
        )


if __name__ == "__main__":
    main()
