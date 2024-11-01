Simple scripts to collect realsense trajectories 

## Installation:
```bash
conda create -n realsense python=3.10
python3 -m pip install -U real-robot
pip install termcolor
```

```bash
pip install termcolor
pip install tqdm
pip install imageio[ffmpeg]
```

## Usage
On Linux: 
```bash
python scripts.py
```

Press " " to start recording and press " " again to stop it. Press "q" to quit.

On Windows:
```bash
python scripts_windows.py
```

Press "a" to start recording and press "a" again to stop it. Press "q" to quit.

