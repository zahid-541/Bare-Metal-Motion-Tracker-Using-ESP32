# ESP32-S3 Moving Object Detector

Detect moving objects in a video using an **ESP32-S3** — no AI, no machine
learning, no cloud. Just plain pixel math running on the microcontroller.

A Python script on your PC reads a video file and sends each frame to the
ESP32-S3 over USB. The ESP32-S3 compares each new frame to the previous ones,
figures out which pixels changed, groups those pixels into objects, and sends
back the bounding boxes. The PC then draws those boxes on the video so you can
see what the ESP32 detected.

That's the whole idea: **the PC just shows the video, the ESP32 does the
actual detection work.**



## How it works (in plain words)

1. The PC grabs a frame from the video and shrinks it to 160×120 grayscale.
2. It sends the frame to the ESP32-S3 over a fast serial connection.
3. The ESP32-S3 keeps a "memory" of what the scene normally looks like (the
   background). It compares the new frame to this background.
4. Any pixel that looks very different from the background is marked as
   "something moved here."
5. The ESP32 groups nearby changed pixels together into objects and draws a
   box around each one.
6. It keeps track of these objects across frames so the boxes don't flicker
   or jump around.
7. It sends the boxes back to the PC, which draws them on the video.

No neural network. No trained model. No internet. If a pixel changes, it's
movement. If it doesn't, it's background. That's it.



## What you need

**Hardware**
- An ESP32-S3 board with PSRAM
- A USB cable
- A PC (Windows, Mac, or Linux)

**Software**
- Arduino IDE (to flash the ESP32)
- Python 3.8 or newer
- A video file to test with (any `.mp4` works)



## Setup

### 1. Flash the ESP32-S3

Open `firmware/esp32_cv_coprocessor.ino` in Arduino IDE, select your
ESP32-S3 board (make sure PSRAM is enabled), and upload.

### 2. Install Python packages

```bash
pip install pyserial opencv-python numpy
```

### 3. Edit the config

Open `host/stream_host.py` and change these two lines to match your setup:

```python
PORT       = 'COM13'        # Windows: COMx, Linux/Mac: /dev/ttyUSB0
VIDEO_PATH = 'video2.mp4'   # path to your test video
```

### 4. Run it

```bash
python host/stream_host.py
```

A window opens showing the video with green boxes around anything that
moves. Press `q` to quit.

---

## Settings you can change

All the detection settings are at the top of the ESP32 code. You can tweak
them without understanding the rest:

| Setting         | What it does                                          |
|-----------------|-------------------------------------------------------|
| `PIXEL_THRESH`  | How much a pixel must change to count as movement     |
| `MIN_BLOB_AREA` | Ignore objects smaller than this (filters out noise)  |
| `MAX_BOXES`     | How many objects can be tracked at once               |
| `MATCH_DIST`    | How far an object can move between frames             |
| `MAX_MISSED`    | How long to keep a box if the object briefly hides    |
| `BOX_PADDING`   | Extra space added around each box                     |

If you're getting too many false detections, raise `PIXEL_THRESH`. If small
objects are being ignored, lower `MIN_BLOB_AREA`.

---

## Why do it this way?

Most "object detection" projects use AI models that need training data, lots
of memory, and usually a GPU. This project shows you can detect moving
objects with nothing but a $10 microcontroller and basic math that's been
around for 40 years.

It's fast, it's cheap, it runs offline, and you can understand every line
of code. It won't tell you *what* the object is (car vs. person vs. dog),
but it will reliably tell you *that* something is moving and *where*.

---

## Limits (honest ones)

- If an object stops moving, it slowly becomes part of the background and
  the box disappears. That's how this kind of detector works.
- It doesn't recognize what objects are — only that something moved.
- If two objects cross paths, their box IDs might swap.
- Works best with a fixed camera. A shaking camera will look like "everything
  moved" to the ESP32.


## License

MIT — use it however you want.
