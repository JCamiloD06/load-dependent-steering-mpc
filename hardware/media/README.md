# Testbench Demonstration Media & Online Playable Videos

This directory hosts demonstration videos, animated GIF previews, and high-resolution photographic documentation of the physical direct-drive steering testbench operating in real time.

---

## 1. Media Assets

| File Name | Format | Resolution | Description |
| :--- | :---: | :---: | :--- |
| **`bench_hil_demonstration.gif`** | Animated GIF / WebM | $720\times 480$ (30 fps) | Real-time direct-drive steering testbench tracking high-speed chicane maneuvers in Assetto Corsa with active Force Feedback load. |
| **`bench_phase1_identification.gif`** | Animated GIF | $640\times 360$ (30 fps) | Physical actuator executing automated multi-frequency chirp and step-torque identification routines. |
| **`hil_cockpit_tracking.mp4`** | H.264 MP4 | 1080p (60 fps) | Dual synchronized capture: physical steering wheel rotation and in-game Assetto Corsa cockpit telemetry at Autodromo Nazionale di Monza. |

---

## 2. Online Embedding in GitHub README

To display videos that run directly inside the GitHub `README.md` without requiring external downloads, use animated GIFs or HTML5 `<video>` tags:

### Embedded GIF Syntax:
```markdown
<p align="center">
  <img src="hardware/media/bench_hil_demonstration.gif" width="720" alt="Physical Direct-Drive Steering Testbench in HIL Operation">
  <br>
  <em>Figure: Real-time closed-loop steering tracking under resistive tire self-aligning torque (Assetto Corsa + ODrive v0.5.6).</em>
</p>
```

---

## 3. Recommended Video Conversion Pipeline

To convert a recorded `.mp4` video (e.g. from a smartphone or OBS Studio) into a lightweight, high-quality GIF for GitHub:

```bash
# Using ffmpeg:
ffmpeg -i input_video.mp4 -vf "fps=25,scale=720:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 hardware/media/bench_hil_demonstration.gif
```

Place your converted `.gif` or `.mp4` files into this directory.
