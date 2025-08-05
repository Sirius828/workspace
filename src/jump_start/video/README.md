# Video Storage Directory

This directory stores recorded videos from the robot's camera during target detection and victory moments.

## File Format
- Detection videos: `detection_YYYYMMDD_HHMMSS.mp4`
- Videos contain the last 10 seconds of camera feed when YOLO detection occurs
- Videos are automatically saved when victory condition is met

## Storage Notes
- Videos are saved in MP4 format with up to 30fps
- Each video contains approximately 10 seconds of footage
- Automatic cleanup may be needed to manage disk space
