# Raspberry Pi Camera
This is where we will place the information of the camera, how to set it up, code library, etc

## Camera Information
Model: RASPBERRY PI AI CAMERA IMX500 


## Setup
The general setup guide that was done is in the [Raspberry Pi AI Camera Documentation](https://www.raspberrypi.com/documentation/accessories/ai-camera.html)


## Code
### Example Applications
After setting up, these are some tests we could do to check for the camera
**Object Detection**
```bash
rpicam-hello -t 0s --post-process-file /usr/share/rpi-camera-assets/imx500_mobilenet_ssd.json --viewfinder-width 1920 --viewfinder-height 1080 --framerate 30
```
**Pose Estimation**
```bash
rpicam-hello -t 0s --post-process-file /usr/share/rpi-camera-assets/imx500_posenet.json --viewfinder-width 1920 --viewfinder-height 1080 --framerate 30
```

Since we are using Python, Picamera2 library will be used. Setup is needed and can be found in the [AI Camera Documentation](https://www.raspberrypi.com/documentation/accessories/ai-camera.html)
An example code that is used for this is from the [github repository](https://github.com/raspberrypi/picamera2/blob/main/examples/imx500/imx500_object_detection_demo.py)
