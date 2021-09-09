## > MOWBOT
### _the worlds most over the top complicated "lawn mower"_
### _or_
### _a F1Tenth autonomous rover / racecar_
### > Hardware
* Compute: NVidia Jetson TX1 (also runs on desktop)
* Lidar: RP Lidar A8 (unused)
* Motor Controller: Enertion Focbox VESC
* Motor: Traxxas Velineon 3500 (3500kv, bldc)
* Servo: Traxxas slash 4x4 stock servo
* Servo Contoller: Adafruit 16 channel PWM driver, pca9685 type
* Gamepad: Logitech F710 wireless usb
* HDD: Samsung 500gig
* Camera / Pose / IMU: Intel Realsense T265 binocular tracking camera
### > Software
* Control code: C++, note: does NOT use MIT Racecar / F1Tenth code / architecture
* ROS: Tested on Noetic & Kame
* Camera: realsense2 API
* Philosophy:
* Code for clarity, not re-use, i'm not microsoft
* Minimal syntax / class use / templates, see above
* Experimental parameters, not simplistic physical models - except when all u have is simplistic models
