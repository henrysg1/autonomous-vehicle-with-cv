# Autonomous Vehicle Computer Vision System

![image](https://github.com/henrysg1/autonomous-vehicle-with-cv/assets/115489513/0f6a188b-b93b-4687-bb44-baf47d97d122)

This project involves the development of an autonomous vehicle capable of line following and symbol recognition using computer vision. The system is implemented using C++ and OpenCV on a Raspberry Pi, with additional control provided by Arduino microcontrollers.

## Introduction

This project demonstrates the creation of a computer vision system for an autonomous vehicle. The system is capable of following lines on the ground and recognising symbols to perform various tasks. The primary components include a Raspberry Pi, a camera, and several Arduino microcontrollers.

## System Overview

### Hardware Overview

The hardware setup includes:
- Raspberry Pi with a connected camera
- Three Arduino Nanos for various controls
- Motors and servo for vehicle movement
- Ultrasonic sensor for distance measurement
- Inertial Measurement Unit (IMU) for incline detection
- Audio amplifier and speaker for audio output
- Level shifter for voltage compatibility between Raspberry Pi and Arduinos

### Software Overview

The software is implemented in C++ using OpenCV for image processing and Pi2c for communication between Raspberry Pi and Arduinos. The main tasks include:
- Line following using color detection
- Symbol recognition for executing specific sub-routines
- Traffic light detection
- Shape detection
- Incline and distance measurement

## Design and Implementation

### Color Detection

![image](https://github.com/henrysg1/autonomous-vehicle-with-cv/assets/115489513/0e9b67f3-adf2-49a0-a22a-7f6c7b21191d)

Color detection is achieved by converting the camera image from BGR to HSV color space and filtering specific colors. This technique is used for both line following and symbol recognition.

### Line Following

![image](https://github.com/henrysg1/autonomous-vehicle-with-cv/assets/115489513/30b820ae-ab7b-4963-a0ec-ddf3d2b21720)

The vehicle follows a line by detecting the color of the line and adjusting the motor speeds to keep the line centered in the camera's view. The system uses HSV values to identify the line and calculates the position of the line's middle pixel to determine the vehicle's direction.

### Symbol Recognition

![image](https://github.com/henrysg1/autonomous-vehicle-with-cv/assets/115489513/7743a239-2dff-439c-b6a7-ef6a820ce3bb)

The system recognizes symbols by capturing images from the camera, converting them to HSV, and comparing them to known symbol images stored in an array. Contour detection is used to identify the shape of the symbols.

## Sub-system Implementation

### Servo Control

![image](https://github.com/henrysg1/autonomous-vehicle-with-cv/assets/115489513/d97723c6-49e1-49bb-947f-bfab9c065989)

The camera's position is controlled by a servo motor, allowing it to switch between line following and symbol recognition modes. The servo is controlled via I2C commands from the Raspberry Pi to the Arduino.

### Level Shifter

![image](https://github.com/henrysg1/autonomous-vehicle-with-cv/assets/115489513/cad92a26-4111-4bed-8298-9eaf9976a041)

A level shifter is used to ensure safe voltage levels between the 3.3V logic of the Raspberry Pi and the 5V logic of the Arduinos.

### Audio Amplifier and Speaker

![image](https://github.com/henrysg1/autonomous-vehicle-with-cv/assets/115489513/64f70b59-5695-47d7-88a1-3532f008c557)

An audio amplifier circuit using the LM386 IC is implemented to provide audio output from the Raspberry Pi. This is used for outputting measurements and other notifications.

## SAE Level of Autonomy

The system achieves SAE Level 4 autonomy, meaning it can handle most driving tasks independently but may not perform well in all conditions without human intervention.

## Further Challenge Objectives

### Traffic Light Detection

The system includes a function to detect green traffic lights and wait until it is safe to proceed.

### Coloured Line Following

The vehicle can switch to following colored lines based on symbol detection, taking shortcuts on the track when indicated.

### Shape Detection

![image](https://github.com/henrysg1/autonomous-vehicle-with-cv/assets/115489513/8c0b2871-e1ec-4b03-b19f-85757beefeaf)

The system can identify and label basic geometric shapes within the camera's view.

### Incline Measurement

Using the MPU-9250 IMU, the system measures the vehicle's incline and provides audio output of the angle.

### Distance Measurement

An ultrasonic sensor measures the distance to objects, and the system announces the distance using the speaker.

## Conclusion

### Completed Challenges

The project successfully implements a functioning computer vision system capable of following lines and recognizing symbols. Additional features include traffic light detection, shape recognition, and audio output for measurements.

### Improvements

Future improvements could include more reliable symbol recognition for colored lines, audio output for shape detection, and successful communication of incline measurements to the Raspberry Pi.
