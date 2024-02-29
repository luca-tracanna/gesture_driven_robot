# Gesture Driven Robot

## Introduction

This project was made by me and [Daniele Borgna](https://github.com/Borgna02).

Our goal was to implement a robot that can be moved with hand gestures.
The robot can follow direct comands sort of like a drone or go to a specified location autonomously.

## Gesture tracking

With the help of the [Mediapipe](https://developers.google.com/mediapipe) library we can easily extract the user's hand "landmarks", after that with some simple math we can tell how many fingers are being held up or which way the user is pointing at.

## Movement options

The robot can be moved either by:
* Pointing with the index finger one of the available directions: forward, front-left, front-right left and right.
* Providing an index of one of the marked locations by raising the appropriate number of fingers.

In manual mode the robot will dodge obstacles in it's path by scanning area in front of it with the provided sonar.

## Odometry

Since movement is never perfect outside of a simulated environment, we needed some sort of odometry system to tell exactly where the robot was going when in automatic mode. We decided to use [AprilTags](https://april.eecs.umich.edu/software/apriltag) to create fixed reference points for the robot to detect with it's camera.

Such tags are placed all over the surring area and can be used to calculate approximately the robot's heading and position with a small margin of error.

## Hardawre choice

We chose a Kobuki base for our project, a simple two-wheeled robot.
The base recives comands from a Raspberry Pi, which is also responsable for capturing images from the camera, detecting tags and reading distance data from the sonar.

The sonar is made from three HC-SR04 ultrasonic distance sensors, each mounted on a servo motor. Both sensors and servos are controlled by an Arduino Nano which is hooked to the Raspberry Pi via USB for both power and to transfer sensor data.

## Code stucture

Everything is written almost entierly in Python, except for the camera access and the raw movement code which had to necessarily be written in C++.

The code is structured in modules. Each module that doesn't require direct harware acces such as the controller is executed in a Docker container for better flexibility, dependecy management and error handling.

Modules use the MQTT protocol to comunicate with eachother, that is why we also use Docker to spin up a mosquitto broker.


## Further reading

A much more in depth description of the entire project and it's implementation can be found in [documento.pdf](https://github.com/luca-tracanna/gesture_driven_robot/blob/main/documento.pdf) (in italian).

## Relevant links

* Our tutor [Prof. Giovanni de Gasperis](https://www.disim.univaq.it/GiovanniDeGasperis/119/).

* Laboratory organization: [AAAI-DISIM-UnivAQ](https://github.com/aaai-disim-univaq).

* [ISRLAB Syllabus](https://univaq.coursecatalogue.cineca.it/insegnamenti/2023/35824/2015/9999/10006?coorte=2021&schemaid=10906).
















