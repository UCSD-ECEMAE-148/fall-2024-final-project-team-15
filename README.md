
# Selfie Robot
MAE 148 Final Project
Fall 2024 Team 15
![](fullcar.jpg)

## Table of Contents
- [Team Members](#team-members)
- [Abstract](#abstract)
- [What We Promised](#what-we-promised)
- [Accomplishments](#accomplishments)
- [Challenges](#challenges)

## Team Members
Anurag Gajaria - MAE Controls & Robotics (MC34) - Class of 2025 - [LinkedIn](https://www.linkedin.com/in/anurag-gajaria/)

Jimmy Nguyen - MAE Controls & Robotics (MC34) - Class of 2025

Michael Ramirez - MAE Controls & Robotics (MC34) - Class of 2025

Jingnan Huang - MAE (Exchange Student)


## Abstract
This projects goal is to follow a selected individual and follow them at a set distance. Once that individual makes a certain gesture (in this case a fist), the robot starts a countdown of 3 seconds and takes a picture. This picture is then saved to a local folder until it can upload the image to a google drive.

This project uses RoboFlow and Depthai for person detection, person following, and gesture identification. It also utilizes Google Cloud Console to create an application to create an application to upload pictures to the project's Google Drive. 

## What We Promised
### Must Have
- Detect a person and follow them at a set distance
- Detect the appropriate gesture and take a picture
- Upload picture to Google Drive

### Nice to Have
- Focus only on the closest person
- Have multiple gestures, one to stop and one to go

## Accomplishments
- Person Detection and Following
	- We were able to create a Depthai model that first finds a person and follows behind them at a set distance of 1 meter.
- Gesture Detection
	- We were able to set up a gesture detection model that could identify rock, paper, and scissors.
	- We were able to use the rock gesture to take pictures.
- Picture Upload
	- We were able to integrate a data upload node that continually verifies the internet connection then uploads the taken pictures to google drive.

## Challenges
- We were having a hard time getting the camera to only focus on one person, as it would rather identify all people and move to the center of them.
- The gesture detection node and the person detection node would try to use the same camera, requiring us to launch the nodes separately.
- The google projects token would expire every hour, resulting in us having to manually create a new token every hour.
 
 ## Videos

### Person Detection 
We combined the Person detection model with the LIDAR, so the robot will automatically follow a person and keep in one meter distance.

[![Person Tracking](https://img.youtube.com/vi/v7JGEvncHDk/hqdefault.jpg)](https://www.youtube.com/watch?v=v7JGEvncHDk)

### Gesture Detection
We created a callback Timer to receive the prediction from the gesture detection model. Once it detects the "Rock", the robot would stop and take a picture after 3 seconds，and save it to the folder we specified. 

[![Gesture Detection](https://img.youtube.com/vi/e5u4Fdw_I-c/hqdefault.jpg)](https://www.youtube.com/watch?v=e5u4Fdw_I-c)

### Picture Upload
When the pictures are saved to the specified folder, it would be uploaded to the Google Drive, and then the folder would be cleared.

[![Data Upload](https://img.youtube.com/vi/IjKZK5SK12w/hqdefault.jpg)](https://www.youtube.com/watch?v=IjKZK5SK12w)

## Software
### RoboFlow

### Depthai

### Google Cloud Console + Google Drive

## Hardware
-   **3D Printing:**  Camera Stands and Jetson Nano Case
-   **Laser Cut:**  Base plate to mount electronics and other components.

**Parts List**

-   Traxxas Chassis with steering servo and sensored brushless DC motor
-   Jetson Nano
-   WiFi adapter
-   64 GB Micro SD Card
-   Adapter/reader for Micro SD Card
-   Logitech F710 controller
-   OAK-D Lite Camera
-   LD19 Lidar (LD06 Lidar)
-   VESC
-   Point One GNSS with antenna
-   Anti-spark switch with power switch
-   DC-DC Converter
-   4-cell LiPo battery
-   Battery voltage checker/alarm
-   DC Barrel Connector
-   XT60, XT30, MR60 connectors

_Additional Parts used for testing/debugging_

-   Car stand
-   USB-C to USB-A cable
-   Micro USB to USB cable
-   5V, 4A power supply for Jetson Nano

## Progress Updates:
[![Proposal](https://lh7-us.googleusercontent.com/docs/AHkbwyKaYouCxZZ74gkw2q0c3lYbNnYGJHuv0AslUVIlt26BxiwKSY2OqE8ucMnkGBHg7CNpC7f1DwirD0vReUjd870YFI62CMv8f9uakiW7SoUcIH14GJum=w1200-h630-p)](https://docs.google.com/presentation/d/1HUT9JTSkYMz4nOk4j9AcVhAh5s9QCuQKqXG9H8esP30/edit?usp=sharing)
[![Update 1](https://lh7-us.googleusercontent.com/docs/AHkbwyK7ucGkFOjssTNrdtaxzbAyWPI4xUVjmsQL6F9tswa_4LffFKxLkrtbWdnJAZ1kpFxQZv0N_9DlDu7YXKWvSrc4yhd153Jmz8kdJXbhJA5Rhs9QOrVL=w1200-h630-p)](https://docs.google.com/presentation/d/1CXbFTw29IVQMEC1bH4hkAPKb_hF3BV6XbaN0xDd4Gfc/edit?usp=sharing)
[![Update 2](https://lh7-us.googleusercontent.com/docs/AHkbwyKXzBv85ZnwzcKgSiO8ucP1KvLMfPRJuIS3jwjOYnm8W-y5DLLnfj_9zmDfqT7TzT0CLe9xyb_FASF3qinY_lBY6pVDujkd2jnbLVq29iVi3KQTSqtt=w1200-h630-p)](https://docs.google.com/presentation/d/1LNuAlSLpy5jb8v_36zEdEwpq6CLo5_kOLiZwK0dzoNM/edit?usp=sharing)
[![Update 3](https://lh7-us.googleusercontent.com/docs/AHkbwyJ3CfxAywzVAGdF4jcgtZYT4ttd4B3d7NS_GeiATv14UGKPCdNu0lS60KXgYmD_EFy-8qBnqH5xTf03zoBAYRlcvQ_IVUvlYj_nNROevG46vt0xJGc=w1200-h630-p)](https://docs.google.com/presentation/d/1XLpculDbEHgQv3KNwmUt6qnbv6uwGzyas07qGfh1bmU/edit?usp=sharing)
[![Update 4](https://lh7-us.googleusercontent.com/docs/AHkbwyK0Ed7o1JHTKUENWlKdIrkuDCo3-phfGcGvCsoawmFidUMcXZo1Nh1i5cv9oAUdC0nWRGPOqrRMeXem3-qHJTwzoV6DoC1I00IkKh35wOUby_U5PWc=w1200-h630-p)](https://docs.google.com/presentation/d/1JtPEuXL4Afvt7PSGOWgyTa4lmRQebnCBscVASes4pXU/edit?usp=sharing)
[![Final](https://lh7-us.googleusercontent.com/docs/AHkbwyKaYouCxZZ74gkw2q0c3lYbNnYGJHuv0AslUVIlt26BxiwKSY2OqE8ucMnkGBHg7CNpC7f1DwirD0vReUjd870YFI62CMv8f9uakiW7SoUcIH14GJum=w1200-h630-p)](https://docs.google.com/presentation/d/1WY-7qo8mRwJC0C6nT3Gl_l8i0WBEJi-sgSuun3u4sJQ/edit?usp=sharing)

## Project Reproduction
If you are interested in reproducing our project, here are a few steps to get you started with our repo:

1.  Follow instuctions on  [UCSD Robocar Framework Guidebook](https://docs.google.com/document/d/1YS5YGbo8evIo9Mlb0J-w2r3bZfju37Zl4UmdaN2CD2A/),  
    pull  `devel`  image on your JTN:  `docker pull djnighti/ucsd_robocar:devel`
2.  `sudo apt update && sudo apt upgrade`  
    (make sure you upgrade the packages, or else it won't work; maybe helpful if you run into some error  [https://askubuntu.com/questions/1433368/how-to-solve-gpg-error-with-packages-microsoft-com-pubkey](https://askubuntu.com/questions/1433368/how-to-solve-gpg-error-with-packages-microsoft-com-pubkey))  
    check if  `slam_toolbox`  is installed and launchable:  
    
    sudo apt install ros-foxy-slam-toolbox
    source_ros2
    ros2 launch slam_toolbox online_async_launch.py
    
    Output should be similar to:
    
    [INFO] [launch]: All log files can be found below /root/.ros/log/2024-03-16-03-57-52-728234-ucsdrobocar-148-07-14151
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [async_slam_toolbox_node-1]: process started with pid [14173]
    [async_slam_toolbox_node-1] 1710561474.218342 [7] async_slam: using network interface wlan0 (udp/192.168.16.252) selected arbitrarily from: wlan0, docker0
    [async_slam_toolbox_node-1] [INFO] [1710561474.244055467] [slam_toolbox]: Node using stack size 40000000
    [async_slam_toolbox_node-1] 1710561474.256172 [7] async_slam: using network interface wlan0 (udp/192.168.16.252) selected arbitrarily from: wlan0, docker0
    [async_slam_toolbox_node-1] [INFO] [1710561474.517037334] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
    [async_slam_toolbox_node-1] [INFO] [1710561474.517655574] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
    
3.  Since we upgrade all existing packges, we need to rebuild VESC pkg under  `/home/projects/sensor2_ws/src/vesc/src/vesc`  
    
    cd /home/projects/sensor2_ws/src/vesc/src/vesc
    git pull
    git switch foxy
    
      
    make sure you are on foxy branch  [![](https://private-user-images.githubusercontent.com/68310078/313373449-10398bc9-f546-497e-8e5f-9f380b39e018.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MzQyMTA4MDcsIm5iZiI6MTczNDIxMDUwNywicGF0aCI6Ii82ODMxMDA3OC8zMTMzNzM0NDktMTAzOThiYzktZjU0Ni00OTdlLThlNWYtOWYzODBiMzllMDE4LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNDEyMTQlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjQxMjE0VDIxMDgyN1omWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPThmNGM3Njg3YmViMzgwOWM2MTY3MDBmNjZkZDIyNmZkOTJiYWU1ZWNkNTI2ZjQwNjQ0MmIxNzMzNTFjNTU1N2EmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.87qr0cTrS4vbg7JwObLDhVFXCQkt8I7WWvz-IqSoe3A)](https://private-user-images.githubusercontent.com/68310078/313373449-10398bc9-f546-497e-8e5f-9f380b39e018.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MzQyMTA4MDcsIm5iZiI6MTczNDIxMDUwNywicGF0aCI6Ii82ODMxMDA3OC8zMTMzNzM0NDktMTAzOThiYzktZjU0Ni00OTdlLThlNWYtOWYzODBiMzllMDE4LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNDEyMTQlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjQxMjE0VDIxMDgyN1omWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPThmNGM3Njg3YmViMzgwOWM2MTY3MDBmNjZkZDIyNmZkOTJiYWU1ZWNkNTI2ZjQwNjQ0MmIxNzMzNTFjNTU1N2EmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.87qr0cTrS4vbg7JwObLDhVFXCQkt8I7WWvz-IqSoe3A)  
    Then, build 1st time under  `sensor2_ws/src/vesc/src/vesc`  
    
    colcon build
    source install/setup.bash
    
    Then, 2nd time but under  `sensor2_ws/src/vesc`  
    
    cd /home/projects/sensor2_ws/src/vesc
    colcon build
    source install/setup.bash
    
    Now, try  `ros2 pkg xml vesc`, check if VESC pkg version has come to  `1.2.0`  
    
4.  Install  **Navigation 2**  package, and related packages:  
    `sudo apt install ros-foxy-navigation2 ros-foxy-nav2* ros-foxy-robot-state-publisher ros-foxy-joint-state-publisher`
5. Pull this repository...
