# Low-Light Pan Tilt Zoom Camera Object Tracker
 
## Introduction 

This repository holds the source files for running VEViD in conjunction with object detection for real time processing on the Jetson Nano. It also includes code for interfacing with and controlling a connected Arducam Pan-Tilt-Zoom (PTZ) camera, enabling enhanced object detection and tracking in low-light conditions. This project was accepted for a poster presentation at the NVIDIA GPU Technology Conference (GTC) 2024. The poster presentation can be found [here.](https://www.nvidia.com/gtc/posters/?search=Physics%20Inspired#/session/1705022965922001O2as)

## Folder Structure

- `assets`: sample input images/videos, sample results, documentations.
- `includes`: head files
- `src`: This folder contains the source code of PhyCV CUDA.
  - `main.cpp`: serves as the entry point for the application.
  - `options.cpp`: processes command-line options.
  - `video.cpp`: runs VEViD on input images, videos, and camera feeds.
  - `detect_net.cpp`: uses the Jetson Inference library to run object detection.
  - `vevid.cu`: the implementations of the vevid algorithms.
  - `kernels.cu`: the implementations of the CUDA kernels required to run VEViD. 
  - `controls.cpp`: the code for control and interfacing of/with the Arducam PTZ camera.

## Get Started

### Platforms 
- This repo uses `CUDA` to leverage parallel processing on the GPU. Make sure you have a compatible GPU and that you have set up `CUDA` before the installation.

- This repo requires the `jetson-inference` library for object detection. Clone the repository and build the project from source following the instructions [here](https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md).

- This repo is tested on NVIDIA Jetson Nano 4GB with the following versions:
  - Jetpack: 4.6.4
  - CUDA: 10.2.300
  - OpenCV: 4.1.1

### Instructions
```bash
# 1. Download the repo from GitHub
git clone https://github.com/{REPO_NAME}
# 2. cd into the repo
cd {REPO_NAME}
# 3. Compile
make
```
Now you should see the executable output `vevid` in the directory. We list typical usages of the repo below

Run VEViD on the video feed from the camera
```bash
./vevid
```

Run VEViD on a single image file. indicate the file location after `-i` 
```bash
./vevid -i ./assets/input_images/dark_road.jpeg
```

Run VEViD on a single video file. indicate the file location after `-v` 
```bash
./vevid -v ./assets/input_videos/video_campus.mp4
```

If you want to save the processed video, indicate saving location after `-w`. Note that saving the processed video may cause latency. So when the `-w` flag is turned on, the on-screen display is turned off.
```bash
./vevid -v ./assets/input_videos/video_campus.mp4 -w ./output/enhanced_campus.mp4
```

The default parameters of VEViD are defined in `includes/options.hpp`. You can change these parameters by using the `-p` and `-r` flags. Use `-p <PARAM=val>` where `PARAM` is one of the VEViD parameters `S, T, b, G` and `val` is a floating point number. Use `-r <width>,<height>` to specify the processed frame size. 


See all the options from the command line:
```bash
./vevid -h
```

Other Usages:

- To enable object detection, add `-d` flag to the command.

- To display timing information, add `-t` flag to the command.

- To enable `VEViD-Lite`, add `-l` flag to the command.

## PTZ Camera Tracking

The code for PTZ camera tracking and interface with the Arducam 14mp PTZ camera can be found in the controls.cpp and controls.hpp files. Currently, an automated tracking algorithm using Kalman filtering and the bounding box data from running object detection on the input frame can be found in the `process_camera()` function in the video.cpp file. This file uses the controller class declared and implemented in controls.hpp and controls.cpp respectively to control the connected PTZ camera. If you would like to add custom code for a PTZ camera of your choice, override the implementations in the controls.cpp source file. In order to run the automated tracking program, simply use `./vevid -d` with a connected Arducam PTZ camera.

## Link to Presentation Slides
https://personalmicrosoftsoftware0-my.sharepoint.com/:p:/g/personal/taejusyee_personalmicrosoftsoftware_ucla_edu/ETQ1_cDFDKlErm9_APUFn2IB4chv3lfY7ukUIL_T-8mJgg?e=dgIIj4
