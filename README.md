# 2024 Winter ECE279AS Project 7: Low-light Enhancement and Object Tracking with PTZ Camera
# PhyCV-CUDA

Welcome to PhyCV-CUDA! This is the CUDA/C++ version of the [PhyCV (Physics-inspired Computer Vision) library](https://github.com/JalaliLabUCLA/phycv/). It is specifically optimized for edge devices with NVIDIA GPUs. This repo is developed by [Jalali-Lab](https://photonics.ucla.edu/) @ UCLA.
 
## Introduction 

PhyCV CUDA holds the source files needed to run the C++/CUDA versions of the [PhyCV](https://github.com/JalaliLabUCLA/phycv/) algorithms. The PhyCV CUDA source code can be built and run on any machine with C++/CUDA support. Testing and benchmarks are done on NVIDIA's Jetson Nano. 

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
