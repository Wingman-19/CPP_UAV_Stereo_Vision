# Obstacle Avoidance Using Stereo Vision

## Introduction
This document gives an overview of our process for performing obstacle avoidance for unmanned air vehicles using stereoscopic vision. The preceeding sections discuss the steps that the code will execute ([Overall Process](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/blob/master/README.md#overall-process)) as well as the algorithm for determining how to avoid an obstacle ([Section Selection](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/blob/master/README.md#section-selection) and [Movements](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/blob/master/README.md#movements)). There are sections for terminology ([Terms](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/blob/master/README.md#terms)) and links to the papers and references that we used throughout the process ([Links / Papers](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/blob/master/README.md#links--papers))

## Requirements
  * Avoid static obstacles using a stereoscopic camera while moving from point A to point B
  * Avoid moving obstacles using a stereoscopic camera while moving from point A to point B

## Overall Process
  1. Open the zed camera
      * Set the parameters here as well
  2. Create the disparity image
      * Using the ZED SDK this is really easy to do
  3. Partition the disparity image into 9 sections
      * Keep track of the starting point for each section (Both horizontally and vertically)
  4. Count the number of pixels in each section that have a value above a given threshold
      * Calculate the percentage for each section
  5. Select the section that has the lowest percentage
      * The section that is selected must have a percentage less than a specified percentage threshold or it will not be selected
      * Discussed in more detail in the [Section Selection](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/blob/master/README.md#section-selection) section
  6. Move in the direction of the selected section
      * Discussed in more detail in the [Movements](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/blob/master/README.md#movements) section
  7. Repeat until the destination is reached
  8. Close the camera
  
## Section Selection
  1. A disparity threshold is determined at the start (120 for now)
  2. A threshold for the percentage of pixels that can be above the disparity threshold is determined at the start (20% for now)
  3. The pixels in each section that have a disparity that is higher than the disparity threshold are counted
  4. The percentage for each section is calculated
  5. A section is determined by selecting the section with the smallest percentage
      * If this section has a percentage higher than the percentage threshold, then it is not selected and a default section is selected (More information on how the UAV moves in this situation in the [Movements](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/blob/master/README.md#movements) section)

## Movements
  * The sections will be numbered from 0 - 8 starting with the top right corner
    * A section value of -1 represents the default section
  * Each section specifies a given action
    * The right 3 sections move the UAV to the right a set number of feet
    * The left 3 sections move the UAV to the left a set number of feet
    * The top 3 sections increase the altitude of the UAV
    * The bottom 3 sections decrease the altitude of the UAV
    * The middle section has the UAV fly forward
    * The default section has the UAV rotate in place
    
## Required Installations to use the Jetson TX1 and the ZED Camera
  * **JetPack**: JetPack is used to flash the Jetson TX1 and add libraries like CUDA and VisionWorks (We are using JetPack 3.0)
    * Information about JetPack can be found [here](https://developer.nvidia.com/embedded/jetpack-notes)
  * **ZED SDK for Jetson TX1**: The ZED SDK is used to be able to capture the disparity images and run programs like the ZED Depth Viewer and ZED Explorer
    * The ZED SDK can be downloaded [here](https://www.stereolabs.com/developers/release/2.0/#sdkdownloads_anchor)
    
## JetPack Installation Issues and Solutions
  * The installation guide that we followed can be found [here](http://docs.nvidia.com/jetpack-l4t/index.html#developertools/mobile/jetpack/l4t/3.0/jetpack_l4t_install.htm)
  * JetPack must be run on a 64 bit host machine that is running Ubuntu 14.04 so that the Jetson can be flashed
    * We used a virtual machine that ran Ubuntu 14.04
  * When flashing the Jetson TX1, there are two options for the Network Layouts
    * At first, we attempted to use the second option (Device get IP assigned by DHCP server on host and access Internet via host machine) but this was running into issues where the connection to the host was being refused
      * All of the tutorials and documentation we found used the first option so finding a solution for this issue was difficult
    * We were able to get access to a router and ended up using the first option (Device access Internet via router/switch) to flash the Jetson TX1
      * This method was easier than the other and we ran into fewer issues
  * Another issue that we found was that when we attempted to ssh into the Jetson TX1, we were getting a connection time out error
    * This issue was solved when we logged out of the Ubuntu account and logged into the Nvidia account on the Jetson TX1
    * We are not sure why exactly this worked, but we were able to immediately ssh into the Jetson and therefore download and install all of the post installation libraries
    
## Terms
  * **Disparity Map**: When two images are taken from slightly different locations, the objects in the image appear to shift. This apparent shifting (or difference in pixel positions) between the two images is called disparity. The disparity between the two images creates the disparity map.
  * **SSH**: SSH is also known as Secure Socket Shell. It is a protocol that provides an administrator with a secure way of remote accessing another computer.

## Links / Papers
  * [Stereoscopic Vision Papers](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/tree/master/Stereo%20Vision%20Papers)
  * [Comparative Presentation of Real-Time Obstacle Avoidance Algorithms Using Solely Stereo Vision](http://83.212.134.96/robotics/wp-content/uploads/2011/12/Comparative-Presentation-of-Real-Time-Obstacle-Avoidance_Kos.pdf)
  * [Stereo vision obstacle avoidance using depth and elevation maps](https://robotica.dc.uba.ar/wp-content/papercite-data/pdf/pire2012.pdf)
  * [JetPack Installation for the Jetson TX1](http://docs.nvidia.com/jetpack-l4t/index.html#developertools/mobile/jetpack/l4t/3.0/jetpack_l4t_install.htm)
  * [JetPack 3.0 Release Notes](https://developer.nvidia.com/embedded/jetpack-notes)
  * [ZED SDK Download](https://www.stereolabs.com/developers/release/2.0/#sdkdownloads_anchor)
