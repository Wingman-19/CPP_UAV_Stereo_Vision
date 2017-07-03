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
    * A section value of 9 represents the default section
  * Each section specifies a given action
    * The right 3 sections set a new waypoint to the right
    * The left 3 sections set a new waypoint to the left
    * The top 3 sections increase the altitude of the UAV
    * The bottom 3 sections decrease the altitude of the UAV
    * The middle section has the UAV continue to the destination waypoint
    * The default section has the UAV rotate in place
    
## Terms
  * **Disparity Map**: When two images are taken from slightly different locations, the objects in the image appear to shift. This apparent shifting (or difference in pixel positions) between the two images is called disparity. The disparity between the two images creates the disparity map.

## Links / Papers
  * [Stereoscopic Vision Papers](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/tree/master/Stereo%20Vision%20Papers)
  * [Comparative Presentation of Real-Time Obstacle Avoidance Algorithms Using Solely Stereo Vision](http://83.212.134.96/robotics/wp-content/uploads/2011/12/Comparative-Presentation-of-Real-Time-Obstacle-Avoidance_Kos.pdf)
  * [Stereo vision obstacle avoidance using depth and elevation maps](https://robotica.dc.uba.ar/wp-content/papercite-data/pdf/pire2012.pdf)
