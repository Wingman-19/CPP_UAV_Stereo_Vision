# Obstacle Avoidance Using Stereo Vision
This code will be used to execute obstacle avoidance using a stereoscopic camera (The ZED camera).

## Overall Prcess
  1. Open the zed camera
  2. Create the disparity image
  3. Partition the disparity image into 9 sections
  4. Count the number of pixels in each section that have a value above a given threshold
      * Calculate the percentage for each section
  5. Select the section that has the lowest percentage
      * The section that is selected must have a percentage less than a specified percentage threshold or it will not be selected
  6. Move in the direction of the selected section
  7. Repeat until the destination is reached
  8. Close the camera

## Section selection and Movements
  * The sections will be numbered from 0 - 8 starting with the top right corner
  * Each section specifies a given action
    * The right 3 sections set a new waypoint to the right
    * The left 3 sections set a new waypoint to the left
    * The top 3 sections increase the altitude
    * The bottom 3 sections decrease the altitude
    * The middle section has the UAV continue to the destination waypoint

## Links / Papers
* [Stereoscopic Vision Papers](https://github.com/Wingman-19/CPP_UAV_Stereo_Vision/tree/master/Stereo%20Vision%20Papers)
* [Comparative Presentation of Real-Time Obstacle Avoidance Algorithms Using Solely Stereo Vision](http://83.212.134.96/robotics/wp-content/uploads/2011/12/Comparative-Presentation-of-Real-Time-Obstacle-Avoidance_Kos.pdf)
* [Stereo vision obstacle avoidance using depth and elevation maps](https://robotica.dc.uba.ar/wp-content/papercite-data/pdf/pire2012.pdf)
