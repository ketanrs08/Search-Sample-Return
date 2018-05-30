# Project: Search and Sample Return

[//]: # (Image References)

[image1]: ./Writeup_Images/Unity.png
[image2]: ./Writeup_Images/Sample_output.jpg
[image3]: ./Writeup_Images/example_grid1.jpg
[image4]: ./Writeup_Images/P_Transformed.png
[image5]: ./Writeup_Images/Thresh_images.png

[//]: # (Link References)

[link1]: ./Writeup_Images/Unity.png

---

## Writeup

### Objective:

Implement image processing and computer vision procedures to achieve autonomous navigation of a Mars rover. Provide the rover with perception and decision taking functionalities' to identify obstacles , navigable terrain and rock samples to collect.

### Steps:

#### 1. Simulator

The Simulation was run in a Unity simulator with the following configuration:

![image1]


#### 2. Collecting Test Images

The simulator is run in Training mode and the recording is started to collect the test data which consists of an image stream from the camera in front of the rover and the values of steering, throttle, brake, speed, position, pitch, yaw and roll. The Images are saved in the `IMG` folder and the telemetry data in a `csv ` file with all the values.

Sample Output Image:

![image2]


#### 3. Notebook Analysis

To create a map of the environment, the images have to be run through a series of following operations. The Notebook consisting of the functions for perception steps can be found here. This Notebook is used to test our `perception_step` and `process_image` functions.

* `perspect_transform ` - This step is performed to change the perspective from the rover front camera view to top down view. OpenCV functions `cv2.getPerspectiveTransform()` and `cv2.warpPerspective()` are used by defining the source and destination points in the images.

    Original Image:

    ![image3]

    Perspective Transformed Image:

    ![image4]


* `color_threshold` - To differentiate between the `navigable terrain`, `obstacles` and the `rock samples`, a color thresholding is performed. To identify the obstacle pixels and the navigable terrain pixels, a color threshold is applied to the `BGR` image with the limits `(160, 160, 160)`. To identify the rock sample pixels, the `BGR` image is converted to `HSV` and a color thresholding with `min_threshold: (60, 120, 110)` and `max_threshold: (135, 255, 255)`

    ![image5]


* `coordinate_transformations` - The warped and thresholded images are then converted from image coords to rover coords using `rover_coords` and map rover space pixels to world space pixels using `pix_to_world`. The final world map has each pixel has its own color based on `obstacle`, `navigable terrain` and `rock_sample` presence.



* `process_image` - This function is defined to pass stored images from the `test_data` folder and the telemetry values from the `.csv` file and implement the above defined perception. An output video is also created.


### Autonomous Navigation and Mapping

#### 1. `perception_step()`  
* The function is similar to the `process_image` defined earlier with an additions of instructions to drive the rover towards the samples when a `rock` is found and to update the `world_map` only when the `pitch` and `roll` are within defined limits. After processing each of the image the `Rover()` data is updated.



#### 2. `decision_step()`
* The function contains conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.

* `Rover.mode`has the following modes defined:

    * `forward` - The Rover keeps navitaing in the navigable terrain avoiding the obstacles with pre-defined throttle.

    * `stop` - If an obstacle is detected and there is no navigable region in front of the rover, this mode is set where the Rover comes to a complete halt and turns in the desired direction until a navigable terrain is found. This mode

    * `stuck` - If the Rover has a defind throttle and is stuck at an obstacle, this mode is used to back up the rover and steer  until a navigable terrain is detected and is free to move.

    * `rock_found` - When the Rover sees a `rock_sample`, this mode is set, where the Rover moves toward the sample and comes to a stop and collects the sample.

### Improvements

Below are a list of potential Improvements:
* Setup a `home` mode which can be set, when the Rover has finished collecting all the rocks and can return to the `start_position`

* Define boundaries to already navigated regions, so that the Rover does not visit the places it has already visited.
