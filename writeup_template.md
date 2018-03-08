## Project: Search and Sample Return
### Using a simulated rover to map its environment, find samples and return them.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.



**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

[thresholds]: ./writeup_src/2018-03-08_thresholds.png
[transformations]: ./writeup_src/2018-03-08_1130_transformations.png
[sim_settings]: ./writeup_src/2018-03-08_simulator_settings.PNG
[req_met]: ./writeup_src/2018-03-08_rover_run03_mission_accomplished.PNG

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

I used most of the functions as they were. For the perspective transformation I used the existing calibration image and read the four points from it that I then passed to the `perspective_transform()` function.

For the detection of navigable terrain, obstacles and rocks, I defined a generic threshold function that works for all of these.

The function input arguments were extended by a new vector resulting in a function that has min values (`rgb_above`), max values (`rgb_below`) available. The algorithm principle is the same as before.

In order to identify the rock, the calibration image was analysed in _the gimp_ resulting in a color code vector for the rock color plus a relative range for the color codes. The resulting min and max color codes are passed to the `color_thresh` function. To make sure that the color thresholds are within the valid range, the two threshold vectors are clipped between 0 and 255.

![thresholds][thresholds]

For coordinate transformation, I used the algorithms from the course and added the two functions `wall_dist` and `sense_ahead` that are useful for wall crawling.

`wall_dist` finds all `y` pixels at a defined `x` distance and gets their lowest and highest value, giving the navigable terrain to the right and to the left from rover perspective. With this method I have tried to do a simple implementation of what I would do if I got the task to follow a wall when looking at a screen showing the camera picture.

`sense_ahead` find the point of navigable terrain that is furthest away from the rover at a given angle. I use this later for autonomous mode to find out when to stop and turn and also to dynamically adapt rover speed.

The following image shows the transformation steps as taught in the course: original image, perspective transformation, thresholding (here of the navigable terrain) and transformation to rover coordinates. While the course teaches the usage of an average polar vector to determine the direction of travel (see red arrow), I chose the above described method to find the distance to the walls at a certain distance ahead the rover 

![transformations][transformations]


#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

1) Source and destination points defined. Coordinates read from calibration image and hard-coded into the _source_ list. Destination is calculated from the size of the image and the desired scale.

2) Perspective transformation applied by simple call of the previously defined function `perspect_transform`

3) Defining thresholds for the previously defined universal threshold function: navigable terrain is between [160,160,160] and [255,255,255]; obstacles are below this, e.g. between [0,0,0] and [160,160,160] and rocks are identified from [162,133,7] +/- 50 for each field.

4) Each image (navigable, rock and obstacle) is transformed to rover coordinates by simple call of the _rover_coords_ function

5) The `pix_to_world` function is used to transform to world coordinates. From the databucket `data`, the current rover parameters (xpos, ypos and yaw) are used as attributes together with the previously generated rover coordinates from the navigable, rock and obstacle images.

6) The worldmap is updated as shown in the example. Layers were chagned in a way that navigable terrain appears green, obstacles appear blue and rocks appear red. Further, rocks gain color saturation more quickly with the goal to make them visible more easily.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

##### Perception
For the `perception_step()` function I basically copied the functions that I previously tested in the _jupyter notebook_. Small adaptations needed to be done to get the data put to the `Rover` object. However, I was not able to show the rover's view on the left side of the screen.

The main changes I did for the percpetion were the following two:

a) In order to improve **fidelity**, I only allowed images to be part of the map when the _pitch_ and _roll_ angles were within defined ranges. This way, incorrect perspective transformations are avoided.

b) To enable the rover to act accordingly, I implemented a **stuck detection** in the perception step. It's a simple state machine, looking if the rover is standing still during a certain time period while throttle is on. It returns to _normal_ when the rover is moving again.

##### Decision
One significant change for the `decision_step()` was the implementation of distance based decisions instead of vectors of navigable terrain.

This is on one hand reflected in the change to `stop` mode and back to `forward` that is now based on the navigable distance at zero degrees ahead instead of the amount of visible pixels.

On the other hand the steering was changed to the wall distance at a certain distance ahead so that steering depends on the actual distance to the wall.

Further, additional modes were added: _findwall_ and _unblock_.

_findwall_ is meant to find a wall in the beginning of the wall crawling algorithm. It was implemented because the rover would go in circles to the left when spawned in a free room.

The _unblock_ mode is activated when the `perception_step()` detects that the rover is stuck. It simply does a time limited right turn in place follwed by a time limited acceleration at steering zero. I tested the implementation of a rotation by a defined angle, but the physics engine would sometimes deflect the rover to the opposite direction, causing unexpected results for the algorithm that would need to be more complex. The time limitation is a reasonable workaround for this issue.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

The rover was run with the follwing settings, resulting in 19..22 FPS on my machine.
![Simulation Settings][sim_settings]

##### Driving
My main approach for driving was to imitate a human driver who wants to do wall crawling. This is why I based perception and decision on cartesian coordinates and tried to hold a certain distance to the wall. 

The percpetion step is very simple and just looks at the wall distance at a certain distance ahead the rover. This might be improved by adapting this distance to the circumstances (complexity of the terrain and rover velocity) or looking at the direction of the wall.

The steering was implemented with a simple 2 point controller that adjusts the steering angle to zero when between two threshold and puts the steering to the maximum angle to get the rover back when its ouside the distance boundaries. This has the disadvantage that the rover is tilting along the _roll_ axis what makes mapping more challenging but the extreme steering also reduces the rovers speed what is a downside for mapping time.

For the velocity I implemented the adaptation of the maximum velocity to the visible distance of navigable terrain. Unfortunately, due to the rough steering method, the speed adaptation would not give an advantage for the mapping time.

Driving can easily be improved by driving smoother and more adapted. PID controllers for throttle and steering angle are a possible approach but also more advanced functions like slip control by controlling the throttle instead of giving a fixed value.

##### Mapping
To get a map that is as complete as possible, a wall crawler was chosen. However, due to poor driving, the rover would often tilt around _roll_ and _pitch_ and decrease map fidelity. This is why I used simple threshold windows for these angles to filter out mapping images that are not appropriate for mapping.

This sceenshot shows that the algorithm leads to a result that meets the requirements: at least 40% of the environment with 60% fidelity and one sample found.

![Requirements met][req_met]

A solution for this issue would be to take the pitch and roll angles into account for the perspective transformation.

Another possibility to improve fidelity might be to weight the map pixels depending on their distance from the rover: the closer they are, the more important they are.


##### Unblocking
Unblocking was solved in two independent parts: one in the `perception_step` and one in the `decision_step`.

In the `perception_step` it's a simple state machine, looking if the rover is standing still during a certain time period while throttle is on. It would then go to a _stuck_ state. It returns to _normal_ when the rover is moving again.

In the `decision_step` the _unblocking_ mode is activated when the rover is _stuck_. It simply does a time limited right turn in place follwed by a time limited acceleration at steering zero.

The unblocking algorithm works fine for this map and project, but it is not successful in a lot of situations that I created manually. Better strategies, that see, if the current method is successful at all, would be a good improvement.

##### Pickup and return
To make this rover a useful tool in the simulation, it should not only find the yellow rocks but also collect them and return to the starting point.

My approach would be to stay on the wall crawler's path while calculating the distance to the detected rock. The distance should decrease while the rover is wall-crawling until a minimum is reached. Then the rover location shall be memorized, it shall approach the rock, pick it up and return to the location it left the crawling path.

To return to the starting point (that is memorized in the beginning of the mission), the rover would face the home position and follow obstacle walls until is has navigable terrain in the direction of home.