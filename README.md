# Behavioral and Path Planning for Self-Driving Cars

This project is part of the Self-Driving Car Engineer Nanodegree Program by [Udacity](https://www.udacity.com/).

This project implements the **Behavioral planner** and the **Motion planner** for self-driving cars.

**Goal:** Drive the vehicle safely in highway, autonomously. The vehicle should:
- drive not faster than 50 mph
- change lane if necessary
- change lane safely
- drive drive smoothly (avoid exceeding maximum acceleration and jerk)
- avoid collision

**Simulator:** can be downloaded from [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)


**Information provided:** 
- Highway waypoints $(x, y, s, d_x, d_y)$
- Ego-car position $(x, y, s, d, yaw, speed)$
- Sensor fusion consists of other vehicles information $(id, x, y, v_x, v_y, s, d)$


**Output:** 
- Path points in global coordinate system $(x,y)$ that the car will visit sequentially every 0.02 seconds


### Methodology
This solution has a `planner` class that consists of:
- **Behavioral planner (BP)** that  performs high-level decision making such as change lane, keep lane, reduce speed, or speed up;
- **Motion planner (MOP)** that generates trajectory correponding to what behavioral planner dictated.

#### Behavioral planner (BP)
This module performs high-level decision making such as keep lane, change lane, etc. The way that BP makes decision is as follows.

A falg vector has been defined, `vector<bool> too_close`, that indicates whether each lane is occupied by other vehicles or not.
The information of other vehicles is provided by  `sensor_fusion`.

First, BP checks if there is a vehicle infront of the ego-car and in the same lane or not:

```
if  (vehicle_s > ego_s) && (vehicle_s - ego_s < 30) && (vehicle_lane == current_lane)
    too_close[current_lane] = true;
```

If there is no vehicle in the current lane, the ego-car just keeps the current lane and goes as fast as possible (not more than the speed limit).

If there is a vehicle in the same lane as the ego-car, and infront of the ego-car with the distance of less than 30 meters the flag of the current lane becomes `true`.
In this case, BP decides whether it needs to *switch lane* (to left or right) or *keep the current lane* and *reduce speed*.

If there is a vehicle infront of the ego-car, BP checks the left lane, if there is enough **gap** on the left lane, issue the lane change. If the left lane is not available or it is occupied, BP check the right lane.

The **gap** on the adjacent lanes is calculated as follows:

```
if (vehicle_s < ego_s + 15 && vehicle_s > ego_s - 15 && vehicle_d >= 0)
    too_close[vehicle_lane] = true;
```

In the above psudocode, given the frenet $s$ of othe vehicles and the ego-car, BP checks if there is a 30 meters gap  (+/- 15 meters infront and back of the ego-car) in the adjacent lanes.
If there is no gap, the corresponding lane is marked as occupied.

In all cases other than the cases that ego-car must reduce its speed, BP check the speed and if it is less than the maximum desirable speed, it increases the speed.


#### Motion planner (MOP)
The motion planner module in the `Planner` class, generates trajectories (sets of points) for the controller of the ego-car to follow.

These trajectories is generated according to BP's decision. To have a smooth driving, MOP consideres points in 60, 90 and 120 meters from the current position of the ego-car,
 and fits a spline to these points. 
 
Finally, we sample from the trajectory and add it to the next points that vehicle needs to follow. The velocity of the ego-car depends on the  spacing of the points.

 
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  $[x,y,s,dx,dy]$ values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

