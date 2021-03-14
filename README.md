# Overview
   
## Project Introduction

The goal is to safely navigate around a virtual highway with other traffic driving +/-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided. There is also a sparse map list of waypoints around the highway. 

## Overall Goals of a Successful Drive

- The car should try to go as close as possible to the 50 MPH speed limit (passing slower traffic when possible). Note that other cars will try to change lanes too. 
- The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 
- The car should be able to make one complete loop around the 6,946m highway. 
- The car should'nt experience total acceleration greater than 10 m/s^2 and jerk greater than 10 m/s^3.

Since the car is trying to go 50 MPH, it should take about 5 minutes to complete 1 loop. 

# TO RUN

Simuly run these commands:

```
mkdir build
cd build/
cmake .. && make
./path_planning
```

or simply run `run_me.sh` after creating the `build` directory (`mkdir build && chmod +x run_me.sh && ./run_me.sh`)

# Model Documentation

## Car Moves Smoothly

- Car also has a relatively gentle acceleration (`0.20` which is under the required `10 m/s^2` acceleration limit).
- Car uses the `spline` library to calculate smooth movements for future waypoints which prevents large acceleration and jerk.
- Previous waypoints are considered while creating new waypoints to ensure a smooth and continuous drive.

## Car Goes Close to 50 MPH

- Model contains a `MAX_SPEED` of 49.0 MPH and will only proceed faster if there is no car in front of the lane it is pursuing (no car directly in front of it or in the lane it is attempting to change to).

## Car Smoothly Changes Lanes

- Car will default to the middle lane but will try and go to the left or right lane if there is a car blocking the immediate front (within 30 meters of car).
- Car uses the same smoothing techniques when changing lanes.

## Car Avoids Collisions

- Car will smoothly decelerate if a car is directly in front of it (within 30 m)
- Car will not move into a new lane if it estimates a car will be within 30 meters in front of it during the lane change.

# Data Provided

## Map

The map of the highway is in `data/highway_map.txt` where each waypoint in the list contains `[x,y,s,dx,dy]` values. 

- `x` and `y` are the waypoint's map coordinate position
- `s` value is the distance along the road to get to that waypoint in meters
- `dx` and `dy` values define the unit normal vector pointing outward of the highway loop

## Simulator Data

Here is the data provided from the Simulator to the C++ Program

### Main Car's Localization Data (No Noise)

- `["x"]` The car's x position in map coordinates
- `["y"]` The car's y position in map coordinates
- `["s"]` The car's s position in frenet coordinates
- `["d"]` The car's d position in frenet coordinates
- `["yaw"]` The car's yaw angle in the map
- `["speed"]` The car's speed in MPH

### Previous Path Data Given to the Planner

> Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

- `["previous_path_x"]` The previous list of x points previously given to the simulator
- `["previous_path_y"]` The previous list of y points previously given to the simulator

### Previous Path's End `s` and `d` Values 

- ["end_path_s"] The previous list's last point's frenet `s` value
- ["end_path_d"] The previous list's last point's frenet `d` value

### Sensor Fusion Data

A list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's:

- unique ID
- `x` position in map coordinates
- `y` position in map coordinates
- `x` velocity in m/s
- `y` velocity in m/s
- `s` position in frenet coordinates
- `d` position in frenet coordinates 


The highway's waypoints loop around so the frenet `s` value (distance along the road) goes from 0 to 6945.554.

# Basic Build Instructions

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./path_planning`

# Details about Run

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

# Dependencies

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
    ```shell
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

