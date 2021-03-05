Surface Edge Explorer (SEE++)
==============================================================================

## Description

SEE++ is a density-based Next Best View (NBV) planning approach for obtaining 3D
scene observations.

## Installation

- Install Dependencies
  - sudo apt install libnlopt-dev liblemon-dev
- Install ROS (includes PCL)
  - http://wiki.ros.org/melodic/Installation/Ubuntu (use full desktop install)
- Clone SEE into your catkin workspace
  - cd ~/catkin_ws/src && git clone https://github.com/robotic-esp/see-public
- Build SEE with catkin
  - cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
  - Using the 'Release' build enables PCL optimisations for the best performance

## Usage

- Specify the desired SEE parameters (i.e., view distance (d), radius (r),
  density (rho), frontier update limit (tau), occlusion search distance
  (psi) and visibility search distance (ups)), the sensor parameters
  (e.g., field-of-view, resolution and fps) and the ROS topic for sensor
  pointclouds (sen_pts_topic) in the launch file see_core/launch/run_see.launch
- Run the launch file
    - roslaunch see_core run_see.launch
- More information can be found on the doxygen webpage
  - https://robotic-esp.github.io/see-public/

## Dependencies
- ROS
- PCL
- NLOPT
- Lemon

## Project Owners

Rowan Border <rborder@robots.ox.ac.uk>

## References

https://github.com/robotic-esp/see-public

## Licence

MIT Licence

## Copyright

Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
