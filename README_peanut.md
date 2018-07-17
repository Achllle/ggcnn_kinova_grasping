# GGCNN demo

## Installation

Temporary installation instructions until we create an install script

* go to a workspace or create a new workspace in a virtualenv
* clone this repo into src
* pip install -r requirements.txt
* clone kinova_ros from bitbucket
* make sure realsense is in the repo and the SDK is installed properly and on the latest version
* catkin_make
* download the weights as in the official README
  * move them to `src/networks/ggcnn_rss`. The launchers will detect them automatically there.

See also [their README](README.md)

### Potential Issues

* if you get an `ImportError` for `em`: `pip install empy`
* if you get an `ImportError` for `rospkg`: make sure your PYTHONPATH is correct
* make sure all the things are sourced

## Execution

1. `roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=m1n6s300`
2. `roslaunch m1n6s300_moveit_config m1n6s300_demo.launch`
3. `roslaunch peanut_moveit arm_execution_wrapper.launch`
4. `roslaunch ggcnn_kinova_grasping launch_all_the_things.launch`
5. `rosrun ggcnn_kinova_grasping kinova_closed_loop.py`

### Notes

The `launch_all_the_things.launch` file has an argument to set the location of the ggcnn model file.
It's default is `$(find ggcnn_kinova_grasping)/../../networks/ggcnn_rss/epoch_29_model.hdf5` which
corresponds to `your_ws/src/networks/ggcnn_rss/epoch_29_model.hdf5`.

