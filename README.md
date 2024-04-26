# Mobile Robotics Individual Project Spring 2024
## Exploring Generalized Robotics Navigation: Novel Environment Navigation and Robot Adaptability Assessment

**Project Contributor**: Yeo Ke Wei 1005165

_SUTD, Design and Artificial Intelligence (DAI)_

[**Reference Code Contributors**](https://general-navigation-models.github.io): Dhruv Shah, Ajay Sridhar, Nitish Dashora, Catherine Glossop, Kyle Stachowicz, Arjun Bhorkar, Kevin Black, Noriaki Hirose, Sergey Levine

_Berkeley AI Research_

---

General Navigation Models are general-purpose goal-conditioned visual navigation policies trained on diverse, cross-embodiment training data, and can control many different robots in zero-shot. They can also be efficiently fine-tuned, or adapted, to new robots and downstream tasks. Our family of models is described in the following research papers (and growing):
1. [GNM: A General Navigation Model to Drive Any Robot](https://sites.google.com/view/drive-any-robot) (_October 2022_, presented at ICRA 2023)
2. [ViNT: A Foundation Model for Visual Navigation](https://general-navigation-models.github.io/vint/index.html) (_June 2023_, presented at CoRL 2023)
3. [NoMaD: Goal Masking Diffusion Policies for Navigation and Exploration](https://general-navigation-models.github.io/nomad/index.html) (_October 2023_)

## Overview
This contains the necessary codes to run and test the project, requiring the repo from [Dumpster robot](https://github.com/nethdeco/rtab_dumpster) and [GNM](https://github.com/robodhruv/visualnav-transformer).

## Setup

The codebase assumes access to a workstation running Ubuntu (tested on 20.04), Python 3.7+, and a GPU with CUDA 10+. It also assumes access to conda, but you can modify it to work with other virtual environment packages, or a native setup. It also requires Noetic (ROS) with Gazebo11 installed.

It is also recommended to learn and install TMUX for the scripts to run on mulitple screens.

### Folder Structure
```
├──Working_Directory
│   ├── requirements.txt
│   ├── README.md
│   ├── gazebo_files
│   │   ├── models
│   │   ├── cafe.world
│   │   └── outdoor.world
│   ├── visualnav-transformer
│   │   ├── deployment
│   │   ├── diffusion_policy
│   │   ├── train
│   │   └──  Original_README.md
└── └──MobileRobotics
    	└── catkin_ws
            └── src
                └── rtab_dumpster
```
### Create Conda Environment

```
cd /Working_Directory
conda create -n vint_deployment
conda activate vint_deployment
pip install -r requirements.txt
```
### Create ROS1 Workspace
Create workspace named as `catkin_ws` using the rtab_dumpster package in `/MobileRobotics/catkin_ws/src`


### Add Gazebo Dependencies
Copy the `models` folder and `.world` files found in `/gazebo_files` and place them in your system's `/home/.gazebo` folder. This is to allow us to retrieve the environments used in the Gazebo simulation.

---

### Manual Operation

#### Start up Simulation
Make sure to run this script inside the deployment/src/ directory.
```
sh run_project.sh <launch_file_name> 

 ## cafe, office, outdoor, city, circuit

sh run_project.sh city
```


#### Get Rosbag images
```
cd ../topomaps/bags
```
In the first plane:
```
rosbag record /camera/rgb/image_raw -O  <namebag_filename>

rosbag record /camera/rgb/image_raw /odom -O city 
```

When the you have finsihed your navigation, kill the ros bag recording immediately. This rosbag records image as well as /odom.

### Create topological map
Make sure to run this script inside the deployment/src/ directory.
Use the same bag used in the previous line.

```
./create_topomap.sh <topomap_name> <bag_filename> <rosbag_playback_rate>

./create_topomap.sh city city.bag 1.5
``` 
This command opens up 4 windows:
1. `roscore`
2. `python create_topomap.py —dt 1 —dir <topomap_name>`: This command creates a directory in `/visualnav-transformer/deployment/topomaps/images` and saves an image as a node in the map every second the bag is played.
3. `rosrun rviz rviz -d rviz/rosbag.rviz`: RViz is used to visualise the rosbag camera feed and Odom.
4. `rosbag play -r <rosbag_playback_rate> <bag_filename>`: This command plays the rosbag at which you can set the playback delay (default is 1.5 secs). The command does not run until you hit `Enter`, which you should only do once the python script gives its waiting message. Once you play the bag, move to the screen where the python script is running so you can kill it when the rosbag stops playing.

When the bag stops playing, kill the tmux session.


### Run navigation with model
Make sure to run this script inside the deployment/src/ directory.
```
./navigate.sh <launch_file_name> '--model <model_name> --dir <topomap_dir>' <output_odom>

./navigate.sh city '--model nomad --dir city' 'city_1'
```
To deploy one of the models from the published results, the reference paper authors are releasing model checkpoints that you can download from [this link](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing).


The `<model_name>` is the name of the model in the `vint_release/deployment/config/models.yaml` file. In this file, you specify these parameters of the model for each model (defaults used):
- `config_path` (str): path of the *.yaml file in `vint_release/train/config/` used to train the model
- `ckpt_path` (str): path of the *.pth file in `vint_release/deployment/model_weights/`

For this experiement, only the `nomad` model was used.

Make sure these configurations match what you used to train the model. The configurations for the models we provided the weights for are provided in yaml file for your reference.

The `<topomap_dir>` is the name of the directory in `visualnav-transformer/deployment/topomaps/images` that has the images corresponding to the nodes in the topological map. The images are ordered by name from 0 to N.

This command opens up 4 windows:

1. `roslaunch rtab_dumpster <launch_file_name>.launch`: This launch file opens the Gazebo environment.
2. `python navigate.py --model <model_name> -—dir <topomap_dir>`: This python script starts a node that reads in image observations from the `/camera/rgb/image_raw` topic, inputs the observations and the map into the model, and publishes actions to the `/waypoint` topic.
3. `rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.5 _turn:=0.5`: Allows us to teleoperate the robot’s base according to the speed and twist arguments.
4. `rosbag record -O <output_odom> /odom`: Records the odometry of the robot to the <output_odom> dir.

When the robot is finishing navigating, kill the `rosbag`.


### Compute Results
Make sure to run this script inside the deployment/src/ directory.
```
python calculate_performance.py --bag <bag_name> --count <number_of_bags>

python calculate_performance.py --bag city --count 5
```
This will print the performance based on the end-point of each bag with reference to the Goal point (denoted by the ending coordinates of the robot at /topomaps/bags).



