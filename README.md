# MMP_TrajOpt

Installation instructions 

Create a new directory 
```
mkdir catkin_ws
cd catkin_ws
```
Build the directory
```
catkin build
```

Git clone this repository 
```
git init . 
git remote add origin https://github.com/MarionLepert/MMP_TrajOpt
git pull origin master 
```

Install the dependencies 
```
rosdep install --from-paths src --ignore-src
```

Launch instructions 

Build directory
```
catkin build
```

Source setup.bash in each terminal window you open (unless already done in .bashrc)
``` 
source devel/setup.bash
```

Launch the move_it launch file for the kinova arm in one terminal window 
``` 
roslaunch kinova_planner move_test.launch
```

In a second terminal window, launch the move_test launch file 
```
roslaunch kinova_planner move_test.launch
``` 
