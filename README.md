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
roslaunch gen3_move_it_config demo.launch sys_mode:=mmp
```

In a second terminal window, launch the move_test launch file 
```
roslaunch kinova_planner move_test.launch args:="0 mmp"
``` 

Use the following arg value to use the entire mobile manipulation platform 
```
sys_mode:=mmp
args:="0 mmp"
```

or use the following arg value to use only the kinova arm 
```
sys_mode:=gen3
args:="0 gen3"
```



