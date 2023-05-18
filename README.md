# xArm_CMU_CornInsert
xArm motion for the custom EE

# install 
Smach package is simple to install.
```
sudo apt-get install ros-noetic-smach*
```

When running ...
```
rosrun smach_viewer smach_viewer.py

```
and you get the error...(this is specific to Ubuntu 20.04 with ROS Noetic)
```
ModuleNotFoundError: No module named 'wxversion'
ModuleNotFoundError: No module named 'xdot'
```

Install dependencies for the smach-viewer
```
sudo apt install libcairo2-dev libxt-dev libgirepository1.0-dev
pip install pycairo PyGObject

```



# set up code
```
cd [write_your_path_to_directory]
git clone https://github.com/MarkLee634/xArm_CMU_CornInsert.git
```

# running
```
python main.py
```