# xArm_CMU_CornInsert
xArm motion for the custom EE

# install 
Smach package is simple to install.
```
sudo apt-get install ros-noetic-smach*
```


# set up code
```
cd [write_your_path_to_directory]
git clone https://github.com/MarkLee634/xArm_CMU_CornInsert.git
```

# running
To run with the FSM that interacts with xArm 
```
python main.py
```

To run a simple open-loop sequence of xArm motions for corn insert using only the python SDK
```
python main_openloop.py
```
