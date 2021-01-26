# Panda demo package

## Installation of other packages needed
Install https://github.com/erdalpekel/panda_simulation and all other packages neededes

## Run pick and place with Gazebo simulation.
This simulation works "Gazebo graso fix plugin". This plugin attach the object when is near the gripper
avoiding some physics error in the gazebo enviroment.
Installation insctructions can be found here: https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation

In adition, you will need to modify franka_ros/franka_description/robots/panda_arm_hand.urdf.xacro.
Add before the ```</robot> ```
```
<!--Gazebo model plugin: grasp fix-->
  <gazebo>
    <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
      <arm>
        <arm_name>panda_arm</arm_name>
        <palm_link> panda_link7 </palm_link>
        <gripper_link> panda_leftfinger </gripper_link>
        <gripper_link> panda_rightfinger </gripper_link>
      </arm>
      <forces_angle_tolerance>100</forces_angle_tolerance>
      <update_rate>25</update_rate>
      <grip_count_threshold>1</grip_count_threshold>
      <max_grip_count>4</max_grip_count>
      <release_tolerance>0.0001</release_tolerance>
      <disable_collisions_on_attach>false</disable_collisions_on_attach>
      <contact_topic>__default_topic__</contact_topic>
    </plugin>
  </gazebo>
```

### Lunching simulation
1. in a terminal: (this will launch panda+gazebo)
	roslaunch panda_demo gazebo_pick_and_place.launch
2. in another terminal
	rosrun panda_demo gazebo_pick_and_place_demo.py
3. add box marker: 
	on rviz click add>by topic> /boxpose> Marker.
	Click on Save Config to lunch the display of this marker every time your lunch Rviz




