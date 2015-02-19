# urdf_management

Package to alter the robot description at runtime.
Uses the robot_descritpion from the parameter server as initial description and publishes the altered robot descriptions as a string to the latched topic ```/dynamic_robot_description```.

## AlterUrdf

### Start
With a running roscore type in a new terminal:
  * ```rosrun urdf_management urdf_management_service.lisp```

### Usage 
The service takes 3 arguments ```action```, ```xml_elements_to_add``` and ```element_names_to_remove```.
  * ```action```: ```ADD``` to add the ```xml_elements_to_add``` to the robot description, ```REMOVE``` to remove the ```element_names_to_remove``` from the robot description
  * ```xml_elements_to_add```: urdf like xml description of the links and joints to add to the robot description. If a link or joint already exists in the current robot model the existing element will be overwritten by the new one.
  * ```element_names_to_remove```: list of names of links that should be removed from the robot description. The links cannot have any children or they have to also be in the list. The joint that connects the link with its parent will also be removed.

### Examples
Add a link and a joint:
```
rosservice call /alter_urdf 1 "<link name=\"new_link\"/> <joint name=\"new_joint\" type=\"fixed\"><parent link=\"root_link\"/><child link=\"new_link\"/></joint>" '[]'
```

Remove a link and the joint that connects it to the robot:
```
rosservice call /alter_urdf 2 "" '["new_link"]'
```

## SimpleAlterUrdf

### Usage
The SimpleAlterUrdf service takes the arguments ```action``` and ```parameter``` where ```action``` is the same as for the AlterUrdf service. ```parameter``` is the name of a parameter on the ros parameter service in the namespace ```urdf_management```. The parameter should contain a xml description of the links and joints you which to modify like ```xml_elements_to_add``` for the AlterUrdf service.

### Example
Add a link and a joint:
```
rosparam set urdf_management/new_link_and_joint "<link name=\"new_link\"/> <joint name=\"new_joint\" type=\"fixed\"><parent link=\"root_link\"/><child link=\"new_link\"/></joint>"
rosservice call /alter_urdf 1 new_link_and_joint
```

Remove the link and joint from the previous example:
```
rosservice call /alter_urdf 2 new_link_and_joint
```


