# carla_scenario

## Demo
![output](https://user-images.githubusercontent.com/38074802/195296981-5ac8c0a4-b225-43a9-8c57-ed863bca9c3f.gif)
```bash
./CarlaUE4.sh
python3 scenario_xml.py -s scenario/town5.xml
python3 manual_control.py --rolename ego_vehicle
```

## Overview
Python 2.x and 3.x scripts which support you to create scenarios in [CALRA](https://carla.org/), written in python, for human beings all over the world.
These scripts enable you to use [CARLA Python API](https://carla.readthedocs.io/en/latest/python_api/) via XML file.


## Features
* Control actors and trafficlight when the ego_vehicle steps on triggers.
* Script read XML file and run CARLA Python API (scenario_xml.py)
  Supports almost all of functions. (spawn, move, kill, pose, traffic light). You can customise it by CARLA Python API.
* Visualize the created scenario (check_scenario.py)
  You can check scenario by projecting spawn points, actor trajectory, kill points.
* Supports ROS msg [derived_object_msgs/Object.msg](http://docs.ros.org/melodic/api/derived_object_msgs/html/index-msg.html) (carla_ros_bridge.py).
  The [carla-ros-bridge]() doesn't support obstacles which added to the CARLA world after launching the carla-ros-bridge. So you need to spawn all of the actors at first. It's not good for long and sophisticated scenarios because it consumes the processing power a lot. These scripts support spawn and kill actors and update them to the ROS message. It saves processing power, good for the long scenarios.
   And this script can repeat the scenario after the scenario finished.
   
## Requirement
* CARLA (0.9.6-0.9.8 are tested)
* ROS (option)

## Install
```bash
pip install -r requirements.txt
```
Prease confirm that you sourced .egg file provided by CALRA.

## Usage
### Create Scenario
Demo scenario is [here](https://github.com/kuriatsu/carla-scenario/blob/master/scenario/town5.xml)

#### 0. Get spawn positions
```bash
# simulator
./CarlaUE4.sh
# Control HERO and copy it's position to the clipboard for scenario configulation
python3 manual_control.py --filter vehicle*
```
* `--filter`: `walker.pedestrian*` or `vehicle*`

##### How to copy HERO position to the clipboard
* `L` Button: get actor's location to clipboard for xml file
* `T` Button: get actor's tralsform to clipboard for xml file

#### 1. Set trigger
The scenario is read from top to bottom. If the vehicle misses one trigger, the scenario will stop at the point. Please be careful when deciding the "thres" value
**The trigger 0 is considered as initial setting. The scenario of trigger 0 is launched when the scenario starts whether the vehicle has stepped on the trigger.**
```xml
<trigger id="enter_road1" thres="2.0">
  <location>138.851516724,-2.28138899803,0.00914668943733</location>
```
* id: Set as you like, understandable.
* thres: The vehicle is considered to have stepped on the trigger when the vehicle is within the range of the value(m) from the trigger position.
* location: trigger location on the map

#### 2. Spawn Actors
walker
```xml
        <spawn id="ai_walker_road1_0">
            <type>ai_walker</type>
            <probability>20</probability>
            <blueprint>random</blueprint>
            <transform>122.392601013,-8.89567947388,1.11982262135,0.0,-179.000015259,0.0</transform>
        </spawn>
```
  * id: Set as you like, understandable.
  * type: `ai_walker` is controlled by simulator, `walker` is controlled by this script (track the specified waypoints).
  * probability: value for derived_object_msgs::Object::classification_certainty
  * blueprint: select from [HERE](https://carla.readthedocs.io/en/latest/bp_library/) or set random.
  * transform: spawn position
  
vehicles
```xml
        <spawn id="vehicle_road2_0">
            <type>vehicle</type>
            <probability>80</probability>
            <blueprint>random</blueprint>
            <transform>-55.2978057861,26.842716217,0.0971065685153,0.0965993627906,90.5140838623,6.39263889752e-05</transform>
        </spawn>
```
  * id: Set as you like, understandable.
  * type: `ai_vehicle` is controlled by simulator, `vehicle` is controlled by this script (track the specified waypoints using pure-pursuit).
  * probability: value for derived_object_msgs::Object::classification_certainty
  * blueprint: select from [HERE](https://carla.readthedocs.io/en/latest/bp_library/) or set random.
  * transform: spawn position
  
static objects
```xml
        <spawn id="static_road5_0">
            <type>static</type>
            <probability>80</probability>
            <collision_range>10.0</collision_range>
            <invincible>true</invincible>
            <blueprint>static.prop.plasticbag</blueprint>
            <transform>-12.8004541397,94.2045593262,0.951499938965,0.0,78.3999176025,0.0</transform>
        </spawn>
```
  * id: Set as you like, understandable.
  * type: `static`
  * probability: value for derived_object_msgs::Object::classification_certainty
  * collision_range: The object disappears when the vehicle is within the value(m) if `invinsible` is set to *true*. 
  * invinsible: If *false*, the object disappears. If *true*, the object becomes invinsible.
  * blueprint: select from [HERE](https://carla.readthedocs.io/en/latest/bp_library/) or set random.
  * transform: spawn position
  
#### 3. Control Actors
* Control ai actor
```xml
        <move id="ai_walker_road1_0">
            <waypoint speed="3.0"/>
        </move>
```
  * waypoint: specify only speed (km/h)
  
* Control actor
```xml
       <move id="vehicle_road2_0">
            <waypoint speed="20.0">-57.9040222168,49.9933433533,1.1593503952</waypoint>
            <waypoint speed="30.0">-40.0210914612,50.7340812683,1.1593503952</waypoint>
        </move>
```
* waypoint: the goal point of actors (transform of point)

#### 4. Kill Actor
```xml
        <kill id="ai_walker_road1_0"/>
        <kill id="vehicle_road2_0"/>
```

#### 5. Control trafficlight
When the vehicle steps the trigger, the light turns to the color
```xml
        <trafficlight>
            <location>-38.7292823792,-8.71572208405,1.11481559277</location>
            <state>red</state>
            <time>30.0</time>
        </trafficlight>
        <trafficlight>
            <location>-60.142124176,-9.56110668182,1.12300264835</location>
            <state>green</state>
            <time>30.0</time>
        </trafficlight>
        <trafficlight>
            <location>-59.4050865173,11.8945960999,1.14123988152</location>
            <state>red</state>
            <time>30.0</time>
        </trafficlight>
        <trafficlight>
            <location>-39.0418167114,11.9720258713,1.15340161324</location>
            <state>green</state>
            <time>30.0</time>
        </trafficlight>
```
location: the traffic light pole location (the script finds pole within 3m from specified position)
state: red, green, yellow
time: Time to keep the color

### Check Scenario
1.  Run simulator with server window (to get bird eye view), then change map `python carla-0.9.8/Python/util/config.py -m Town05`
1. Run check scenario
```bash
python check_scenario.py -s /path/to/scenario.xml
```
1. You can check the scenario. (Consistency check function will be supported...)

### Run Scenario
1. Run simulator and change map.

1. spawn ego vehicle with rolename=ego_vehicle
```bash
python manual_control.py --rolename ego_vehicle
```

1. Run scenario
```bash
python scenario.xml -s /path/to/scenario.xml
```

### Run Scenario with ROS
1. Run simulator and change map.

1. spawn ego vehicle with rolename=ego_vehicle
```bash
python manual_control.py --rolename ego_vehicle
```
1. roscore

1. run scenario with ros
```bash
python carla_ros_bridge.py -s /path/to/scenario.xml
```
The object msg will be published
