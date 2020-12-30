#!/bin/bash

if [ $1 = "Town05" ]; then
    scenarios=(
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_static1.xml"
    # "/home/kuriatsu/Source/carla_scenario/scenario/town5_base.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_cross0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_stands0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_static0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_stands0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_cross0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_static0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_static0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_stands0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_cross0.xml"
    # "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_cross1.xml"
    # "/home/kuriatsu/Source/carla_scenario/scenario/town5_pedestrian_stands1.xml"
    # "/home/kuriatsu/Source/carla_scenario/scenario/town5_static1.xml"
    )
    python /home/kuriatsu/Source/carla_scenario/carla_ros_bridge.py -s "${scenarios[@]}"

elif [ $1 = "Town04" ]; then
    scenarios=(
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_base.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town4_intervene.xml"
    )
    python /home/kuriatsu/Source/carla_scenario/carla_ros_bridge.py -s "${scenarios[@]}"
# 7: not for control
elif [ $1 = "Town01" ]; then
    scenarios=(
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_surprise_1.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_6.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_10.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_14.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_5.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_8.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_12.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_9.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_11.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_16.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_4.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_13.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_7.xml"
    # "/home/kuriatsu/Source/carla_scenario/scenario/town1_15.xml"
    # "/home/kuriatsu/Source/carla_scenario/scenario/town1_3.xml"
    )
    python /home/kuriatsu/Source/carla_scenario/carla_ros_bridge.py -s "${scenarios[@]}"

else
    echo "no town specified"
fi
# args=""
# for scenario in "${scenarios[@]}"; do
#     args+=""
#     args+="[ ${scenario} ]"
# done
