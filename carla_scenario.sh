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
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_3.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_base.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_3.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_3.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_3_3.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_base.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_4.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_3_4.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_4.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_base.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_1.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_1.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_3_1.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_base.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_3_2.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_2.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_2.xml"
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
