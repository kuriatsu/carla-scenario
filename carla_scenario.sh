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
    # 1
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_left_pose_back.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_6_left_cross.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_3_right_cross.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_4_right_pose.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_left_cross.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_5_right_pose.xml"
    # 2
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_3_right_pose.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_4_left_cross.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_left_cross_forward.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_5_right_cross.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_left_pose.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_6_left_pose.xml"
    # 3
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_right_cross.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_4_left_pose.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_3_left_cross.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_5_left_pose.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_left_pose_forward.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_6_right_cross.xml"
    # 4
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_3_left_pose.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_4_right_cross.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_right_pose.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_6_right_pose.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_1_0.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_2_left_cross_back.xml"
    "/home/kuriatsu/Source/carla_scenario/scenario/town1_5_left_cross.xml"
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
