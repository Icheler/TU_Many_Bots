#!/bin/bash
echo "-------------------------------------------------------------"
echo "TU Many Bots configuration"
echo "-------------------------------------------------------------"
echo "tmb_ROBOT_ENV: [simple_corridor, maze, maze_simple, maze_simple_2, maze_clutter, maze_clutter_limited]"
echo "tmb_start_both: [True, False]"
echo "tmb_with_computer_vision: [True, False]"
echo "-------------------------------------------------------------"
if [ -z ${tmb_ROBOT_ENV+x} ]
then
  world=maze_clutter_limited
else
  world=$tmb_ROBOT_ENV
fi

if [[ $tmb_with_computer_vision == False ]]
then
  computer_vision_enabled=$tmb_with_computer_vision
else
  computer_vision_enabled=True
fi

if [[ $tmb_with_predicting_yaw == False ]]
then
  with_predicting_yaw=$tmb_with_predicting_yaw
else
  with_predicting_yaw=True
fi

if [[ $tmb_start_both == False ]]
then
  start_both=$tmb_start_both
else
  start_both=True
fi

export tmb_ROBOT_ENV=$world
export tmb_ROBOT=rto-1
export tmb_ROBOT_BLIND=rto-blind
export tmb_start_both=$start_both
export tmb_with_predicting_yaw=$with_predicting_yaw
export tmb_with_computer_vision=$computer_vision_enabled


echo "selected world is: $world"
echo "starting both robots: $start_both"
echo "with predicting yaw: $with_predicting_yaw"
echo "tmb_with_computer_vision": $computer_vision_enabled
echo "-------------------------------------------------------------"

if [ $world="simple_corridor" ]
then
  export tmb_start_robot1_x="0.0"
  export tmb_start_robot1_y="0.0"
  export tmb_start_robot1_z="0.0"
  export tmb_start_robot1_yaw="0"

  export tmb_start_robot2_x="1.0"
  export tmb_start_robot2_y="0.0"
  export tmb_start_robot2_z="0.0"
  export tmb_start_robot2_yaw="0"

  export tmb_start_robot_blind_x="-7.0"
  export tmb_start_robot_blind_y="0.0"

  export tmb_start_robot_blind_z="0.0"
  export tmb_start_robot_blind_yaw="5.7"

  export tmb_start_goal_x="0.60339"
  export tmb_start_goal_y="6.06"
  export tmb_start_goal_yaw="0"

  export tmb_start_robot_1_map_transform_x="0"
  export tmb_start_robot_1_map_transform_y="0"
  export tmb_start_robot_1_map_transform_z="0"

  export tmb_start_robot_2_map_transform_x="0"
  export tmb_start_robot_2_map_transform_y="0"
  export tmb_start_robot_2_map_transform_z="0"
fi
if [ $world = "maze" ]
then
  export tmb_start_robot1_x="4.3"
  export tmb_start_robot1_y="-6"
  export tmb_start_robot1_z="0.0"
  export tmb_start_robot1_yaw="3.0"

  export tmb_start_robot2_x="21.3"
  export tmb_start_robot2_y="8.0"
  export tmb_start_robot2_z="0.0"
  export tmb_start_robot2_yaw="-1.6"

  export tmb_start_robot_blind_x="15.0"
  export tmb_start_robot_blind_y="-9.0"
  export tmb_start_robot_blind_z="0.0"
  export tmb_start_robot_blind_yaw="0.0"

  export tmb_start_goal_x="0"
  export tmb_start_goal_y="0"
  export tmb_start_goal_yaw="0"
fi
if [ $world = "maze_simple" ]
then
  export tmb_start_robot1_x="3.7"
  export tmb_start_robot1_y="-8.4"
  export tmb_start_robot1_z="0.0"
  export tmb_start_robot1_yaw="0.0"

  export tmb_start_robot2_x="9.5"
  export tmb_start_robot2_y="8.5"
  export tmb_start_robot2_z="0.0"
  export tmb_start_robot2_yaw="-1.6"

  export tmb_start_robot_blind_x="-9.0"
  export tmb_start_robot_blind_y="-8.0"
  export tmb_start_robot_blind_z="0.0"
  export tmb_start_robot_blind_yaw="0.0"

  export tmb_start_goal_x="-4.60322"
  export tmb_start_goal_y="6.47698"
  export tmb_start_goal_yaw="0"
fi
if [ $world = "maze_simple_2" ]
then
  export tmb_start_robot1_x="1.0"
  export tmb_start_robot1_y="0.0"
  export tmb_start_robot1_z="0.0"
  export tmb_start_robot1_yaw="0.0"

  export tmb_start_robot2_x="-1.0"
  export tmb_start_robot2_y="0.0"
  export tmb_start_robot2_z="0.0"
  export tmb_start_robot2_yaw="0.0"

  export tmb_start_robot_blind_x="-5.0"
  export tmb_start_robot_blind_y="8.0"
  export tmb_start_robot_blind_z="0.0"
  export tmb_start_robot_blind_yaw="0.0"

  export tmb_start_goal_x="4.26245"
  export tmb_start_goal_y="-9.112855"
  export tmb_start_goal_yaw="0"
fi
if [ $world = "maze_clutter" ]
then
  export tmb_start_robot1_x="1.0"
  export tmb_start_robot1_y="0.0"
  export tmb_start_robot1_z="0.0"
  export tmb_start_robot1_yaw="0.0"

  export tmb_start_robot2_x="-1.0"
  export tmb_start_robot2_y="0.0"
  export tmb_start_robot2_z="0.0"
  export tmb_start_robot2_yaw="0.0"

  export tmb_start_robot_blind_x="-5.0"
  export tmb_start_robot_blind_y="8.0"
  export tmb_start_robot_blind_z="0.0"
  export tmb_start_robot_blind_yaw="0.0"

  export tmb_start_goal_x="4.04245"
  export tmb_start_goal_y="-9.42855"
  export tmb_start_goal_yaw="0"

  export tmb_start_robot_1_map_transform_x="0"
  export tmb_start_robot_1_map_transform_y="0"
  export tmb_start_robot_1_map_transform_z="0"

  export tmb_start_robot_2_map_transform_x="0"
  export tmb_start_robot_2_map_transform_y="0"
  export tmb_start_robot_2_map_transform_z="0"
fi
if [ $world = "maze_clutter_limited" ]
then
  export tmb_start_robot1_x="1.0"
  export tmb_start_robot1_y="0.0"
  export tmb_start_robot1_z="0.0"
  export tmb_start_robot1_yaw="0.0"

  export tmb_start_robot2_x="-1.0"
  export tmb_start_robot2_y="0.0"
  export tmb_start_robot2_z="0.0"
  export tmb_start_robot2_yaw="0.0"

  export tmb_start_robot_blind_x="-7.5"
  export tmb_start_robot_blind_y="-4.0"
  export tmb_start_robot_blind_z="0.0"
  export tmb_start_robot_blind_yaw="0.0"

  export tmb_start_goal_x="2"
  export tmb_start_goal_y="2.5"
  export tmb_start_goal_yaw="0"

  export tmb_start_robot_1_map_transform_x="0"
  export tmb_start_robot_1_map_transform_y="0"
  export tmb_start_robot_1_map_transform_z="0"

  export tmb_start_robot_2_map_transform_x="0"
  export tmb_start_robot_2_map_transform_y="0"
  export tmb_start_robot_2_map_transform_z="0"
fi
