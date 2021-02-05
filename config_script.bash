#!/bin/bash
echo "-------------------------------------------------------------"
echo "viable robots are: [rto-1]"
echo "viable worlds are: [simple_corridor, maze, maze_simple, maze_simple_2, maze_clutter]"
echo "-------------------------------------------------------------"
if [ -z ${robot_env+x} ]
then
  world=simple_corridor
else
  world=$robot_env
fi

export tmb_ROBOT_ENV=$world
export tmb_ROBOT=rto-1
export tmb_ROBOT_BLIND=rto-blind

echo "selected world is: $world"
echo "selected robot is: rto-1"
echo "selected blind robot is: rto-blind"
echo "-------------------------------------------------------------"

if [ $world="simple_corridor" ]
then
  export tmb_start_robot1_x="0.0"
  export tmb_start_robot1_y="0.0"
  export tmb_start_robot1_z="0.0"
  export tmb_start_robot1_yaw="0.0"

  export tmb_start_robot2_x="1.0"
  export tmb_start_robot2_y="0.0"
  export tmb_start_robot2_z="0.0"
  export tmb_start_robot2_yaw="0"

  export tmb_start_robot_blind_x="4.0"
  export tmb_start_robot_blind_y="-3.5"
  export tmb_start_robot_blind_z="0.0"
  export tmb_start_robot_blind_yaw="5.7"
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
fi
