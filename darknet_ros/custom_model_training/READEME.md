# Training the model for TU many bots.
***

#### 1) Setting up darknet yolo
 Training the model for this project is to be done outside of the project workspace.

```
- cd Documents
- git clone https://github.com/pjreddie/darknet
- cd darknet
```

To achieve a reasonable speed, we need the GPU.
Open the Makefile found within the darknet directory; `sudo nano Makefile`
and change `GPU=1` and `OPENCV=1`
```
- make
```

###### Troubleshooting
- Ensure cuda is installed by running `nvidia-smi`
- Ensure OpenCV install is OK by running `pkg-config --cflags opencv`

***
#### 2) Training the model.
Copy the directory **custom_test_data** into the darknet folder.
This consists of the following:
- **yolo-obj.cfg** (the full definition of the neural network used)
- **darknet19_448.conv.23** (the initial weights)
- **obj.data** (defines the amount of classes and where to find training data)
- **data/** (the data folder)
- **data/img** (the images + annotations used for training)
- **data/obj.names** (the names of each of the classes)
- **data/train.txt** (the list of photos used for training)


If the above worked, you can proceed to training the network.

```
sudo ../darknet detector train obj.data yolo-obj.cfg darknet19_448.conv.23
```

The computed weights will be saved in the /backup folder.

---
##### 3) Using the model

The model used by the project is defined by the config in
darknet_ros/darknet_ros/config/robotyolo.yaml.
Namely,

```
yolo_model:

  config_file:
    name: robotyolo.cfg
  weight_file:
    name: robotyolo.weights
  threshold:
    value: 0.3
  detection_classes:
    names:
      - robot
      - blind_robot
      - space_man
```

Go to catkin_ws/---/darknet_ros/yolo_network_config   
Where you will find the cfg/  and weights/ .
- Copy the yolo-obj.cfg file to the /cfg directory and rename it to robotyolo.cfg
- Copy the weights to the /weights directory and rename it to robotyolo.weights
