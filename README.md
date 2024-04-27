# Level_FRONT_LIFT_POS

## 1. For What?

<image src="figure/figure_1.jpg">

Estimate the height h and then find out the lift position to level the platform.

## 2. Topics to subscribe to and publish

### a. The topic to subscribe to

To get orientation of the platform, install the pxhawk 4.

From the pxhawk, get the topic called **/mavros/imu/data**.

To get to know the configuration of the leg information,

get the motor position.

The following topic includes motor positions.

**/actual**

### b. The topic to publish

The estimated height of obstacle: **/h_est**

The desired position of front lift: **/q_des**

Roll and pitch angle: **/rp_result** (deg)

Herein, theta and phi represent pitch and roll, respectively.

## 3. Setup

Navigate to launch directory.

Set offset value of lift motors.

Setup like trajectory generator. 

The values of **offset0** ~ **offset2** correspond to **offset_pos0** ~ **offset_pos2**.

## 4. How to launch the program.

Launch the height estimation node.
```
roslaunch height_estimator height_est.launch
```

How to install mavros.

```
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-mavros-msgs
```

```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
```

```
sudo bash ./install_geographiclib_datasets.sh
```


Launch the pix hawk4.
```
roslaunch mavros px4.launch
```
