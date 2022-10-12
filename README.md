# opensw_ros
Open ROS driver for communicating with SlamTec Slamware devices such as the
[M2M1 Mapper](https://www.slamtec.com/en/Lidar/Mapper) over ethernet.  Other
SlamTec devices based on SlamWare may also be supported.  For standalone lidar
devices, such as the RPLidar S1, the 
[rplidar_ros](https://github.com/Slamtec/rplidar_ros) package should be used,
which is based on a different interface and protocol.

# Nodes

## IMU Driver

The `imu_driver` node publishes the IMU data from a SlamWare device.

### Subscribed topics

### Published topics

| Topic  | Type | Description | 
|-----|----|----|
| **imu** | `sensor_msgs/Imu` | IMU sensor data |

### Parameters

| Name  | Type | Description | Default |
|----|----|----|----|
| **host**| `string` | Host to connect to | "192.168.11.11" |
| **frame_id** | `string` | Frame to use for the published messages | "imu" |
| **port**| `int` | TCP port to connect to | `1445` |
| **rate**| `double` | Publish rate | `50.0` |
| **use_raw_data**| `bool` | Use raw data fields | `true` |

## Laser Driver

The `laser_driver` node publishes the laser scan data from a SlamWare device.

### Subscribed topics

### Published topics

| Topic  | Type | Description | 
|-----|----|----|
| **scan** | `sensor_msgs/Scan` | Laser scan sensor data |

### Parameters

| Name  | Type | Description | Default |
|----|----|----|----|
| **host**| `string` | Host to connect to | "192.168.11.11" |
| **frame_id** | `string` | Frame to use for the published messages | "laser" |
| **max_range** | `double` | Max sensor range in meters | `40.0` |
| **min_range** | `double` | Min sensor range in meters | `0.0` |
| **port**| `int` | TCP port to connect to | `1445` |
| **rate**| `double` | Publish rate | `20.0` |
