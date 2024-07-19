# Digital Backbone Interfaces

The digital backbone (DBB) architecture is inspired by the unified namespace concept. It uses MQTT broker to stream data from source to respective databases.

### from_mqtt

Following interfaces subscribes to topics from the broker to store in their respective databases

| Interface | Storage Location/Database    |
| --------- | --------------------------   |
| [mqtt_cfs](https://github.com/cmu-mfi/dbb_interfaces/tree/main/from_mqtt/mqtt_cfs)  | Cloud File Storage (CFS)     |
| [mqtt_pi](https://github.com/cmu-mfi/dbb_interfaces/tree/main/from_mqtt/mqtt_pi)   | Aveva OSIsoft PI Historian   |
| [mqtt_sql](https://github.com/cmu-mfi/dbb_interfaces/tree/main/from_mqtt/mqtt_sql)  | SQL                      |

### to_mqtt

Following interfaces publishes topics to the broker by listening to their respective sources

| Interface     | Source    |
| ------------- | --------------------------        |
| [lfs_mqtt](https://github.com/cmu-mfi/dbb_interfaces/tree/main/to_mqtt/lfs_mqtt)      | Local File Storage                |
| [ros_mqtt_lfs](https://github.com/cmu-mfi/dbb_interfaces/tree/main/to_mqtt/ros_mqtt_lfs)  | Non-Textual Data from ROS Topics (like Images/Audio/Video)  |
| [ros_mqtt_pi](https://github.com/cmu-mfi/dbb_interfaces/tree/main/to_mqtt/ros_mqtt_pi)   | Time Series Textual ROS Topics [mqtt spb] |
