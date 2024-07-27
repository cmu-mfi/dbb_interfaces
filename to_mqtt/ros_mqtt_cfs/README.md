# ROS Topics to MQTT Broker for Non-Textual data

Publishes non-textual data (like audio, images, pointcloud) from ROS topics to the MQTT broker.

For installation and usage follow steps [here](https://github.com/cmu-mfi/dbb_interfaces?tab=readme-ov-file#install-and-usage)

## Config Files 

**config.yaml**

| Parameter | Details    |
| --------- | --------------------------   |
| `ros_websocket` | Websocket details to connect to ROS network <br> Expects ROS network to have a [websocket server](https://github.com/RobotWebTools/rosbridge_suite) |
| `ros_topics`    | List of ROS topics to be listening for data.<br> Required fields: name, type, mqtt_topic, mqtt_dict_keys <br> Recommended fields: fileprefix, fileext. <br> Other keys can be added as required by the respective topic handler. |

Each ros_topic "type" needs a handler class. Example: *AudioHandler* in audio_handler.py for ros_topic type *sounddevice_ros/AudioData*

**setup.yaml**

It contains details required to connect to the MQTT Broker.