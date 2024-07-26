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
| [ros_mqtt_cfs](https://github.com/cmu-mfi/dbb_interfaces/tree/main/to_mqtt/ros_mqtt_cfs)  | Non-Textual Data from ROS Topics (like Images/Audio/Video)  |
| [ros_mqtt_pi](https://github.com/cmu-mfi/dbb_interfaces/tree/main/to_mqtt/ros_mqtt_pi)   | Time Series Textual ROS Topics [mqtt spb] |


## Install and Usage

Follow the steps for each interface on your system. Replace `from_mqtt/mqtt_cfs` with your desired interface. 
Each interface may have their own config files. Refer to respective README for details before use.

**Windows Command Prompt**

Installation
```
$ cd from_mqtt/mqtt_cfs
$ python -m venv venv
$ venv\Scripts\activate.bat
$ pip install -r requirements.txt
```

Usage
```
$ cd from_mqtt/mqtt_cfs
$ venv\Scripts\activate.bat
$ python mqtt_cfs.py
```
*Note: For Windows you can alternatively use executables in [executable template](https://github.com/cmu-mfi/dbb_interfaces/tree/main/executable%20template) for install and use*

**Linux Terminal**

Installation
```
$ cd from_mqtt/mqtt_cfs
$ python3 -m venv .venv
$ source .venv/bin/activate
$ python3 -m pip install -r requirements.txt
```

Usage
```
$ cd from_mqtt/mqtt_cfs
$ source venv\Scripts\activate
$ python mqtt_cfs.py
```