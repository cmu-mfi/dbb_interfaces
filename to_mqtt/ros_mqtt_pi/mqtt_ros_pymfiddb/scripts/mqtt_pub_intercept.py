# https://gist.github.com/wkentaro/2cd56593107c158e2e02

'''
1. init RosDataObject
2. init PushMqttStream using RosDataObject
3. create a class which has PushMqttStream as a member, 
    and a AnyMsg callback function
4. Use async spinner with many threads to run the callback function

'''

import yaml
# from mfi_ddb.data_objects import RosDataObject
from mfi_ddb import PushStreamToMqttSpb, RosCallback, RosDataObject

if __name__ == "__main__":
    
    ros_config_file = "/home/mfi/repos/dbb_interfaces/to_mqtt/ros_mqtt_pi/mqtt_ros_pymfiddb/config/ros_config.yaml"
    mqtt_config_file = "/home/mfi/repos/dbb_interfaces/to_mqtt/ros_mqtt_pi/mqtt_ros_pymfiddb/config/mqtt_config.yaml"
    
    with open(ros_config_file, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader) 
    
    data_obj = RosDataObject(config, enable_topic_polling=False)
    data_obj.get_data()
    mqtt_stream = PushStreamToMqttSpb(mqtt_config_file, data_obj)
    ros_callback = RosCallback(mqtt_stream)