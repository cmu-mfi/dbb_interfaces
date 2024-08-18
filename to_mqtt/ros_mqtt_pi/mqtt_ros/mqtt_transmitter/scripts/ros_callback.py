import rospy
from geometry_msgs.msg import WrenchStamped
from industrial_msgs.msg import RobotStatus
from mqtt_spb_wrapper import MqttSpbEntityDevice
from sensor_msgs.msg import JointState


def wrenchstamped_to_bytearray(ros_data, callback_args):
    device = callback_args[0]
    prefix = callback_args[1]
    if isinstance(ros_data, WrenchStamped):
        
        prefix = 'DATA/'+prefix
        data = ros_data
        device.data.set_value(prefix+"/header.seq", data.header.seq)
        device.data.set_value(prefix+"/header.stamp.secs", data.header.stamp.secs)
        device.data.set_value(prefix+"/header.stamp.nsecs", data.header.stamp.nsecs)
        device.data.set_value(prefix+"/header.frame_id", data.header.frame_id)        
        device.data.set_value(prefix+"/wrench.force.x", data.wrench.force.x)
        device.data.set_value(prefix+"/wrench.force.y", data.wrench.force.y)
        device.data.set_value(prefix+"/wrench.force.z", data.wrench.force.z)
        device.data.set_value(prefix+"/wrench.torque.x", data.wrench.torque.x)
        device.data.set_value(prefix+"/wrench.torque.y", data.wrench.torque.y)
        device.data.set_value(prefix+"/wrench.torque.z", data.wrench.torque.z)
        
        device.publish_data()
        
    else:
        rospy.logerr(f"Data is not of type WrenchStamped, it is {type(ros_data)}")

def robotstatus_to_bytearray(ros_data, callback_args):
    device = callback_args[0]
    prefix = callback_args[1]
    if isinstance(ros_data, RobotStatus):

        prefix = 'DATA/'+prefix
        data = ros_data
        device.data.set_value(prefix+"/header.seq", data.header.seq)
        device.data.set_value(prefix+"/header.stamp.secs", data.header.stamp.secs)
        device.data.set_value(prefix+"/header.stamp.nsecs", data.header.stamp.nsecs)
        device.data.set_value(prefix+"/header.frame_id", data.header.frame_id)   
        device.data.set_value(prefix+"/mode.val", data.mode.val)
        device.data.set_value(prefix+"/e_stopped.val", data.e_stopped.val)
        device.data.set_value(prefix+"/drives_powered.val",data.drives_powered.val)
        device.data.set_value(prefix+"/motion_possible.val",data.motion_possible.val)
        device.data.set_value(prefix+"/in_motion.val", data.in_motion.val)
        device.data.set_value(prefix+"/in_error.val", data.in_error.val)
        device.data.set_value(prefix+"/error_code", data.error_code)
        
        device.publish_data()
    else:
        rospy.logerr(f"Data is not of type RobotStatus, it is of type {type(ros_data)}")

def jointstates_to_bytearray(ros_data, callback_args):
    device = callback_args[0]
    prefix = callback_args[1]
    if isinstance(ros_data, JointState):

        prefix = 'DATA/'+prefix
        data = ros_data
        device.data.set_value(prefix+"/header.seq", data.header.seq)
        device.data.set_value(prefix+"/header.stamp.secs", data.header.stamp.secs)
        device.data.set_value(prefix+"/header.stamp.nsecs", data.header.stamp.nsecs)
        device.data.set_value(prefix+"/header.frame_id", data.header.frame_id)
        
        for i in range(1, len(data.name) + 1):
            try:
                device.data.set_value(f"{prefix}/position/joint_{i}", data.position[i-1])
            except:
                pass
            try:
                device.data.set_value(f"{prefix}/velocity/joint_{i}", data.velocity[i-1])
            except:
                pass

        device.publish_data()
    else:
        rospy.logerr(f"Data is not of type JointState, it is of type {type(ros_data)}")