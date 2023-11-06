import rospy

from std_msgs.msg import String

from airsim_ros_pkgs.msg import Image, VelCmd

from utils import image_transport

class Uav:
    

    def __init__(self, vehicle_name : str) -> None:
        rospy.Subscriber("/airsim_node/"+vehicle_name+"/Stereo_Cam/Scene", \
                         Image, self.__callback_rgb)
        
        rospy.Subscriber("/airsim_node/"+vehicle_name+"/Stereo_Cam/DepthPerspective", \
                         Image, self.__callback_depth)
        
        self.__vel_pub = rospy.Publisher("/airsim_node/"+vehicle_name+"/vel_cmd_world_frame", VelCmd, queue_size=1)
    
        self.__pub_info = rospy.Publisher("uav_info", \
                                          String, queue_size=10)
        
        self.__vehicle_name = vehicle_name
        self.__rgb = None
        self.__depth = None

        
    def callback_image(func):
        def callback(self, *args, **kwargs):
            data, img_type = func(self, *args, **kwargs)
            if data:
                cv_rgb = image_transport(data)
                self.__setattr__("__"+img_type, cv_rgb)
            else:
                info = f"Error in {img_type} cam!"
                self.__pub_info.publish(info)

        return callback
    
    @callback_image
    def __callback_rgb(self, data):
        return data, "rgb"
    
    @callback_image
    def __callback_depth(self, data):
        return data, "depth"
    
    def __str__(self) -> str:
        return self.__vehicle_name
    

