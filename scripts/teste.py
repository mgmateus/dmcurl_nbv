

class T:
    def __init__(self) -> None:
        self.a = ["oi"]

    
    def __teste(fun):
        def c(self, *args, **kwargs):
            data, tp = fun(self,*args, **kwargs)
            self.__getattribute__(tp).append(data)
        return c
    
    @__teste
    def g(self, data):
        return data, "a"
    
if __name__ == "__main__":
    t = T()
    print(t.g("oi"))
    print(t.a)
    
- rtabmap:
      layout: tiled
      panes:
        - sleep 10; roslaunch rtabmap_launch rtabmap.launch rtabmap_args:="--delete_db_on_start" rgb_topic:=/airsim_node/Hydrone/Stereo_Cam/Scene depth_topic:=/airsim_node/Hydrone/Stereo_Cam/DepthPerspective camera_info_topic:=/airsim_node/Hydrone/Stereo_Cam/Scene/camera_info odom_topic:=/airsim_node/Hydrone/odom_local_ned imu_topic:=/airsim_node/Hydrone/imu/Imu visual_odometry:=false frame_id:=Stereo_Cam_optical approx_sync:=false rgbd_sync:=true queue_size:=1000
