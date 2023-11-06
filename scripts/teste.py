

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

- simulation:
      layout: tiled
      panes:
        - sleep 10; roslaunch rtabmap_ros rtabmap.launch \
                    subscribe_rgbd:=true \
                    compressed:=true \
                    rtabmap_args:="--delete_db_on_start" \
                    rgbd_sync:=true \
                    approx_rgbd_sync:=true \
                    rgb_topic:=/airsim_node/Hydrone/Stereo_Cam/Scene \
                    camera_info_topic:=/airsim_node/Hydrone/Stereo_Cam/Scene/camera_info \
                    rtabmapviz:=false