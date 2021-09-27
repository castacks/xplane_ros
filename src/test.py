from utils import read_traffic_file, to_local_runway_frame



def read_traffic():

    path ='/home/jay/xplane_ros_ws/src/xplane_ros/utils/304.txt'

    read_traffic_file(path)

if __name__ == '__main__':

    read_traffic()
