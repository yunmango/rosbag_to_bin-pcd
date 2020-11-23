import argparse
import rosbag
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np
from pathlib2 import Path
import glob
import rospy

import pcl_helper


if __name__=='__main__':
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--data_dir', type=str, default=None, help='directory path')
    args = parser.parse_args()

    rospy.init_node('txt_to_rosbag')

    folder_name = args.data_dir.split('/')[-1]
    save_path = args.data_dir +'/'+ folder_name + '.bag'

    write_rosbag = rosbag.Bag(save_path, 'w')

    list_txt = sorted(glob.glob(args.data_dir+'/*.txt'))

    r = rospy.Rate(10)  # 10hz
    #while not rospy.is_shutdown():
    for txt_file in list_txt:
        print(txt_file)
        with open(txt_file, 'r') as f:
            lines = f.readlines()
            pc_list = []
            for line in lines:
                pc_xyzrgb = line.strip().split(',')
                pc_xyz = [float(pc_xyzrgb[0]), float(pc_xyzrgb[1]), float(pc_xyzrgb[2])]
                color = [int(pc_xyzrgb[3]), int(pc_xyzrgb[4]), int(pc_xyzrgb[5])]
                float_color = pcl_helper.rgb_to_float(color)
                pc_list.append([pc_xyz[0], pc_xyz[1], pc_xyz[2], float_color])

            XYZRGB_cloud = pcl.PointCloud_PointXYZRGB()
            XYZRGB_cloud.from_list(pc_list)
            
            msg_pc = pcl_helper.pcl_to_ros(XYZRGB_cloud)

            write_rosbag.write(topic='/soslab_sl/pointcloud', msg=msg_pc, t=rospy.Time.now())
                

        r.sleep()
    
    write_rosbag.close()


