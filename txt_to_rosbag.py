import argparse
import rosbag
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np
from pathlib2 import Path
import glob
import rospy
from sensor_msgs.msg import PointCloud2

import pcl_helper


if __name__=='__main__':
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--data_dir', type=str, default=None, help='directory path')
    args = parser.parse_args()

    rospy.init_node('txt_to_rosbag')
    pub = rospy.Publisher('/soslab_sl/pointcloud', PointCloud2, queue_size=10)

    folder_name = args.data_dir.split('/')[-1]
    save_path = args.data_dir +'/'+ folder_name + '.bag'
    

    write_rosbag = rosbag.Bag(save_path, 'w')


    # pcd_path = args.data_dir + '/pcd'   
    # pcd_list = sorted(glob.glob(pcd_path+'/*.pcd'))
    # r = rospy.Rate(10)  # 10hz
    # for pcd_file in pcd_list:
    #     print(pcd_file)
    #     pc_xyzrgb = pcl.load_XYZRGB(pcd_file)
    #     np_pc_xyzrgb = pc_xyzrgb.to_array()
    #     print(np_pc_xyzrgb)
    #     msg_pc = pcl_helper.pcl_to_ros(pc_xyzrgb)

    #     write_rosbag.write(topic='/soslab_sl/pointcloud', msg=msg_pc, t=rospy.Time.now())

    #     r.sleep()
    
    # write_rosbag.close()
    
    txt_path = args.data_dir + '/txt'
    list_txt = sorted(glob.glob(txt_path+'/*.txt'))
    print(list_txt)
    r = rospy.Rate(10)  # 10hz
    for txt_file in list_txt:
        print(txt_file)
        with open(txt_file, 'r') as f:
            lines = f.readlines()
            pc_list = []
            color_list = []

            pc_xyzrgb_list = []
            for line in lines:
                pc_xyzrgb = line.strip().split(',')
                
                pc_xyz = [float(pc_xyzrgb[0]), float(pc_xyzrgb[1]), float(pc_xyzrgb[2])]
                color = [int(pc_xyzrgb[3]), int(pc_xyzrgb[4]), int(pc_xyzrgb[5])]

                #pc_list.append(pc_xyz)
                #color_list.append(color)

                float_color = pcl_helper.rgb_to_float(color)
                pc_xyzrgb_list.append([pc_xyz[0], pc_xyz[1], pc_xyz[2], float_color])

            #np_pc_list = np.asarray(pc_list)
            #np_color = np.asarray(color_list)
            
            #msg_pc = pcl_helper.xyzrgb_array_to_pointcloud2(points=np_pc_list, colors=np_color, stamp=time, frame_id='map')

            XYZRGB_cloud = pcl.PointCloud_PointXYZRGB()
            XYZRGB_cloud.from_list(pc_xyzrgb_list)
            msg_pc = pcl_helper.pcl_to_ros(XYZRGB_cloud)

            time = rospy.Time.now()
            write_rosbag.write(topic='/soslab_sl/pointcloud', msg=msg_pc, t=time)

        r.sleep()
    
    write_rosbag.close()


