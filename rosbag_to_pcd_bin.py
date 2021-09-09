import argparse
import rosbag
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np
#from pathlib2 import Path
from pathlib import Path
import glob
import copy

from pcl_helper import float_to_rgb

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--bag_path', type=str, default=None, help='directory path')
    parser.add_argument('--lidar_topic', type=str, default=None, help='lidar topic name')
    parser.add_argument('--is_rgb', type=bool, default=False, help='is rg?')
    args = parser.parse_args()
    
    bag = rosbag.Bag(args.bag_path)
        
    bag_filename = (args.bag_path.split('/')[-1]).split('.')[0]
    print(bag_filename)

    save_path = args.bag_path.split('.')[0]
    Path(save_path).mkdir(parents=True, exist_ok=True)
    print(save_path)
    
    for lidar_topic, lidar_msg, lidar_t in bag.read_messages(topics=[args.lidar_topic]):
        
        test_list_pc = [] # test
        
        # lidar msg to numpy array(n,4)
        list_pc = []
        for data in pc2.read_points(lidar_msg, skip_nans=False):
            
            if args.is_rgb == True:
                rgb = float_to_rgb(data[3])
                xyzrgb = [data[0], data[1], data[2]]+rgb 
                list_pc.append( xyzrgb )
            else:
                list_pc.append([data[0], data[1], data[2], data[3]])
        
        np_pc = np.array(list_pc, dtype=np.float32)
        print(np_pc.shape)
        
        # save numpy array to bin or pcd
        filename = str(lidar_t)
        print(filename)
        
        np_pc_bin = copy.deepcopy(np_pc)
        bin_path_ = save_path+'/bin'
        Path(bin_path_).mkdir(parents=True, exist_ok=True)
        bin_path = bin_path_ + "/" + filename + ".bin"
        np_pc_bin.astype(np.float32).tofile(bin_path)
        
        np_pc_pcd = copy.deepcopy(np_pc)
        pcd_path_ = save_path+'/pcd'
        Path(pcd_path_).mkdir(parents=True, exist_ok=True)
        pcd_path = pcd_path_+"/"+filename+".pcd"
        
        if args.is_rgb == True:
            pc_rgb = pcl.PointCloud_PointXYZRGB()
            pc_rgb.from_list(test_list_pc)
            pcl.save(pc_rgb, pcd_path)
        else:
            pc_intensity = pcl.PointCloud_PointXYZI()
            pc_intensity.from_array(np_pc_pcd)
            pcl.save(pc_intensity, pcd_path)

    print('DONE')
            

