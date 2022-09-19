# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Process the point-cloud and prepare it for object detection
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# general package imports
import cv2
import numpy as np
import torch
import matplotlib.pyplot as plt
# add project directory to python path to enable relative imports
import os
import sys
import zlib
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
from google.protobuf.json_format import MessageToDict
from PIL import Image

# waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2, label_pb2

# object detection tools and helper functions
import misc.objdet_tools as tools
import open3d as o3d

## !!!! should be added as confige !!!! ##
bev_width=608
bev_height=608
lim_x = [0, 50]
lim_y = [-25, 25]
lim_z = [-3, 3]
bev_width = 608
bev_height = 608
conf_thresh = 0.5
model = 'darknet'



# visualize lidar point-cloud
def show_pcl(pcl):
    ####### ID_S1_EX2 START #######     
    #######
    print("student task ID_S1_EX2")
    global scan
    scan=1
    # step 1 : initialize open3d with key callback and create window
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("pcl", width=800, height=800, left=50, top=50,visible=True)

    # step 2 : create instance of open3d point-cloud class
    pcd=o3d.geometry.PointCloud()
    # step 3 : set points in pcd instance by converting the point-cloud into 3d vectors (using open3d function Vector3dVector)
    pcd.points=o3d.utility.Vector3dVector(pcl[:,:3])

    # step 4 : for the first frame, add the pcd instance to visualization using add_geometry; for all other frames, use update_geometry instead
    def callback(vis):
        global scan
        print("key")
        scan=0
        return scan
    vis.register_key_callback(262,callback)
    # step 5 : visualize point cloud and keep window open until right-arrow is pressed (key-code 262)
    
    while scan==1:
        #print("reg")
        '''o3d.visualization.draw_geometries([pcd],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])'''
        vis.add_geometry(pcd)
        #print("geo")
        vis.poll_events()
        vis.update_renderer()
    
    #######
    ####### ID_S1_EX2 END #######     
       

# visualize range image
def show_range_image(frame, lidar_name):

    ####### ID_S1_EX1 START #######     
    #######
    #print("student task ID_S1_EX1")

    # step 1 : extract lidar data and range image for the roof-mounted lidar
    #lidar=[obj for obj in frame.lasers if obj.name==lidar_name[0]] 
    lidar=[]
    ri=[]

    msg=MessageToDict(frame)     #nice_idea
    print(msg.keys())

    #print(lidar_name) #=1
    #print(type(frame.lasers)) #<class 'google.protobuf.pyext._message.RepeatedCompositeContainer'>
    for a in frame.lasers:
        #print(a.name==lidar_name) #=1
        if a.name==lidar_name:
            lidar.append(a)
    #lidar  type :  <class 'list'>
    #lidar[0] type:<class 'simple_waymo_open_dataset_reader.dataset_pb2.Laser'>

    if len(lidar[0].ri_return1.range_image_compressed) > 0: 
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar[0].ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
        print("flag")
    print(np.shape(ri))
    
    # step 2 : extract the range and the intensity channel from the range image
    # step 3 : set values <0 to zero
    #range values ---> 
    ri[ri<0]=0.0
    ri_range = ri[:,:,0]
    ri_intensity=ri[:,:,1]
    
    
    # step 4 : map the range channel onto an 8-bit scale and make sure that the full range of values is appropriately considered
    ri_range = ri_range * 255 / (np.amax(ri_range) - np.amin(ri_range))
    img_range = ri_range.astype(np.uint8)
    #cv2.imshow("res", img_range)
    #cv2.waitKey(0)


    # step 5 : map the intensity channel onto an 8-bit scale and normalize with the difference between the 1- and 99-percentile to mitigate the influence of outliers
    ri_intensity = np.amax(ri_intensity)/2 * ri_intensity * 255 / (np.amax(ri_intensity) - np.amin(ri_intensity))
    img_intensity = ri_intensity.astype(np.uint8)

    #cv2.imshow("res", img_intensity)
    #cv2.waitKey(0)
    # step 6 : stack the range and intensity image vertically using np.vstack and convert the result to an unsigned 8-bit integer
    
    img_range_intensity=np.vstack((ri_range,ri_intensity)).astype(np.uint8)
    center=int(np.shape(img_range_intensity)[1]/2)
    rng=int(np.shape(img_range_intensity)[1]/4)
    img_range_intensity=img_range_intensity[:,center-rng:center+rng]
    print("shape of img range intensity", np.shape(img_range_intensity))
    #img_range_intensity = [] # remove after implementing all steps
    #######
    ####### ID_S1_EX1 END #######     
    
    return img_range_intensity


# create birds-eye view of lidar data
def bev_from_pcl(lidar_pcl, configs):
    vis=False
    # remove lidar points outside detection area and with too low reflectivity
    mask = np.where((lidar_pcl[:, 0] >= configs.lim_x[0]) & (lidar_pcl[:, 0] <= configs.lim_x[1]) &
                    (lidar_pcl[:, 1] >= configs.lim_y[0]) & (lidar_pcl[:, 1] <= configs.lim_y[1]) &
                    (lidar_pcl[:, 2] >= configs.lim_z[0]) & (lidar_pcl[:, 2] <= configs.lim_z[1]))
    lidar_pcl = lidar_pcl[mask]  # (N, 4)
    
    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]  
    #print(np.shape(lidar_pcl))  # (N, 4)

    # convert sensor coordinates to bev-map coordinates (center is bottom-middle)
    ####### ID_S2_EX1 START #######     
    #######
    print("student task ID_S2_EX1")

    ## step 1 :  compute bev-map discretization by dividing x-range by the bev-image height (see configs)
    discreteX= (configs.lim_x[1]-configs.lim_x[0])/configs.bev_height  # m/pixel


    ## step 2 : create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates    
    lidar_pcl_cpy=np.copy(lidar_pcl)
    lidar_pcl_cpy[:,0]=np.int_(np.floor(lidar_pcl_cpy[:,0]/discreteX))#+ (configs.bev_width-1)/2


    # step 3 : perform the same operation as in step 2 for the y-coordinates but make sure that no negative bev-coordinates occur
    #lidar_pcl_cpy[:,1]=((lidar_pcl_cpy[:,1])-2*np.amin(lidar_pcl_cpy[:,1]))/discreteX  #+ configs.bev_width/2)
    lidar_pcl_cpy[:,1]=np.abs(np.int_(np.floor(lidar_pcl_cpy[:,1]/discreteX) + (bev_width+1)/2))
    #print(np.amin(lidar_copy[:,1]))


    # step 4 : visualize point-cloud using the function show_pcl from a previous task
    #show_pcl(lidar_copy)

    lidar_pcl_cpy[lidar_pcl_cpy[:,3]>1.0, 3]=1.0
    ####### ID_S2_EX1 END #######     
    # Compute intensity layer of the BEV map
    ####### ID_S2_EX2 START #######     
    #######
    print("student task ID_S2_EX2")
    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    intensity_map=np.zeros((configs.bev_height, configs.bev_width))

    # step 2 : re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then -z (use numpy.lexsort)
    sort_idx=np.lexsort((-lidar_pcl_cpy[:,2], lidar_pcl_cpy[:,1], lidar_pcl_cpy[:,0]))
    sorted_lidar_copy=lidar_pcl_cpy[sort_idx]


    ## step 3 : extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    ##          also, store the number of points per x,y-cell in a variable named "counts" for use in the next task
    _, idx, counts=np.unique(sorted_lidar_copy[:,0:2], axis=0, return_index=True, return_counts=True)
    lidar_pcl_top=sorted_lidar_copy[idx]
    #print(np.shape(lidar_pcl_top), "here")
    #print(lim_z)
    #print(np.shape(map))


    ## step 4 : assign the intensity value of each unique entry in lidar_top_pcl to the intensity map 
    ##          make sure that the intensity is scaled in such a way that objects of interest (e.g. vehicles) are clearly visible    
    ##          also, make sure that the influence of outliers is mitigated by normalizing intensity on the difference between the max. and min. value within the point cloud
    pr=np.percentile(lidar_pcl_top[:,3], 50, axis=0)
    intensity_map[np.int_(lidar_pcl_top[:,0]), np.int_(lidar_pcl_top[:,1])] = lidar_pcl_top[:, 3] / (np.amax(lidar_pcl_top[:, 3])-np.amin(lidar_pcl_top[:, 3]))
    

    ## step 5 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background
    if vis==True:
        intensity_map=(intensity_map*256).astype(np.uint8)
        cv2.imshow("intensity_map", intensity_map)
        cv2.waitKey(0)

    '''pixel_plot = plt.figure()
    #pixel_plot.add_axes()
    plt.title("pixel_plot")
    pixel_plot = plt.imshow(
    intensity_map, cmap='twilight', interpolation='nearest')
    plt.colorbar(pixel_plot)
    plt.show()'''
    


    #######
    ####### ID_S2_EX2 END ####### 
    # Compute height layer of the BEV map
    ####### ID_S2_EX3 START #######     
    #######
    print("student task ID_S2_EX3")
    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    height_map=np.zeros((bev_height, bev_width))


    ## step 2 : assign the height value of each unique entry in lidar_top_pcl to the height map 
    ##          make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    ##          use the lidar_pcl_top data structure from the previous task to access the pixels of the height_map
    height_map[np.int_(lidar_pcl_top[:,0]), np.int_(lidar_pcl_top[:,1])]= (lidar_pcl_top[:,2] / float(np.abs(lim_z[1] - lim_z[0])))
    if vis==True:
        height_map=(height_map*256).astype(np.uint8)
        cv2.imshow("height_map", height_map)
        cv2.waitKey(0)


    ## step 3 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background
    #######
    ####### ID_S2_EX3 END #######       
    # Compute density layer of the BEV map
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    _, _, counts = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True, return_counts=True)
    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64)) 
    density_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = normalizedCounts
        


        

    # assemble 3-channel bev-map from individual maps
    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]  # r_map
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]  # g_map
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]  # b_map
    if vis==True:
        cv2.imshow("dd", density_map)
        cv2.waitKey(0)
    #print("bev_shape", np.shape(bev_map))
    '''im=torch.from_numpy(bev_map)
    im=(torch.permute(im, (1,2,0)))
    print(im.size())
    im=im.numpy()
    print(np.shape(im))'''
    #im=Image.fromarray(bev_map)
    '''plt.imshow(im)
    plt.show()'''
    
    '''cv2.imshow("bev_map", im)
    cv2.waitKey(0)'''

    # expand dimension of bev_map before converting into a tensor
    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map

    bev_maps = torch.from_numpy(bev_maps)  # create tensor from birds-eye view
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps


