from functools import partial
from glob import glob
import cv2
import json
import numpy as np
import yaml

# Global data
feature_idx = 0
d_feature_num = 8

color_list = [
    (122, 0, 0),
    (0, 122, 0), 
    (0, 0, 122),
    (0, 122, 122),
    (122, 0, 122),
    (122, 122, 0),
    (255, 122, 0),
    (255, 255, 0),
]

# 2D to 3D method
# Raycast a 2D point to 3D
def ray_cast(intrinsic, image_point, depth):
    # Parse
    fx = intrinsic[0]
    fy = intrinsic[1]
    cx = intrinsic[2]
    cy = intrinsic[3]
    
    ix = image_point[0]
    iy = image_point[1]
    
    x = float(ix - cx) / fx * depth
    y = float(iy - cy) / fy * depth
    z = depth
    return [x, y, z]
    

def click_event(img_name, intrinsic, feature_list, event, x, y, flags, params):
    global feature_idx
    cross_size = 8	
    # Checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        if feature_idx >= d_feature_num:
            return  # Upper bound
        
        if (x < 30) and (y < 30):  # Click on the preserved space: meaning lacking this feature
            feature_list.append([0.0, 0.0, 0.0])
            print("Feature skipped")
        else:
            # Read depth
            depth = depth_img[y, x] / 1000.0  # (mm) to (m)
            # Raycast
            feature_pose = ray_cast(intrinsic, [x, y], depth)
            print("{}: {}, {}, {}".format(feature_idx, feature_pose[0], feature_pose[1], feature_pose[2]))
            feature_list.append(feature_pose)
            # draw crossing on the pixel
            cv2.line(color_img, (x - cross_size, y), (x + cross_size, y), color_list[feature_idx % len(color_list)], 1)
            cv2.line(color_img, (x ,y - cross_size), (x, y + cross_size), color_list[feature_idx % len(color_list)], 1)
            cv2.imshow(img_name, color_img)
            
        # update feature idx
        feature_idx = feature_idx + 1
       
    # Checking for right mouse clicks 
    if event == cv2.EVENT_RBUTTONDOWN:
        if feature_idx <= 0:
            return  # Lower bound
        if (feature_list):
            feature_idx = feature_idx - 1
            coord = feature_list.pop()
            # clean crossing on the pixel
            cv2.line(color_img, (coord[0] - cross_size, coord[1]), (coord[0] + cross_size, coord[1]), (255, 255, 255), 1)
            cv2.line(color_img, (coord[0], coord[1] - cross_size), (coord[0], coord[1] + cross_size), (255, 255, 255), 1)
            cv2.imshow(img_name, color_img)
            print("{} is poped".format(feature_idx))
        

# Driver function
if __name__=="__main__":
    # Constants
    d_num_cam = 3
    d_num_frame = 30
    d_step_frame = 10
    img_dir_path_root = "WebViewer3D/ros/data/test_data/calibration"
    intrinsic_path = "WebViewer3D/ros/data/io.yaml"
    # Intrinsics
    intrinsics = list()
    with open(intrinsic_path, 'r') as stream:
        camera_data = yaml.safe_load(stream)
        for _cam_idx in range(d_num_cam):
            intrinsics.append(
                camera_data["cam{}".format(_cam_idx)]["intrinsics"]
            )
    # intrinsics = [
    #     camera_data["cam0"]["intrinsics"],  
    #     camera_data["cam1"]["intrinsics"],
    #     camera_data["cam2"]["intrinsics"]  
    # ]
    feature_poses = list()
    for frame_idx in range(0, d_num_frame, d_step_frame):
        feature_pose_frame = list()  # frame[ feature_id[ cam[ poses]]]
        for _feature_idx in range(d_feature_num):
            feature_pose_frame.append(list())
            
        for cam_idx in range(d_num_cam):
            img_color_name = "{}/cam-{:02d}/frame-{:06d}.color.png".format(img_dir_path_root, cam_idx, frame_idx)
            img_depth_name = "{}/cam-{:02d}/frame-{:06d}.depth.png".format(img_dir_path_root, cam_idx, frame_idx)
            print("Opening {}.".format(img_color_name))
            # Reading the image
            color_img = cv2.imread(img_color_name, 1)   # Color
            depth_img = cv2.imread(img_depth_name, -1)  # Unchanged
            # Displaying the image
            img_name = "cam-{}/frame-{}".format(cam_idx, frame_idx)
            cv2.imshow(img_name, color_img)
            # Setting callback
            feature_idx = 0  # Reset feature idx
            feature_list = list()
            cv2.setMouseCallback(img_name, 
                partial(click_event, img_name, intrinsics[cam_idx], feature_list))
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            # Write
            for _feature_idx in range(d_feature_num):
                feature_pose_frame[_feature_idx].append(
                    feature_list[_feature_idx]
                )
        
        feature_poses.append(feature_pose_frame)
    
    with open("WebViewer3D/ros/data/feature_poses.json", "w") as output_file:
        json.dump(feature_poses, output_file)