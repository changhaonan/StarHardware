import os

if __name__ == "__main__":
    # 1. Prepare the path
    bag_file = "/home/robot-learning/Projects/StarHardware/ros1/src/monocam_record/data/realsense2_2022-09-09-22-44-25.bag"
    color_topic_name = "/rgb_map"
    depth_topic_name = "/depth_map"
    save_dir = "./"

    # 2. Create the directory
    if not os.path.exists(os.path.join(save_dir, "cam-00")):
        os.makedirs(os.path.join(save_dir, "cam-00"))
    
    # 3. Transfer the data
    os.system(f"rosrun monocam_record ros2img --bag_file {bag_file} --color_topic_name {color_topic_name} --depth_topic_name {depth_topic_name} --save_dir {save_dir}")

    # 4. Log
    print(f"Transfer the data from {bag_file} to {save_dir} successfully!")