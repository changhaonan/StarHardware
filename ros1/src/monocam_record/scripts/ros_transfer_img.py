import os

if __name__ == "__main__":
    bag_file = "/home/robot-learning/Projects/StarHardware/ros1/src/monocam_record/data/realsense2_2022-09-09-22-06-52.bag"
    color_topic_name = "/rgb_map"
    depth_topic_name = "/depth_map"
    save_dir = "."
    os.system(f"rosrun monocam_record ros2img --bag_file {bag_file} --color_topic_name {color_topic_name} --depth_topic_name {depth_topic_name} --save_dir {save_dir}")