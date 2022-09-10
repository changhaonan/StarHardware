import os
import argparse

if __name__ == "__main__":
    # 1. Parse arguments, root_save_dir and root_data_dir
    default_root_dir = os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../data"))
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--root_save_dir", type=str, default=".")
    parser.add_argument("-d", "--root_data_dir", type=str, default=default_root_dir)
    args = parser.parse_args()

    root_save_dir = args.root_save_dir
    root_data_dir = args.root_data_dir

    # 2. Get through the  directory
    full_file_name_list = list()
    file_name_list = list()
    for root, dirs, files in os.walk(root_data_dir):
        for file in files:
            if (file.startswith("realsense2")):
                full_file_name_list.append(os.path.join(root, file))
                file_name_list.append(file)

    # 3. Generate the data
    color_topic_name = "/rgb_map"
    depth_topic_name = "/depth_map"
    for bag_file, file_name in zip(full_file_name_list, file_name_list):
        # 3.1. Generate the directory
        save_dir = os.path.join(root_save_dir, file_name.split(".")[0])
        if not os.path.exists(os.path.join(save_dir, "cam-00")):
            os.makedirs(os.path.join(save_dir, "cam-00"))
    
        # 3.2. Transfer the data
        os.system(f"rosrun monocam_record ros2img --bag_file {bag_file} --color_topic_name {color_topic_name} --depth_topic_name {depth_topic_name} --save_dir {save_dir}")

        # 3.3. log
        print(f"Transfer the data from {bag_file} to {save_dir} successfully!")