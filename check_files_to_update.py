import docker
import os
import time
import tarfile
from io import BytesIO

IMAGE_NAME = 'olagh/humble_zed_humble:latest'  # Image name to search for
CONTAINER_DIRECTORY = '/root/ros2_ws/rosbag_files'
HOST_DIRECTORY = '/home/jetson/ros_bags'
CHECK_INTERVAL = 10

def get_container_id(image_name):
    client = docker.from_env()
    containers = client.containers.list(filters={'ancestor': image_name})
    if containers:
        return containers[0].id
    else:
        raise Exception(f"No running container found with image {image_name}")

def list_files_in_container(container_id, container_directory):
    client = docker.from_env()
    container = client.containers.get(container_id)
    command = f"ls -1 {container_directory}"
    exit_code, output = container.exec_run(command)
    if exit_code != 0:
        raise Exception(f"Error listing files in container: {output.decode('utf-8')}")
    return output.decode('utf-8').strip().split('\n')

def copy_files_from_container(container_id, container_directory, host_directory):
    client = docker.APIClient(base_url='unix://var/run/docker.sock')
    tar_stream, _ = client.get_archive(container_id, container_directory)
    
    tar_data = BytesIO()
    for chunk in tar_stream:
        tar_data.write(chunk)
    tar_data.seek(0)
    
    with tarfile.open(fileobj=tar_data) as tar:
        tar.extractall(path=host_directory)

def move_new_rosbag_folders():
    container_id = get_container_id(IMAGE_NAME)
    
    while True:
        try:
            # Get the list of files in the container directory
            container_files = list_files_in_container(container_id, CONTAINER_DIRECTORY)
            
            # Get the list of files in the host directory
            files_in_host = os.listdir(HOST_DIRECTORY)
            
            # Identify new files
            new_files = set(container_files) - set(files_in_host)
            
            # Move new files to host directory
            if new_files:
                for new_file in new_files:
                    src = os.path.join(CONTAINER_DIRECTORY, new_file)
                    dest = os.path.join(HOST_DIRECTORY, new_file)
                    copy_files_from_container(container_id, src, dest)
                    print(f"Moved {new_file} from container to host.")
            else:
                print("No new files to move.")
        
        except Exception as e:
            print(f"Error: {e}")
        
        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    move_new_rosbag_folders()
