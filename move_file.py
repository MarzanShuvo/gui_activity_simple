import docker
import os
import time
import tarfile
import logging
from io import BytesIO
from datetime import datetime, timedelta
import schedule

# Configuration
IMAGE_NAME = 'olagh/zed_docker:env_var'
CONTAINER_DIRECTORY = '/root/ros2_ws/rosbag_files'
HOST_DIRECTORY = '/home/jetson/ros_bags'
CHECK_INTERVAL = 10*60
LOG_FILE = 'rosbag_mover.log'

# Setup logging
logging.basicConfig(filename=LOG_FILE, level=logging.INFO, 
                    format='%(asctime)s - %(levelname)s - %(message)s')

def log_and_print(message):
    logging.info(message)
    print(message)

def get_container_id(image_name):
    client = docker.from_env()
    containers = client.containers.list(filters={'ancestor': image_name})
    if containers:
        log_and_print(f"Found container {containers[0].id} for image {image_name}")
        return containers[0].id
    else:
        log_and_print(f"No running container found with image {image_name}")
        raise Exception(f"No running container found with image {image_name}")

def list_files_in_container(container_id, container_directory):
    client = docker.from_env()
    container = client.containers.get(container_id)
    command = f"ls -1 {container_directory}"
    exit_code, output = container.exec_run(command)
    if exit_code != 0:
        error_message = output.decode('utf-8')
        log_and_print(f"Error listing files in container: {error_message}")
        raise Exception(f"Error listing files in container: {error_message}")
    file_list = output.decode('utf-8').strip().split('\n')
    log_and_print(f"Files in container ({container_directory}): {file_list}")
    return file_list

def copy_files_from_container(container_id, container_directory, host_directory):
    client = docker.APIClient(base_url='unix://var/run/docker.sock')
    tar_stream, _ = client.get_archive(container_id, container_directory)
    
    tar_data = BytesIO()
    for chunk in tar_stream:
        tar_data.write(chunk)
    tar_data.seek(0)
    
    with tarfile.open(fileobj=tar_data) as tar:
        tar.extractall(path=host_directory)
        log_and_print(f"Extracted files to {host_directory}")

def move_new_rosbag_folders():
    container_id = get_container_id(IMAGE_NAME)
    
    try:
        # Get the list of files in the container directory
        container_files = list_files_in_container(container_id, CONTAINER_DIRECTORY)
        
        # Get the list of files in the host directory
        files_in_host = os.listdir(HOST_DIRECTORY)
        log_and_print(f"Files in host ({HOST_DIRECTORY}): {files_in_host}")
        
        # Identify new files
        new_files = set(container_files) - set(files_in_host)
        log_and_print(f"New files to move: {new_files}")
        
        # Move new files to host directory
        if new_files:
            for new_file in new_files:
                src = os.path.join(CONTAINER_DIRECTORY, new_file)
                dest = os.path.join(HOST_DIRECTORY, new_file)
                copy_files_from_container(container_id, src, dest)
                log_and_print(f"Moved {new_file} from container to host.")
        else:
            log_and_print("No new files to move.")
                
    except Exception as e:
        log_and_print(f"Error: {e}")

def run_for_30_minutes():
    log_and_print("Starting 30-minute run.")
    start_time = datetime.now()
    end_time = start_time + timedelta(minutes=30)
    
    while datetime.now() < end_time:
        move_new_rosbag_folders()
        time.sleep(CHECK_INTERVAL)  # Wait for the specified interval before checking again
    
    log_and_print("Completed 30-minute run.")

def schedule_task():
    log_and_print("Scheduling task to run every 1 hours.")
    schedule.every(7).hours.do(run_for_30_minutes)  # Schedule the function to run every 30 minutes

    while True:
        schedule.run_pending()
        time.sleep(1)

if __name__ == "__main__":
    log_and_print("Starting the scheduler script.")
    schedule_task()
