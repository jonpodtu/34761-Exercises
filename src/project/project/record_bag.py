import subprocess
import signal
import sys


# ros2 bag record -o /odom /particle_estimation_stack /particle_estimation_own
def signal_handler(sig, frame):
    print('Stopping the recording and exiting...')
    process.terminate()  # or process.kill() if terminate does not work
    sys.exit(0)

def start_rosbag(topics, save_path):
    command = f"rosbag record -o {save_path} {' '.join(topics)}"
    process = subprocess.Popen(command, shell=True)
    print(f"Recording started for topics: {topics} at {save_path}")
    return process

signal.signal(signal.SIGINT, signal_handler)

# Example usage:
save_directory = '/path/to/save/directory/bag_prefix'
topics_to_record = ["/camera/image_raw", "/odom", "/sensor_data"]
process = start_rosbag(topics_to_record, save_directory)