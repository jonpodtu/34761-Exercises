import rosbag
import matplotlib.pyplot as plt

def plot_rosbag(bag_file, topic):
    timestamps, data_values = [], []
    
    # Read messages and gather data directly
    with rosbag.Bag(bag_file) as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            data_values.append(msg.data)
            timestamps.append(t.to_sec())

    # Plot the data
    plt.plot(timestamps, data_values, label='Sensor Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Data Value')
    plt.title(f'Data from topic {topic}')
    plt.legend()
    plt.grid(True)
    plt.show()

# Example usage
plot_rosbag('/path/to/your/rosbag.bag', '/sensor_data')