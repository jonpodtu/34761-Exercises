path = '/Users/jdkm/Library/Mobile Documents/com~apple~CloudDocs/DTU-data/34761 - Robot Autonomy/jonas-exersices-final-project/data/rosbag2_2024_04_23-14_28_38_0/rosbag2_2024_04_23-14_28_38_0/'

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# create reader instance and open for reading
with Reader(path) as reader:
    # topic and msgtype information is available on .connections list

    print(reader.connections)

    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        # if connection.topic == '/imu_raw/Imu':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            print(msg.header.frame_id)
# df_laser = pd.read_csv(LASER_MSG)
# df_laser # prints laser data in the form of pandas dataframe


# def plot_rosbag(bag_file, topic):
#     timestamps, data_values = [], []
    
#     # Read messages and gather data directly
#     with rosbag.Bag(bag_file) as bag:
#         for _, msg, t in bag.read_messages(topics=[topic]):
#             data_values.append(msg.data)
#             timestamps.append(t.to_sec())

#     # Plot the data
#     plt.plot(timestamps, data_values, label='Sensor Data')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Data Value')
#     plt.title(f'Data from topic {topic}')
#     plt.legend()
#     plt.grid(True)
#     plt.show()

# # Example usage
# plot_rosbag('/path/to/your/rosbag.bag', '/sensor_data')