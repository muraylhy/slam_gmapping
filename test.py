import pandas as pd
from sklearn.model_selection import train_test_split
import transforms3d.euler as tf_euler

# 读取CSV文件
data = pd.read_csv('/home/drl03/catkin_songcan/src/slam_gmapping/odom_data_02.csv')

# data['temp_imu_w'] = data['temp_imu_w'] * 3.1415926 / 180.0
z_values = data['odom_z'].tolist()
w_values = data['odom_ori_w'].tolist()
quaternions = list(zip(w_values, [0.0] * len(z_values), [0.0] * len(z_values), z_values))
yaw_angles = [tf_euler.quat2euler(q, axes='sxyz')[2] for q in quaternions]
data['yaw_angle'] = yaw_angles
data['yaw_angle'] = data['yaw_angle'].diff()

selected = ['timestamp', 'odom_w', 'yaw_angle']
test = data[selected][1:]
test.to_csv('/home/drl03/catkin_songcan/src/slam_gmapping/temp_yaw.csv', index=False)