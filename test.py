import pandas as pd
from sklearn.model_selection import train_test_split
import transforms3d.euler as tf_euler

# 读取CSV文件
data = pd.read_csv('./amcl_and_odom_data.csv')

z_values = data['odom_fusion_z'].tolist()
w_values = data['odom_fusion_ori_w'].tolist()
quaternions = list(zip(w_values, [0.0] * len(z_values), [0.0] * len(z_values), z_values))
yaw_angles = [tf_euler.quat2euler(q, axes='sxyz')[2] for q in quaternions]
data['yaw_angle'] = yaw_angles
data['yaw_angle'] = data['yaw_angle'].diff()

selected = ['odom_w', 'yaw_angle']
test = data[selected]
test.to_csv('./test.csv', index=False)