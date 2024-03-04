import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from model import PosePredictionModel
import transforms3d.quaternions as tf_quaternions
import transforms3d.euler as tf_euler

import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# 读取CSV文件
data = pd.read_csv('amcl_odom_data.csv')

# z_values = data['amcl_ori_z'].tolist()
# w_values = data['amcl_ori_w'].tolist()
z_values = data['odom_fusion_z'].tolist()
w_values = data['odom_fusion_ori_w'].tolist()
quaternions = list(zip(w_values, [0.0] * len(z_values), [0.0] * len(z_values), z_values))
yaw_angles = [tf_euler.quat2euler(q, axes='sxyz')[2] for q in quaternions]
data['yaw_angle'] = yaw_angles

# 提取特征和标签
features = data[['odom_fusion_v', 'odom_fusion_w', 'yaw_angle']].iloc[:-1].values
# features = data[['odom_w']].values

data['delta_x'] = data['amcl_pos_x'].diff()
data['delta_y'] = data['amcl_pos_y'].diff()
data['delta_yaw'] = data['yaw_angle'].diff()
labels = data[['delta_x', 'delta_y', 'delta_yaw']].iloc[1:].values
# labels = data[['yaw_angle']].values

# 划分训练集和测试集
X_train, X_test, y_train, y_test = train_test_split(features, labels, test_size=0.2)

# 转换为PyTorch张量
X_train_tensor = torch.tensor(X_train, dtype=torch.float32)
y_train_tensor = torch.tensor(y_train, dtype=torch.float32)
X_test_tensor = torch.tensor(X_test, dtype=torch.float32)
y_test_tensor = torch.tensor(y_test, dtype=torch.float32)

# 初始化模型
input_size = 3  # 输入特征的数量
hidden_size = 128  # 隐藏层的神经元数量
output_size = 3  # 输出的位姿数量
model = PosePredictionModel(input_size, hidden_size, output_size)

# 定义损失函数和优化器
criterion = nn.MSELoss()
optimizer = optim.AdamW(model.parameters(), lr=0.001)

# 训练模型
num_epochs = 1000
for epoch in range(num_epochs):
    # 前向传播
    outputs = model(X_train_tensor)
    loss = criterion(outputs, y_train_tensor)

    # 反向传播和优化
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

    if (epoch+1) % 100 == 0:
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')

# 在测试集上评估模型
model.eval()
with torch.no_grad():  
    test_outputs = model(X_test_tensor)
    test_loss = criterion(test_outputs, y_test_tensor)
    print(f'Test Loss: {test_loss.item():.4f}')

# 保存模型
torch.save(model.state_dict(), 'pose_prediction_model.pth')