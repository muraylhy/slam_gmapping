import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

data = pd.read_csv('./test.csv')

x_data = data['odom_w'].values
y_data = data['yaw_angle'].values

mean_value = np.mean(y_data)
std_dev = np.std(y_data)
outlier_threshold = std_dev

cleaned_indices = np.abs(y_data - mean_value) < outlier_threshold
x_data_cleaned = x_data[cleaned_indices]
y_data_cleaned = y_data[cleaned_indices]

# 定义拟合函数
def linear_function(x, slope, intercept):
    return slope * x + intercept

# 利用 curve_fit 进行拟合
params, covariance = curve_fit(linear_function, x_data_cleaned, y_data_cleaned)

# 提取拟合参数
slope, intercept = params

# 打印拟合结果
print("斜率:", slope)
print("截距:", intercept)

# 生成拟合曲线的预测值
fit_line = linear_function(x_data_cleaned, slope, intercept)

# 绘制原始数据和拟合曲线
plt.scatter(x_data_cleaned, y_data_cleaned, label='原始数据')
plt.plot(x_data_cleaned, fit_line, color='red', label='拟合曲线')
plt.show()