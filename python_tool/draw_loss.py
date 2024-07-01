'''
    本脚本用于画出loss的二维图像
    可以设置delta和s作为调节变量
'''
import numpy as np
import matplotlib.pyplot as plt

# 参数设置
delta = 1.0  # 阈值参数
s = np.linspace(-3, 3, 500)  # 残差范围

# 定义损失函数
def trivial_loss(s):
    return s ** 2

def huber_loss(s, delta):
    return np.where(np.abs(s) <= delta, s ** 2, 2 * delta * np.abs(s) - delta ** 2)

def cauchy_loss(s, delta):
    return delta ** 2 * np.log1p((s / delta) ** 2)

def arctan_loss(s, delta):
    return delta ** 2 * np.arctan(s / delta)

# 计算损失值
trivial = trivial_loss(s)
huber = huber_loss(s, delta)
cauchy = cauchy_loss(s, delta)
arctan = arctan_loss(s, delta)

# 绘制图形
plt.figure(figsize=(10, 6))

plt.plot(s, trivial, label='Trivial Loss', linewidth=2)
plt.plot(s, huber, label='Huber Loss', linewidth=2)
plt.plot(s, cauchy, label='Cauchy Loss', linewidth=2)
plt.plot(s, arctan, label='Arctan Loss', linewidth=2)

plt.xlabel('Residual (s)')
plt.ylabel('Loss')
plt.title(f'Loss Functions (delta = {delta})')
plt.legend()
plt.grid(True)

plt.show()