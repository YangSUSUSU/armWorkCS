from numpy.polynomial.polynomial import Polynomial
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# 原始信号（量化后的数据）
x = np.linspace(0, 10, 5)
y = np.sin(x)

# 使用立方样条插值
cs = CubicSpline(x, y)
x_interp = np.linspace(0, 10, 100)  # 更细的x轴
y_interp = cs(x_interp)

plt.plot(x, y, 'o', label="Quantized Signal")
plt.plot(x_interp, y_interp, label="Cubic Spline Interpolation")
plt.legend()
plt.show()
