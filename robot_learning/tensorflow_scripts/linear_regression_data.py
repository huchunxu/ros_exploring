#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

# 从-1到1等差采样101个点
trX = np.linspace(-1, 1, 101)
# 根据线性关系计算每个x对应的y值，并且加入一些噪声
trY = 2 * trX + \
	np.ones(*trX.shape) * 4 + \
	np.random.randn(*trX.shape) * 0.03

# 创建点图
plt.figure(1)  # 选择图表1
plt.plot(trX, trY, 'o')

plt.xlabel('trX')    #为x轴加注释
plt.ylabel('trY')    #为y轴加注释
 
plt.show()