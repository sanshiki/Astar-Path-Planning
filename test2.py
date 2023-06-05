import numpy as np
import matplotlib.pyplot as plt
import scipy.misc
from sklearn.cluster import KMeans
from queue import PriorityQueue
import path_planning as pp


inflation_size = 0

depthmap = np.load('map.npy')    #使用numpy载入npy文件

map_r = depthmap[:,:,0]
map_g = depthmap[:,:,1]
map_b = depthmap[:,:,2]
map_h = depthmap[:,:,3]

## 根据map_h得出高程变化率图
# 计算高程梯度
gradient_x = np.diff(map_h, axis=1)
gradient_y = np.diff(map_h, axis=0)


#将x,y梯度矩阵化成相同大小
gradient_x_left = np.pad(gradient_x, ((0,0),(0,1)), 'constant', constant_values=0)
gradient_y_up = np.pad(gradient_y, ((0,1),(0,0)), 'constant', constant_values=0)

gradient_x_right = -gradient_x_left
gradient_y_down = -gradient_y_up

# 计算梯度图的梯度
gradient_x = np.diff(gradient_x, axis=1)
gradient_y = np.diff(gradient_y, axis=0)

#上坡阈值
go_up_threshold = 0.12
#下坡阈值
go_down_threshold = -0.20

#分别得到上、下、左、右的二值化可通行性矩阵
up_bin = np.where(np.logical_and(gradient_y_up < go_up_threshold, gradient_y_up > go_down_threshold),1, 0)
down_bin = np.where(np.logical_and(gradient_y_down < go_up_threshold, gradient_y_down > go_down_threshold),1, 0)
left_bin = np.where(np.logical_and(gradient_x_left < go_up_threshold, gradient_x_left > go_down_threshold),1, 0)
right_bin = np.where(np.logical_and(gradient_x_right < go_up_threshold, gradient_x_right > go_down_threshold),1, 0)

#当上、下、左、右均可通行时，将此处置为1
gradient_bin = np.where(np.logical_and(np.logical_and(up_bin == 1, down_bin == 1), np.logical_and(left_bin == 1, right_bin == 1)), 1, 0)

##--------------------------绘制图像--------------------------##
fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(1, 5, figsize=(6,6))

ax1.imshow(up_bin, cmap='gray')
# ax1.imshow(gradient_x, cmap='gray')
ax1.set_title('up_bin')

ax2.imshow(down_bin, cmap='gray')
# ax2.imshow(gradient_y, cmap='gray')
ax2.set_title('down_bin')

ax3.imshow(left_bin, cmap='gray')
ax3.set_title('left_bin')

ax4.imshow(right_bin, cmap='gray')
ax4.set_title('right_bin')

ax5.imshow(gradient_bin, cmap='gray')
ax5.set_title('gradient_bin')

plt.show()