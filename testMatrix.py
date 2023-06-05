import numpy as np
import matplotlib.pyplot as plt
import path_planning as pp
import subprocess

# 创建一个numpy矩阵
# a = np.load('./data/map.npy')    #使用numpy载入npy文件
a = np.load('./data/map.npy')    #使用numpy载入npy文件

map_h = a[:,:,3]

np.savetxt('data/Elevation_map.txt', map_h, fmt='%f')

use_ca = True

if use_ca == True:
    ## 根据map_h得出高程变化率图
    # 计算高程梯度
    gradient_x = np.diff(map_h, axis=1)
    gradient_y = np.diff(map_h, axis=0)

    #将x,y梯度矩阵化成相同大小
    gradient_x_left = np.pad(gradient_x, ((0,0),(0,1)), 'constant', constant_values=0)
    gradient_y_up = np.pad(gradient_y, ((0,1),(0,0)), 'constant', constant_values=0)

    gradient_x_right = -gradient_x_left
    gradient_y_down = -gradient_y_up

    #上坡阈值
    go_up_threshold = 0.16
    #下坡阈值
    go_down_threshold = -0.25

    #分别得到上、下、左、右的二值化可通行性矩阵
    up_bin = np.where(np.logical_and(gradient_y_up < go_up_threshold, gradient_y_up > go_down_threshold),1, 0)
    down_bin = np.where(np.logical_and(gradient_y_down < go_up_threshold, gradient_y_down > go_down_threshold),1, 0)
    left_bin = np.where(np.logical_and(gradient_x_left < go_up_threshold, gradient_x_left > go_down_threshold),1, 0)
    right_bin = np.where(np.logical_and(gradient_x_right < go_up_threshold, gradient_x_right > go_down_threshold),1, 0)

    #当上、下、左、右均可通行时，将此处置为1
    abs_gradient_bin = np.where(np.logical_and(np.logical_and(up_bin == 1, down_bin == 1), np.logical_and(left_bin == 1, right_bin == 1)), 1, 0)

    final_plan = abs_gradient_bin.copy()

else:
    ## 根据map_h得出高程变化率图
    # 计算高程梯度
    gradient_x = np.diff(map_h, axis=1)
    gradient_y = np.diff(map_h, axis=0)

    #将x,y梯度矩阵化成相同大小
    gradient_x = np.pad(gradient_x, ((0,0),(0,1)), 'constant', constant_values=0)
    gradient_y = np.pad(gradient_y, ((0,1),(0,0)), 'constant', constant_values=0)


    abs_gradient = np.sqrt(gradient_x**2 + gradient_y**2)

    abs_gradient_bin = np.where(abs_gradient < 0.14, 1, 0)

    final_plan = abs_gradient_bin.copy()



# 输出为Matrix.txt，数据类型为整型
np.savetxt('data/Matrix.txt', final_plan, fmt='%d')

# 执行可知性文件main
subprocess.call('./build/main')

#将Matrix1.txt读取为矩阵
# b = np.loadtxt('Matrix1.txt')

#Path.txt中存着二维元组，表示路径
path = np.loadtxt('data/path.txt', dtype=int)

path = path.tolist()

path = pp.bezier_curve_fit(path)

# 将读取的path添加到final_plan中
for i in range(len(path)):
    final_plan[path[i][0]][path[i][1]] = 0.5



#用plt显示矩阵
fig,(ax1,ax2) = plt.subplots(1,2,figsize=(10,10))

ax1.imshow(abs_gradient_bin, cmap='terrain')
ax2.imshow(final_plan, cmap='terrain')
# ax2.imshow(b, cmap='terrain')

plt.show()

