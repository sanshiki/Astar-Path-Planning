import numpy as np
import matplotlib.pyplot as plt
import scipy.misc
from sklearn.cluster import KMeans
from queue import PriorityQueue
import path_planning as pp


inflation_size = 0

use_ca = True

depthmap = np.load('data/map.npy')    #使用numpy载入npy文件

map_r = depthmap[:,:,0]
map_g = depthmap[:,:,1]
map_b = depthmap[:,:,2]
map_h = depthmap[:,:,3]

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


start = (435,325)
# end = (153,47)
end = (267,68)

complete = False
# use_max_inflation = True
use_max_inflation = False

iterate_time = 0
path = 0
if use_max_inflation == True:
    while complete == False:
        iterate_time += 1
        print("iterate_time:",iterate_time)
        print("inflation_size:",inflation_size)
        #地图障碍物膨胀处理
        abs_gradient_bin = final_plan.copy()
        abs_gradient_bin = pp.inflation(abs_gradient_bin, inflation_size)
        # 对二值化处理后的梯度图进行路径规划，使用A*算法
        temp_path = pp.astar_pathfinding(start,end,abs_gradient_bin)

        if temp_path == None:#路径规划失败
            complete = True
        else:
            print("Length of path:",pp.getLength(pp.bezier_curve_fit(temp_path)))
            inflation_size += 1
            path = temp_path
else:
    abs_gradient_bin = pp.inflation(abs_gradient_bin, 14)
    path = pp.astar_pathfinding(start,end,abs_gradient_bin)

#进行剪枝
path = pp.path_pruning(path)


#平滑化处理
path = pp.bezier_curve_fit(path)


    # #进行碰撞检测
    # res = pp.collision_detection(abs_gradient_bin,path,1)
    # if res['collision'] == True and iterate_time <= 100:
    #     #如果碰撞，则将碰撞点加入障碍物集合
    #     abs_gradient_bin[res['collision_point'][0]][res['collision_point'][1]] = 0
    #     #将新加入的障碍物膨胀处理
    #     abs_gradient_bin = pp.rect_inflation(abs_gradient_bin,res['collision_point'][0],res['collision_point'][1],inflation_size)
    # else:
    #     #如果不碰撞，则路径规划完成
    #     complete = True

map_after_plan = abs_gradient_bin.copy()
#将path中的点添加到map_after_plan中，使路径呈现红色
for i in range(len(path)):
    map_after_plan[path[i][0]][path[i][1]] = 3
    final_plan[path[i][0]][path[i][1]] = 3




##--------------------------绘制图像--------------------------##
fig, (ax2, ax3, ax4) = plt.subplots(1, 3, figsize=(6,6))
# im1 = ax1.imshow(abs_gradient, cmap='terrain')
# ax1.set_title('abs_gradient')
# ax1.axis('off')
# fig.colorbar(im1)


im2 = ax2.imshow(abs_gradient_bin, cmap='terrain')
ax2.set_title('abs_gradient_bin')
ax2.axis('off')
# fig.colorbar(im2)

im3 = ax3.imshow(map_after_plan, cmap='terrain')
ax3.set_title('map_after_plan')
ax3.axis('off')
# fig.colorbar(im3)

im4 = ax4.imshow(final_plan, cmap='terrain')
ax4.set_title('final_plan')
ax4.axis('off')
# fig.colorbar(im4)


plt.show()

#将路径保存到txt文件中
# np.savetxt('path.txt',path,fmt='%d',delimiter=',')


# #将梯度图二值化
# threshold = 0.1
# map_h_bin = np.where(gradient_x < threshold, 1, 0)

# # 使用K-means算法进行聚类
# binary_points = map_h_bin
# kmeans = KMeans(n_clusters=1, random_state=0)
# kmeans.fit(binary_points)

# # 获取聚类标签
# labels = kmeans.labels_

# # 获取散点图的坐标范围
# x_min, x_max = np.min(binary_points[:, 0]), np.max(binary_points[:, 0])
# y_min, y_max = np.min(binary_points[:, 1]), np.max(binary_points[:, 1])

# # 创建二维矩阵map_h_bin，并初始化为0
# map_h_bin = np.zeros((y_max - y_min + 1, x_max - x_min + 1), dtype=int)

# # 根据聚类标签将对应位置的值填入矩阵中
# for point, label in zip(binary_points, labels):
#     x, y = point[0] - x_min, point[1] - y_min
#     map_h_bin[y, x] = label

# 绘制高程图、高程梯度图和二值化梯度图
# fig, (ax1, ax2, ax3, ax4) = plt.subplots(1, 4, figsize=(12,12))


# im1 = ax1.imshow(map_h, cmap='terrain')
# ax1.set_title('Elevation Map')
# ax1.axis('off')

# im2 = ax2.imshow(gradient_x, cmap='terrain')
# ax2.set_title('Gradient X')
# ax2.axis('off')

# im3 = ax3.imshow(gradient_y, cmap='terrain')
# ax3.set_title('Gradient Y')
# ax3.axis('off')

# im4 = ax4.imshow(map_h_bin, cmap='gray')
# ax4.set_title('Gradient X Binary')
# ax4.axis('off')

# # fig.colorbar(im1)
# # fig.colorbar(im2)
# # fig.colorbar(im3)
# fig.colorbar(im4)

plt.show()