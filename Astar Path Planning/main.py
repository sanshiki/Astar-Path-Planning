import numpy as np
from module import path_planning as pp
from module import display as disp

#-----------------------参数设置-----------------------#

start = (435,325)
# end = (153,47)
end = (267,68)

safety_radius = 15

#-----------------------路径规划-----------------------#

elevation_map = np.load('../data/map.npy')    #使用numpy载入npy文件

gradient_bin = pp.get_gradient_bin_map(elevation_map, 0.16, -0.25)   #获取梯度二值化地图

path = pp.astar_pathfinding(gradient_bin,start,end,safety_radius)    #A*算法寻路

path = pp.path_pruning(path)    #路径修剪

# path = pp.bezier_curve_fit(path)    #贝塞尔曲线拟合


#-----------------------显示结果-----------------------#

#将路径画在地图上
disp.draw_path(gradient_bin,path)

disp.show_map(gradient_bin)