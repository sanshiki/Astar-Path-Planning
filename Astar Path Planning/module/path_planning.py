import numpy as np
import time
import math
from queue import PriorityQueue
from scipy.special import comb


## 根据elevation_map得出高程变化率图
def get_gradient_bin_map(elevation_map, go_up_threshold, go_down_threshold):

    elevation_map_h = elevation_map[:,:,3]
    # 计算高程梯度
    gradient_x = np.diff(elevation_map_h, axis=1)
    gradient_y = np.diff(elevation_map_h, axis=0)

    #将x,y梯度矩阵化成相同大小
    gradient_x_left = np.pad(gradient_x, ((0,0),(0,1)), 'constant', constant_values=0)
    gradient_y_up = np.pad(gradient_y, ((0,1),(0,0)), 'constant', constant_values=0)

    gradient_x_right = -gradient_x_left
    gradient_y_down = -gradient_y_up

    #分别得到上、下、左、右的二值化可通行性矩阵
    up_bin = np.where(np.logical_and(gradient_y_up < go_up_threshold, gradient_y_up > go_down_threshold),1, 0)
    down_bin = np.where(np.logical_and(gradient_y_down < go_up_threshold, gradient_y_down > go_down_threshold),1, 0)
    left_bin = np.where(np.logical_and(gradient_x_left < go_up_threshold, gradient_x_left > go_down_threshold),1, 0)
    right_bin = np.where(np.logical_and(gradient_x_right < go_up_threshold, gradient_x_right > go_down_threshold),1, 0)

    #当上、下、左、右均可通行时，将此处置为1
    gradient_bin = np.where(np.logical_and(np.logical_and(up_bin == 1, down_bin == 1), np.logical_and(left_bin == 1, right_bin == 1)), 1, 0)

    return gradient_bin

def astar_pathfinding(grid,_start,_goal,safety_radius):
    start_time = time.time()
    start = _start
    goal = _goal

    # 定义启发函数，这里使用欧几里德距离
    def heuristic(node):
        return np.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)
    

    # 定义移动的四个方向（上、下、左、右）和对应的移动代价
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    costs = [1, 1, 1, 1]

    # 初始化数据结构
    frontier = PriorityQueue()
    frontier.put((0, start))  # 使用优先级队列存储候选节点
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
    # while frontier.qsize() > 0:
        current = frontier.get()[1]
        if frontier.qsize() > 0:
            temp = frontier.get()
            # print("priority:",temp[0])
            frontier.put(temp)

        if current == goal:
            break

        for i in range(len(directions)):
            direction = directions[i]
            new_node = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= new_node[0] < grid.shape[0] and 0 <= new_node[1] < grid.shape[1]:
                new_cost = cost_so_far[current] + costs[i]
                if grid[new_node] == 1 and (new_node not in cost_so_far or new_cost < cost_so_far[new_node]) and circle_inflation_search(grid,new_node[0],new_node[1],safety_radius) == False:
                    cost_so_far[new_node] = new_cost
                    priority = new_cost + heuristic(new_node)
                    frontier.put((priority, new_node))
                    came_from[new_node] = current
    # 从终点回溯构建路径
    path = []
    current = goal
    while current != start:
        path.append(current)
        try:
            current = came_from[current]
        except KeyError:
            return None
    path.append(start)
    path.reverse()
    print("Astar path finding done, time cost:",time.time() - start_time,"s")
    return path

def rect_limit(grid,x,y,d):
    x_min = 0
    x_max = 0
    y_min = 0
    y_max = 0
    if x < d:
        x_min = 0
        x_max = x + d
    elif x > grid.shape[0] - d:
        x_min = x - d
        x_max = grid.shape[0]
    else:
        x_min = x - d
        x_max = x + d
    if y < d:
        y_min = 0
        y_max = y + d
    elif y > grid.shape[1] - d:
        y_min = y - d
        y_max = grid.shape[1]
    else:
        y_min = y - d
        y_max = y + d
    return x_min,x_max,y_min,y_max



#对传入的可通行性图和给定的路径以及半径作矩形碰撞检测，已弃用
def collision_detection(grid,path,d):
    #定义矩形碰撞检测函数
    def rect_collision_detection(x,y,d):
        x_min,x_max,y_min,y_max = rect_limit(grid,x,y,d)
        
        for i in range(x_min,x_max):
            for j in range(y_min,y_max):
                if grid[i][j] == 0:
                    print("collide at: ",i,j)
                    return True
        return False

    result = {
        'collision':False,
        'collision_point':None
    }

    #对路径中的点进行碰撞检测
    for i in range(len(path)):
        if rect_collision_detection(path[i][0],path[i][1],d) == True:
            result['collision'] = True
            result['collision_point'] = path[i]
            break
    
    return result


#地图障碍物膨胀处理

#单点膨胀：将某个点周围d距离内的点都设置为障碍物
def rect_inflation(grid,x,y,d):
    x_min,x_max,y_min,y_max = rect_limit(grid,x,y,d)
    
    for i in range(x_min,x_max):
        for j in range(y_min,y_max):
            grid[i][j] = 0

    return grid

#单点检索：搜索某个点周围d距离内的点是否有障碍物，若有则返回True
def rect_inflation_search(grid,x,y,d):
    x_min,x_max,y_min,y_max = rect_limit(grid,x,y,d)
    
    for i in range(x_min,x_max):
        for j in range(y_min,y_max):
            if grid[i][j] == 0:
                return True
    return False

def circle_inflation_search(grid,x,y,d):
    x_min,x_max,y_min,y_max = rect_limit(grid,x,y,d)

    for i in range(x_min,x_max):
        for j in range(y_min,y_max):
            if grid[i][j] == 0:
                if (i-x)**2 + (j-y)**2 <= d**2:
                    return True

    return False


use_search = False

#对整个地图作膨胀处理
def inflation(grid,d):
    if use_search == True:
        if d <= 10:#d小于10时，直接对整个地图作膨胀处理
            temp_grid = grid.copy()
            for i in range(temp_grid.shape[0]):
                for j in range(temp_grid.shape[1]):
                    if temp_grid[i][j] == 0:
                        grid = rect_inflation(grid,i,j,d)
            return grid
        else:#d大于10时，对地图中可通行点进行检索
            temp_grid = grid.copy()
            for i in range(temp_grid.shape[0]):
                for j in range(temp_grid.shape[1]):
                    if temp_grid[i][j] == 1:
                        if rect_inflation_search(temp_grid,i,j,d) == True:
                            grid[i][j] = 2
            for i in range(grid.shape[0]):
                for j in range(grid.shape[1]):
                    if grid[i][j] == 2:
                        grid[i][j] = 0
            return grid
    else:
        temp_grid = grid.copy()
        for i in range(temp_grid.shape[0]):
            for j in range(temp_grid.shape[1]):
                if temp_grid[i][j] == 0:
                    grid = rect_inflation(grid,i,j,d)
        return grid

#路径剪枝
# def path_pruning(path):
#     pruned_path = []
#     pruned_path.append(path[0])
#     for i in range(1,len(path)-1):
#         if path[i-1][0] == path[i+1][0] or path[i-1][1] == path[i+1][1]:
#             continue
#         else:
#             pruned_path.append(path[i])
#     pruned_path.append(path[-1])
#     return pruned_path

def path_pruning(path):
    start_time = time.time()
    pruned_path = [path[0]]
    back_point = path[0]
    current_point = path[1]

    ang_err = 0.25
    point_min_dist = 8
    point_max_dist = 15

    for i in range(1,len(path)):
        front_point = path[i]
        ang1 = math.atan2(front_point[1]-current_point[1],front_point[0]-current_point[0])
        ang2 = math.atan2(current_point[1]-back_point[1],current_point[0]-back_point[0])
        err = abs(ang1-ang2)
        dist = math.sqrt((front_point[0]-current_point[0])**2 + (front_point[1]-current_point[1])**2)

        if (err > ang_err and dist < point_max_dist) or dist < point_min_dist:
            continue
        else:
            pruned_path.append(current_point)
            back_point = current_point
            current_point = front_point

    pruned_path.append(path[-1])
    print("path pruning done,time cost: ",time.time()-start_time,"s")
    return pruned_path


#贝塞尔曲线拟合
def bezier_curve_fit(path):
    start_time = time.time()
    def getB(i):
        t = comb(n, i) * init_t**i * (1 - init_t)**(n - i)
        # print("comb: ",comb(n, i))
        return np.array([t, t]).T

    points = np.array(path)    
    n = points.shape[0] - 1
    init_t = np.linspace(0, 1, 500)
    P = np.zeros((500, 2))    
    temp = getB(0) * points[0]
    for i in range(n + 1):
        P += getB(i) * points[i]
    P = P.astype(int)
    print("bezier curve fit done,time cost: ",time.time()-start_time,"s")
    return P.tolist()   
