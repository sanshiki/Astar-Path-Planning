import matplotlib.pyplot as plt

# 将path中的二维点添加到二值化的地图上
def draw_path(grid,path):
    for point in path:
        grid[point[0]][point[1]] = 3



def show_map(grid):
    plt.imshow(grid, cmap='terrain')
    plt.axis('off')
    plt.show()