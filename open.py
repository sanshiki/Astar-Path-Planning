import numpy as np
import matplotlib.pyplot as plt
import scipy.misc
depthmap = np.load('map.npy')    #使用numpy载入npy文件

mapclip1 = depthmap[:,:,0]
mapclip2 = depthmap[:,:,1]
mapclip3 = depthmap[:,:,2]
mapclip4 = depthmap[:,:,3]

# plt.imshow(depthmap)              #执行这一行后并不会立即看到图像，这一行更像是将depthmap载入到plt里
# plt.colorbar()                   #添加colorbar
# scipy.misc.imsave("depth.png", depthmap)  #将depthmap保存为png灰度图
# plt.savefig('depthmap.png')       #执行后可以将文件保存为jpg格式图像，可以双击直接查看。也可以不执行这一行，直接执行下一行命令进行可视化。但是如果要使用命令行将其保存，则需要将这行代码置于下一行代码之前，不然保存的图像是空白的

plt.imshow(mapclip1)              #执行这一行后并不会立即看到图像，这一行更像是将depthmap载入到plt里
plt.savefig('./picture/mapclip1.png')

plt.imshow(mapclip2)              #执行这一行后并不会立即看到图像，这一行更像是将depthmap载入到plt里
plt.savefig('./picture/mapclip2.png')

plt.imshow(mapclip3)              #执行这一行后并不会立即看到图像，这一行更像是将depthmap载入到plt里
plt.savefig('./picture/mapclip3.png')

plt.imshow(mapclip4)              #执行这一行后并不会立即看到图像，这一行更像是将depthmap载入到plt里
plt.savefig('./picture/mapclip4.png')

## 根据map_h得出高程变化率图
# 计算高程梯度
gradient_x = np.diff(mapclip4, axis=1)
gradient_y = np.diff(mapclip4, axis=0)

#将梯度图二值化
threshold = 0.1
map_h_bin = np.where(gradient_x < threshold, 1, 0)

plt.imshow(gradient_x)
plt.savefig('./picture/gradient_x.png')
plt.imshow(gradient_y)
plt.savefig('./picture/gradient_y.png')


plt.show()                        #在线显示图像
