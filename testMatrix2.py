import numpy as np
import matplotlib.pyplot as plt
import subprocess
import path_planning as pp

import socket

import threading
import time

def udpSendData():
    time.sleep(1)
    # 创建UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 目标地址和端口
    host = '127.0.0.1'  # C++接收方的IP地址
    port = 12345  # C++接收方的端口号

    # 发送数据
    message = b"69 171 332 466"
    sock.sendto(message, (host, port))

    # 关闭套接字
    sock.close()


# 开两个线程，一个执行可执行文件，一个执行udpSendData
t1 = threading.Thread(target=subprocess.call, args=('./build/main',))
t2 = threading.Thread(target=udpSendData)
t1.start()
t2.start()
t1.join()
t2.join()



#将Matrix1.txt读取为矩阵
gradient_bin = np.loadtxt('data/gradient_bin.txt', dtype=int)
final_plan = gradient_bin.copy()

#Path.txt中存着二维元组，表示路径
path = np.loadtxt('data/path.txt', dtype=int)

path = path.tolist()

if len(path) > 0:
    # path = pp.bezier_curve_fit(path)

    # 将读取的path添加到final_plan中
    for i in range(len(path)):
        final_plan[path[i][0]][path[i][1]] = 0.5



    #用plt显示矩阵
    fig,ax1 = plt.subplots(1,figsize=(10,10))

    # ax1.imshow(abs_gradient_bin, cmap='terrain')
    ax1.imshow(final_plan, cmap='terrain')
    # ax2.imshow(b, cmap='terrain')

    plt.show()
else:
    print("路径规划失败！")