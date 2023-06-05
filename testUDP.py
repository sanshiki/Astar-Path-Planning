import socket

# 创建UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 目标地址和端口
host = '127.0.0.1'  # C++接收方的IP地址
port = 12345  # C++接收方的端口号

# 发送数据
message = b"1 2 3 4"
sock.sendto(message, (host, port))

# 关闭套接字
sock.close()
