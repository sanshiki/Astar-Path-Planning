#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <unistd.h>

int main() {
    // 创建UDP套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    
    // 设置本地地址和端口
    struct sockaddr_in localAddr{};
    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(12345);  // 本地监听端口
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);  // 监听所有网络接口
    
    // 绑定套接字到本地地址和端口
    bind(sockfd, (struct sockaddr*)&localAddr, sizeof(localAddr));
    
    // 接收数据
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    ssize_t numBytes = recvfrom(sockfd, buffer, sizeof(buffer)-1, 0, nullptr, nullptr);
    
    // 输出接收到的数据
    if (numBytes > 0) {
        std::cout << "Received message: " << buffer << std::endl;
    }
    
    // 关闭套接字
    close(sockfd);
    
    return 0;
}
