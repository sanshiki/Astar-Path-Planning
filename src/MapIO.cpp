#include "MapIO.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <unistd.h>

namespace CONG
{
    int MAT_SIZE;
    int SAFETY_ZONE;
    bool USE_BEST;
    double TURN_DAMPING;
    bool UDP;
    string HOST;
    int PORT;
    int START_X;
    int START_Y;
    int END_X;
    int END_Y;
    double GO_UP_THRESHOLD;
    double GO_DOWN_THRESHOLD;
}

//读取numpy输出的矩阵
Eigen::MatrixXd readMatrix()
{
    const int mat_size = CONG::MAT_SIZE;
    std::ifstream file("./data/Matrix.txt");
    if (!file) {
        std::cerr << "无法打开矩阵文件" << std::endl;
        return Eigen::MatrixXd();
    }

    // 读取文件的行数和列数
    int rows = mat_size,cols = mat_size;

    Eigen::MatrixXd matrix(rows, cols); // 创建Eigen矩阵

    // 逐个读取文件中的数据并存储到矩阵中
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            file >> matrix(i, j);
        }
    }

    file.close(); 

    return matrix;
}

Eigen::MatrixXd readMatrix2()
{
    const int mat_size = CONG::MAT_SIZE;
    std::ifstream file("./data/Elevation_map.txt");
    if (!file) {
        std::cerr << "无法打开矩阵文件" << std::endl;
        return Eigen::MatrixXd();
    }

    // 读取文件的行数和列数
    int rows = mat_size,cols = mat_size;

    Eigen::MatrixXd matrix(rows, cols); // 创建Eigen矩阵

    // 逐个读取文件中的数据并存储到矩阵中
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            file >> matrix(i, j);
        }
    }

    file.close(); 

    return matrix;
}

//将矩阵保存为txt文件
void writeMatrix(const Eigen::MatrixXd& mat)
{
    std::ofstream file("./data/gradient_bin.txt");
    if (!file) {
        std::cerr << "无法打开文件" << std::endl;
        return;
    }

    // 逐个读取矩阵中的数据并存储到文件中
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            file << mat(i, j) << " ";
        }
        file << std::endl;
    }

    file.close();
}

void writePath(Path& path)
{
    std::ofstream file("./data/path.txt");
    if (!file) {
        std::cerr << "无法打开路径文件" << std::endl;
        return;
    }

    // 逐个读取矩阵中的数据并存储到文件中
    for (int i = 0; i < path.size(); ++i) {
        file << path[i].x << " " << path[i].y << std::endl;
    }

    file.close();
}

//读取配置文件
void readConfigure()
{
    std::ifstream file("Configure.txt");
    if (!file) {
        std::cerr << "无法打开配置文件" << std::endl;
        return;
    }

    std::string line;
    std::string key;
    std::string value;

    while (getline(file,line))
    {
        key = line.substr(0,line.find("="));
        value = line.substr(line.find("=")+1,line.length());
        //去掉空格
        key.erase(0,key.find_first_not_of(" "));
        key.erase(key.find_last_not_of(" ") + 1);
        value.erase(0,value.find_first_not_of(" "));
        value.erase(value.find_last_not_of(" ") + 1);
        if(key == std::string("SAFETY_ZONE"))
        {
            CONG::SAFETY_ZONE = stoi(value);
        }
        else if(key == std::string("USE_BEST"))
        {
            if(value == std::string("true"))
            {
                CONG::USE_BEST = true;
            }
            else
            {
                CONG::USE_BEST = false;
            }
        }
        else if(key == std::string("MAT_SIZE"))
        {
            CONG::MAT_SIZE = stoi(value);
        }
        else if(key == std::string("TURN_DAMPING"))
        {
            CONG::TURN_DAMPING = stod(value);
        }
        else if(key == std::string("UDP"))
        {
            if(value == std::string("true"))
            {
                CONG::UDP = true;
            }
            else
            {
                CONG::UDP = false;
            }
        }
        else if(key == std::string("HOST"))
        {
            CONG::HOST = value;
        }
        else if(key == std::string("PORT"))
        {
            CONG::PORT = stoi(value);
        }
        else if(key == std::string("GO_UP_THRESHOLD"))
        {
            CONG::GO_UP_THRESHOLD = stod(value);
        }
        else if(key == std::string("START"))    //START的形式：START = (x,y)
        {
            int x = stoi(value.substr(1,value.find(",")-1));
            int y = stoi(value.substr(value.find(",")+1,value.find(")")-value.find(",")-1));
            CONG::START_X = x;
            CONG::START_Y = y;
        }
        else if(key == std::string("END"))    //END的形式：END = (x,y)
        {
            int x = stoi(value.substr(1,value.find(",")-1));
            int y = stoi(value.substr(value.find(",")+1,value.find(")")-value.find(",")-1));
            CONG::END_X = x;
            CONG::END_Y = y;
        }
        else if(key == std::string("GO_UP_THRESHOLD"))
        {
            CONG::GO_UP_THRESHOLD = stod(value);
        }
        else if(key == std::string("GO_DOWN_THRESHOLD"))
        {
            CONG::GO_DOWN_THRESHOLD = stod(value);
        }
        else
        {
            std::cerr << "未知配置信息：" << key << std::endl;
            return;
        }
    }
}

void udpGetData(int& startX,int& startY,int& endX,int& endY)
{
    // 创建UDP套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    
    // 设置地址和端口
    struct sockaddr_in Addr{};
    Addr.sin_family = AF_INET;
    Addr.sin_port = htons(CONG::PORT);  // 本地监听端口
    Addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 监听所有网络接口
    
    // 绑定套接字到本地地址和端口
    bind(sockfd, (struct sockaddr*)&Addr, sizeof(Addr));
    
    // 接收数据，如果超时则报错
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    ssize_t numBytes = recvfrom(sockfd, buffer, sizeof(buffer)-1, 0, nullptr, nullptr);
    if (numBytes < 0) {
        std::cerr << "接收数据失败" << std::endl;
        return;
    }

    // 接受数据的一般形式：x1 y1 x2 y2 
    //数字和数字之间用空格隔开，最后一个数字后面没有空格
    // 解包
    std::string str(buffer);
    std::string::size_type pos = str.find(" ");
    startX = stoi(str.substr(0,pos));
    str = str.substr(pos+1,str.length());
    pos = str.find(" ");
    startY = stoi(str.substr(0,pos));
    str = str.substr(pos+1,str.length());
    pos = str.find(" ");
    endX = stoi(str.substr(0,pos));
    str = str.substr(pos+1,str.length());
    endY = stoi(str.substr(0,str.length()));
    
    // 关闭套接字
    close(sockfd);
}