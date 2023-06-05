#ifndef __MAPIO_H__
#define __MAPIO_H__

#include "include.h"
#include "Path_Planning.h"

using namespace std;




Eigen::MatrixXd readMatrix();
Eigen::MatrixXd readMatrix2();
void writeMatrix(const Eigen::MatrixXd& mat);
void writePath(Path& path);
void readConfigure();
void udpGetData(int& startX,int& startY,int& endX,int& endY);


#endif // !__MAPIO_H__

