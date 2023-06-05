#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>
#include <cmath>
#include <queue>
#include <eigen3/Eigen/Core>
#include <ctime>

namespace CONG
{
    extern int MAT_SIZE;
    extern int SAFETY_ZONE;
    extern bool USE_BEST;
    extern double TURN_DAMPING;
    extern bool UDP;
    extern std::string HOST;
    extern int PORT;
    extern int START_X;
    extern int START_Y;
    extern int END_X;
    extern int END_Y;
    extern double GO_UP_THRESHOLD;
    extern double GO_DOWN_THRESHOLD;
}

#endif // __INCLUDE_H__