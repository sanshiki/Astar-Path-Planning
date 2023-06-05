#ifndef __TRAVERSABILITY_MAPPING_H__
#define __TRAVERSABILITY_MAPPING_H__

#include "include.h"

class Traversability_Map
{
    public:
        Traversability_Map() : go_up_threshold(0), go_down_threshold(0) {}
        Traversability_Map(const Eigen::MatrixXd& mat) : matrix(mat), go_up_threshold(0), go_down_threshold(0) {}
        Eigen::MatrixXd getGradientX();
        Eigen::MatrixXd getGradientY();
        Eigen::MatrixXd getMap();
        void setThreshold(double up, double down) {
            this->go_up_threshold = up;
            this->go_down_threshold = down;
        }
        double up() { return this->go_up_threshold; }
        double down() { return this->go_down_threshold; }
    private:
        Eigen::MatrixXd matrix;
        double go_up_threshold;//上坡阈值
        double go_down_threshold;//下坡阈值
};

#endif // !__Traversability_Mapping