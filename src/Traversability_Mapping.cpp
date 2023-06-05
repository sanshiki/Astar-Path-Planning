#include "Traversability_Mapping.h"


Eigen::MatrixXd Traversability_Map::getGradientX()
{
    Eigen::MatrixXd gradient_x(matrix.rows(), matrix.cols());
    gradient_x.setZero();
    for (int i = 0; i < matrix.rows(); ++i)
    {
        for (int j = 0; j < matrix.cols(); ++j)
        {
            if (j == 0)
            {
                gradient_x(i, j) = matrix.coeff(i, j + 1) - matrix.coeff(i, j);
            }
            else if (j == matrix.cols() - 1)
            {
                gradient_x(i, j) = matrix.coeff(i, j) - matrix.coeff(i, j - 1);
            }
            else
            {
                gradient_x(i, j) = (matrix.coeff(i, j + 1) - matrix.coeff(i, j - 1)) / 2;
            }
        }
    }
    return gradient_x;
}

Eigen::MatrixXd Traversability_Map::getGradientY()
{
    Eigen::MatrixXd gradient_y(matrix.rows(), matrix.cols());
    gradient_y.setZero();
    for (int i = 0; i < matrix.rows(); ++i)
    {
        for (int j = 0; j < matrix.cols(); ++j)
        {
            if (i == 0)
            {
                gradient_y(i, j) = matrix.coeff(i + 1, j) - matrix.coeff(i, j);
            }
            else if (i == matrix.rows() - 1)
            {
                gradient_y(i, j) = matrix.coeff(i, j) - matrix.coeff(i - 1, j);
            }
            else
            {
                gradient_y(i, j) = (matrix.coeff(i + 1, j) - matrix.coeff(i - 1, j)) / 2;
            }
        }
    }
    return gradient_y;
}

Eigen::MatrixXd Traversability_Map::getMap(){
            Eigen::MatrixXd gradientXLeft = getGradientX();
            Eigen::MatrixXd gradientYUp = getGradientY();
            Eigen::MatrixXd gradientXRight = gradientXLeft * -1;
            Eigen::MatrixXd gradientYDown = gradientYUp * -1;
            // setThreshold(0.16, -0.25);
            setThreshold(CONG::GO_UP_THRESHOLD,CONG::GO_DOWN_THRESHOLD);

            //将上下左右梯度矩阵根据阈值进行二值化
            Eigen::MatrixXd gradientXLeft_bin = (gradientXLeft.array() < up() && gradientXLeft.array() > down()).cast<double>();
            Eigen::MatrixXd gradientYUp_bin = (gradientYUp.array() < up() && gradientYUp.array() > down()).cast<double>();
            Eigen::MatrixXd gradientXRight_bin = (gradientXRight.array() < up() && gradientXRight.array() > down()).cast<double>();
            Eigen::MatrixXd gradientYDown_bin = (gradientYDown.array() < up() && gradientYDown.array() > down()).cast<double>();
            

            //当上下左右梯度矩阵均为1时，说明该点为可通行
            Eigen::MatrixXd traversabilityMap = (gradientXLeft_bin.array() + gradientYUp_bin.array() + gradientXRight_bin.array() + gradientYDown_bin.array() == 4).cast<double>();
            
            return traversabilityMap;
        }