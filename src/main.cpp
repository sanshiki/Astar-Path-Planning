#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "Path_Planning.h"
#include "MapIO.h"
#include "Traversability_Mapping.h"
using namespace std;


/*
TODO LIST:
* udp包                             基本完成
* 转弯阻尼                           已完成，暂时不用了
    * 转弯阻尼的细分
    * 转弯阻尼优化
* 代码排版整理                        基本完成
* CMake                             已完成
* 高城图处理迁移至C++                
    * 二值化                         已完成
* 贝塞尔曲线移植                      已完成，不保证有没有bug
* Point 改用Eigen库（可选）
* 最佳路径再尝试（可选）
* A*算法优化（可选）
* A*算法有bug：选取某些点，设定安全半径过大时有时候会卡在current = came_from[current]这一步，原因未知       已解决
* Debug开关
*/


int main()
{
    readConfigure();
    Traversability_Map elev_map(readMatrix2());
    Eigen::MatrixXd gradient_bin = elev_map.getMap();
    Point start;
    Point end;
    if(CONG::UDP == false)
    {
        start = Point(CONG::START_X,CONG::START_Y);
        end = Point(CONG::END_X,CONG::END_Y);
    }
    else
    {
        int startX,startY,endX,endY;
        udpGetData(startX,startY,endX,endY);
        start = Point(startX,startY);
        end = Point(endX,endY);
    }
    Path raw_path;
    // Point end(435,300);
    // Point end(404,288);

    const bool use_best = CONG::USE_BEST;          //是否使用最佳路径（未完成）

    if(use_best)
    {
        int inflation_size = 0;
        const int max_inflation_size = 15;

        /*
        安全系数：膨胀尺寸越大安全系数越高
        长度系数：路径欧几里德长度越短长度系数越高
        */
        map<int,int> path_record;   //第一个int用来记录膨胀尺寸，第二个int用来记录路径长度
        double safety_score,length_score;
        Eigen::MatrixXd temp_bin = gradient_bin;

        // raw_path = AStar(start,end,temp_bin);
        do
        {            
            inflation_size++;
            cout << "第" << inflation_size << "次膨胀" << endl;
            inflation(temp_bin,inflation_size,true);
            raw_path = AStar(start,end,temp_bin);
            path_pruning(raw_path);
            
            cout << "路径长度：" << raw_path.getLength() << endl;
            path_record[inflation_size] = raw_path.getLength();
            temp_bin = gradient_bin;
        }while (raw_path.size() != 0 && inflation_size < max_inflation_size);

        inflation_size -= 1;

        for(int i=1;i<=inflation_size;i++)
        {
            cout << "-------------第" << i << "次膨胀-------------" << endl;
            safety_score = (double)i/inflation_size;
            length_score = (1 - (double)path_record[i]/heuristic(start,end)/5)/1.6;
            path_record[i] = safety_score*1000 + length_score*1000;
            cout << "安全系数：" << safety_score << endl;
            cout << "长度系数：" << length_score << endl;
        }

        int best_inflation_size = 0;
        int best_score = 0;
        for(auto it=path_record.begin();it!=path_record.end();it++)
        {
            if(it->second > best_score)
            {
                best_score = it->second;
                best_inflation_size = it->first;
            }
        }

        //输出最佳path
        cout << "最佳膨胀尺寸：" << best_inflation_size << endl;
        inflation(gradient_bin,best_inflation_size,true);
        raw_path = AStar(start,end,gradient_bin);
        path_pruning(raw_path);
        writePath(raw_path);
        
    }
    else
    {
        raw_path = AStar(start,end,gradient_bin,CONG::SAFETY_ZONE);
        path_pruning(raw_path);
        bezier_curve_fit(raw_path);
        // cubic_spline_fit(raw_path);
        writePath(raw_path);
        writeMatrix(gradient_bin);
    }
    return 0;
}