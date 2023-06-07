#include "Path_Planning.h"
#include <boost/multiprecision/cpp_int.hpp>
#include <boost/multiprecision/cpp_bin_float.hpp>

using namespace boost::multiprecision;


// double heuristic(Point a, Point b) {
//     return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
// }

// Path AStar(const Point& start, const Point& end, const Eigen::MatrixXd& grid) {
//     int rows = grid.rows();
//     int cols = grid.cols();

//     Node* startNode = new Node(start.x, start.y, 0.0, heuristic(start, end), nullptr);
//     std::vector<Node*> openSet {startNode};
//     std::vector<Node*> closedSet;

//     while (!openSet.empty()) {
//         // 从openSet中选择f_score最小的节点
//         Node* current = openSet[0];
//         int currentIndex = 0;
//         for (int i = 1; i < openSet.size(); ++i) {
//             if (openSet[i]->f_score < current->f_score) {
//                 current = openSet[i];
//                 currentIndex = i;
//             }
//         }

//         // 到达目标节点，返回最优路径
//         if (current->row == end.x && current->col == end.y) {
//             std::vector<Node*> nodePath = reconstructPath(current);
//             std::vector<Point> tuplePath;
//             for (Node* node : nodePath) {
//                 tuplePath.push_back(Point(node->row, node->col));
//             }
//             return Path(tuplePath);
//         }

//         // 移动当前节点到closedSet中
//         openSet.erase(openSet.begin() + currentIndex);
//         closedSet.push_back(current);

//         // 获取当前节点的相邻节点
//         std::vector<Node*> neighbours = getNeighbours(*current, grid);

//         for (Node* neighbour : neighbours) {
//             // 跳过已在closedSet中的节点
//             if (std::find(closedSet.begin(), closedSet.end(), neighbour) != closedSet.end()) {
//                 continue;
//             }

//             double tentative_g_score = current->g_score + 1.0; // 这里假设每个相邻节点的代价为1.0

//             // 如果该相邻节点已在openSet中且新的路径代价更大，则跳过
//             if (std::find(openSet.begin(), openSet.end(), neighbour) != openSet.end() && tentative_g_score >= neighbour->g_score) {
//                 continue;
//             }

//             // 更新相邻节点的代价和父节点
//             neighbour->g_score = tentative_g_score;
//             neighbour->f_score = neighbour->g_score + heuristic(Point(neighbour->row, neighbour->col), end);
//             neighbour->parent = current;

//             // 如果相邻节点不在openSet中，则将其加入openSet
//             if (std::find(openSet.begin(), openSet.end(), neighbour) == openSet.end()) {
//                 openSet.push_back(neighbour);
//             }
//         }
//     }

//     // 无法找到最优路径，返回空路径
//     //输出路径
//     for(int i = 0;i < closedSet.size();i++)
//     {
//         std::cout << closedSet[i]->row << " " << closedSet[i]->col << std::endl;
//     }
//     return Path();
// }

// std::vector<Node*> reconstructPath(Node* current) {
//     std::vector<Node*> totalPath {current};
//     while (current->parent != nullptr) {
//         current = current->parent;
//         totalPath.push_back(current);
//     }
//     std::reverse(totalPath.begin(), totalPath.end());
//     return totalPath;
// }

// std::vector<Node*> getNeighbours(const Node& node, const Eigen::MatrixXd& grid) {
//     std::vector<Node*> neighbours;
//     int rows = grid.rows();
//     int cols = grid.cols();

//     if (node.row > 0 && grid(node.row - 1, node.col) == 0.0) {
//         neighbours.push_back(new Node(node.row - 1, node.col));
//     }
//     if (node.row < rows - 1 && grid(node.row + 1, node.col) == 0.0) {
//         neighbours.push_back(new Node(node.row + 1, node.col));
//     }
//     if (node.col > 0 && grid(node.row, node.col - 1) == 0.0) {
//         neighbours.push_back(new Node(node.row, node.col - 1));
//     }
//     if (node.col < cols - 1 && grid(node.row, node.col + 1) == 0.0) {
//         neighbours.push_back(new Node(node.row, node.col + 1));
//     }

//     return neighbours;
// }


// 欧几里德距离
double heuristic(Point node, Point end)
{
    return std::sqrt((node.x - end.x) * (node.x - end.x) + (node.y - end.y) * (node.y - end.y));
}

struct cmp{
    bool operator()(Node a, Node b)
    {
        return a.priority > b.priority;
    }
};


// A*算法
Path AStar(const Point& start, const Point& end, const Eigen::MatrixXd& grid,const int safety_zone)
{
    //                               up          down       left        right      up-left      up-right    down-left   down-right
    std::vector<Point> directions = {Point(0,-1),Point(0,1),Point(-1,0),Point(1,0),Point(-1,-1),Point(1,-1),Point(-1,1),Point(1,1)};
    int costs[8] = {1,1,1,1,1,1,1,1};

    std::priority_queue<Node, vector<Node>, cmp> frontier;

    PathRecordv2 came_from;
    Costv2 cost_so_far;
    cost_so_far[start] = 0;
    frontier.push(Node(0,start));
    bool pathFound = false;
    clock_t start_time = clock();

    while(!frontier.empty())
    {
        // cout << "-------------------------------------------------" << endl;
        Point current = frontier.top().pos;
        frontier.pop();

        if(current == end)
        {
            pathFound = true;
            break;
        }

        clock_t lap1 = clock();
        for(int i=0;i<directions.size();i++)
        {
            clock_t lap2 = clock();
            Point direction = directions[i];
            Point new_pos = current + direction;
            if((new_pos.x >= 0 && new_pos.x < grid.rows()) && (new_pos.y >= 0 && new_pos.y < grid.cols()))
            {
                int new_cost = cost_so_far[current] + costs[i];
                if(grid(new_pos.x,new_pos.y) == 1 && (cost_so_far.find(new_pos) == -1 || new_cost < cost_so_far[new_pos]) && !circle_inflation_search(grid,new_pos.x,new_pos.y,safety_zone)/*!rect_inflation_search(grid,new_pos.x,new_pos.y,safety_zone)*/)
                {
                    cost_so_far[new_pos] = new_cost;
                    int priority = new_cost + heuristic(new_pos,end);
                    // if(current.direction != new_pos.direction) priority *= CONG::TURN_DAMPING;       //转弯阻尼，已废弃
                    frontier.push(Node(priority,new_pos));
                    came_from[new_pos] = current;
                    // cout << "current: " << current.x << " " << current.y << " " << current.direction << endl;
                }
            }
            // cout << "第" << i << "次循环用时：" << (double)(clock() - lap2) / CLOCKS_PER_SEC << "s" << endl;
        }
        // cout << "总循环用时：" << (double)(clock() - lap1) / CLOCKS_PER_SEC << "s" << endl;
    }

    //输出路径
    Path path;
    if(came_from.size() == 0 || !pathFound)
    {
        std::cout << "No path found!" << std::endl;
    }    
    else
    {
        std::cout << "Path found!" << std::endl;
        cout << "A* spend:" << (double)(clock() - start_time) / CLOCKS_PER_SEC << "s" << endl;
        Point current = end;
        while(current != start)
        {
            path.push_front(current);
            current = came_from[current];
        }

        path.push_front(start);
    }

    return path;
}


void rect_limit(int r,int c,int d,int x,int y,int& x_min,int& x_max,int& y_min,int& y_max)
{
    if(x - d < 0)
    {
        x_min = 0;
        x_max = x + d;
    }
    else if (x + d > r - 1)
    {
        x_min = x - d;
        x_max = r - 1;
    }
    else
    {
        x_min = x - d;
        x_max = x + d;
    }

    if(y - d < 0)
    {
        y_min = 0;
        y_max = y + d;
    }
    else if (y + d > c - 1)
    {
        y_min = y - d;
        y_max = c - 1;
    }
    else
    {
        y_min = y - d;
        y_max = y + d;
    }
}

//单点膨胀：将某个点周围d距离内的点都设置为障碍物
void rect_inflation(Eigen::MatrixXd& grid,int x,int y,int d)
{
    int x_min,x_max,y_min,y_max;
    rect_limit(grid.rows(),grid.cols(),d,x,y,x_min,x_max,y_min,y_max);

    for(int i=x_min;i<=x_max;i++)
    {
        for(int j=y_min;j<=y_max;j++)
        {
            grid(i,j) = 0;
        }
    }
}

//单点检索：搜索某个点周围d距离内的点是否有障碍物，若有则返回True
bool rect_inflation_search(const Eigen::MatrixXd& grid,int x,int y,int d)
{
    int x_min,x_max,y_min,y_max;
    rect_limit(grid.rows(),grid.cols(),d,x,y,x_min,x_max,y_min,y_max);

    for(int i=x_min;i<=x_max;i++)
    {
        for(int j=y_min;j<=y_max;j++)
        {
            if(grid(i,j) == 0)
            {
                return true;
            }
        }
    }
    return false;
}

bool circle_inflation_search(const Eigen::MatrixXd& grid,int x,int y,int d)
{
    int x_min,x_max,y_min,y_max;
    rect_limit(grid.rows(),grid.cols(),d,x,y,x_min,x_max,y_min,y_max);

    for(int i=x_min;i<=x_max;i++)
    {
        for(int j=y_min;j<=y_max;j++)
        {
            if(grid(i,j) == 0)
            {
                if((i-x)*(i-x) + (j-y)*(j-y) <= d*d)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

//对整个地图做膨胀
void inflation(Eigen::MatrixXd& grid,int d,bool use_search)
{
    if(use_search)
    {
        Eigen::MatrixXd temp_grid = grid;
        for(int i=0;i<grid.rows();i++)
        {
            for(int j=0;j<grid.cols();j++)
            {
                if(temp_grid(i,j) == 0)
                {
                    rect_inflation(grid,i,j,d);
                }
            }
        }
    }
    else
    {
        Eigen::MatrixXd temp_grid = grid;
        for(int i=0;i<grid.rows();i++)
        {
            for(int j=0;j<grid.cols();j++)
            {
                if(temp_grid(i,j) == 0)
                {
                    rect_inflation(grid,i,j,d);
                }
            }
        }
    }
}

//路径剪枝：将路径中的冗余点去掉
void path_pruning(Path& path)
{
    vector<Point> pruned_path;
    pruned_path.push_back(path[0]);
    cout << "path pruning..." << endl;
    Point back_point = path[0];
    Point current_point = path[1];
    for(int i=1;i<path.size();i++)
    {
        //v1：仅仅去掉了非必要的拐点
        // if(heuristic(path[i-1],path[i+1]) < 2)
        // {
        //     path.erase(i);
        //     i--;
        // }
        // else
        // {
        //     continue;
        // }

        //v2：
        Point front_point = path[i];
        //如果current_point大约在back_point和front_point的连线上，则去掉current_point
        double ang1 = atan2(front_point.y - back_point.y,front_point.x - back_point.x);
        double ang2 = atan2(current_point.y - back_point.y,current_point.x - back_point.x);
        double err = abs(ang1 - ang2);
        // double err = heuristic(back_point,current_point) + heuristic(current_point,front_point) - heuristic(back_point,front_point);
        // cout << "err: " << err << endl;
        if((err < 0.25 && heuristic(current_point,front_point) < 15) || heuristic(current_point,front_point) < 8)
        {
            continue;
        }
        else
        {
            pruned_path.push_back(current_point);
            back_point = current_point;
            current_point = front_point;
        }
    }
    pruned_path.push_back(path[path.size()-1]);
    path = pruned_path;
    cout << "Path pruning done!" << endl;
}

//贝塞尔曲线拟合
/*
可以仿照以下的python代码：
def bezier_curve_fit(path):
    def getB(i):
        t = comb(n, i) * init_t**i * (1 - init_t)**(n - i)
        return np.array([t, t]).T

    points = np.array(path)
    n = points.shape[0] - 1
    init_t = np.linspace(0, 1, 1000)
    P = np.zeros((1000, 2))
    for i in range(n + 1):
        P += getB(i) * points[i]
    P = P.astype(int)
    return P.tolist()
*/


// void bezier_curve_fit(Path& path) {
//     int n = path.size() - 1;
//     std::vector<double> init_t(1000);
//     for (int i = 0; i < 1000; i++) {
//         init_t[i] = i / 999.0;
//     }

//     Path P(1000, Point(0, 0));
//     for (int i = 0; i <= n; i++) {
//         Path B(n + 1, Point(0, 0));
//         for (int j = 0; j <= n; j++) {
//             int coef = 1;
//             for (int k = 0; k <= n; k++) {
//                 if (k == j)
//                     continue;
//                 coef *= (i - k);
//             }
//             coef /= static_cast<double>(i - j);

//             for (int t = 0; t < 1000; t++) {
//                 B[j] = B[j] + path[j] * coef * pow(init_t[t], j) * pow(1 - init_t[t], n - j);
//             }
//         }
//         for (int t = 0; t < 1000; t++) {
//             P[t] = P[t] + B[i];
//         }
//     }

//     path = P;
// }

// void bezier_curve_fit(Path& path) {
//     int n = path.size() - 1;
//     vector<double> init_t(n);
//     for (int i = 0; i < n; i++) {
//         init_t[i] = i / static_cast<double>(n - 1);
//     }

//     vector<Point> P(n);
//     for (int i = 0; i < n; i++) {
//         Point point(0, 0);
//         for (int j = 0; j <= n; j++) {
//             double coef = 1;
//             for (int k = 0; k <= n; k++) {
//                 if (k == j)
//                     continue;
//                 coef *= (i - k) / static_cast<double>(j - k);
//             }
//             point.x += path[j].x * coef * std::pow(init_t[i], j) * std::pow(1 - init_t[i], n - j);
//             point.y += path[j].y * coef * std::pow(init_t[i], j) * std::pow(1 - init_t[i], n - j);
//         }
//         P[i] = Point(static_cast<int>(point.x), static_cast<int>(point.y));
//         cout << "point: " << point.x << ", " << point.y << endl;
//     }

//     path = P;
// }

#define LINSPACE_SIZE 500

// void bezier_curve_fit(Path& path_) {

//     std::vector<Eigen::Vector2d> path;
//     //Path类转Vector2d
//     for (int i = 0; i < path_.size(); i++) {
//         path.push_back(Eigen::Vector2d(path_[i].x, path_[i].y));
//     }

//     auto getB = [](int i, int n, const Eigen::VectorXd& init_t) {
//         Eigen::VectorXd t = Eigen::VectorXd::Zero(init_t.size());
//         for (int j = 0; j < init_t.size(); j++) {
//             double comb = 1.0;
//             for (int k = 0; k < n; k++) {
//                 if (k == i)
//                     continue;
//                 comb *= (i - k);
//             }
//             comb /= static_cast<double>(i - j);

//             t(j) = comb * pow(init_t(j), i) * pow(1.0 - init_t(j), n - i);
//         }
//         return t.transpose();
//     };

//     Eigen::MatrixX2d points(path.size(), 2);
//     for (int i = 0; i < path.size(); i++) {
//         points.row(i) = path[i].transpose();
//     }

//     int n = points.rows() - 1;
//     Eigen::VectorXd init_t = Eigen::VectorXd::LinSpaced(LINSPACE_SIZE, 0.0, 1.0);
//     Eigen::MatrixX2d P = Eigen::MatrixX2d::Zero(LINSPACE_SIZE, 2);
//     for (int i = 0; i <= n; i++) {
//         Eigen::VectorXd B = getB(i, n, init_t);
//         for (int j = 0; j < LINSPACE_SIZE; j++) {
//             P.row(j) += B(j) * points.row(i);
//         }
//     }


//     // Vector2d转Path类
//     for (int i = 0; i < P.rows(); i++) {
//         path_[i].x = P(i, 0);
//         path_[i].y = P(i, 1);
//     }
// }

cpp_bin_float_oct comb(int n,int i)
{
    //循环
    cpp_bin_float_oct res = 1;
    for(int j=0;j<i;j++)
    {
        res *= (cpp_bin_float_oct)(n-j)/(cpp_bin_float_oct)(j+1);
    }
    return res;
}   


cpp_bin_float_oct bezier_pow(cpp_bin_float_oct t,int i)
{
    for(int j=0;j<i;j++)
    {
        t *= t;
    }
    return t;
}

cpp_bin_float_oct getB(int i,int n,int t)
{
    // 将公式comb(n,i)*bezier_pow(t,i)*bezier_pow(1-t,n-i) 合在一起计算
    // 分开算：comb过大而bezier_pow过小，会导致溢出
    cpp_bin_float_oct res = 1;
    for(int j=0;j<i;j++)
    {
        res *= (n-j)/(j+1);
        res *= t;
    }
    for(int j=0;j<n-i;j++)
    {
        res *= (n-i-j)/(j+1);
        res *= (1-t);
    }
    return res;
}


Point bezier_curve(Path& path,cpp_bin_float_oct t)
{
    int n = path.size() - 1;
    cpp_bin_float_oct tempX,tempY;
    Point res;
    static bool comb_flag = false;
    static vector<cpp_bin_float_oct> comb_vec;
    for(int i=0;i<=n;i++)
    {
        if(comb_flag == false)
        {
            comb_vec.push_back(comb(n,i));
            // cout << "comb in cpp: " << comb_vec[i] << endl;
        }
        cpp_bin_float_oct b = comb_vec[i]*pow(t,i)*pow(1-t,n-i);
        // cout << "comb=" << comb_vec[i] << ", b=" << b << " pow(t,i)=" << pow(t,i) << endl;
        // cpp_bin_float_oct b = getB(i,n,t);
        tempX += path[i].x*b;
        tempY += path[i].y*b;
        // cout << "calculating: " << path[i].x << ", " << path[i].y << "| i=" << i << ", n=" << n << ", t=" << t << endl;
    }
    comb_flag = true;
    
    if(t == 1) //最后一个点，生成完这个点后重置标志位并清空组合数容器
    {
        comb_flag = false;
        comb_vec.clear();
    }
    
    res.x = tempX.convert_to<int>();
    res.y = tempY.convert_to<int>();
    // cout << "-------------------generate point: " << res.x << ", " << res.y << endl;
    // cout << "tempX=" << tempX << ", tempY=" << tempY << endl;
    return res;
}

void bezier_curve_fit(Path& path)
{
    vector<Point> res;
    for(cpp_bin_float_oct t;t <= 1;t+=1.0/LINSPACE_SIZE)
    {
        res.push_back(bezier_curve(path,t));
    }
    path = res;
    cout << "Bezier Curve Fit Done!" << endl;
}


void cubic_spline_fit(Path& path)
{
    vector<Point> smoothed_path;
    smoothed_path.push_back(path[0]);
    for(int i=0;i<path.size();i++)
    {
        if(i <= path.size() - 4)
        {
            // cout << "i=" << i << endl;
            int insertNum = heuristic(path[i],path[i+3]);
            for(double t = 0;t<=1;t+=1.0/insertNum)
            {
                double a1 = pow(1-t,3)/6;
                double a2 = (3*pow(t,3)-6*pow(t,2)+4)/6;
                double a3 = (-3*pow(t,3)+3*pow(t,2)+3*t+1)/6;
                double a4 = pow(t,3)/6;
                double x = a1*path[i].x + a2*path[i+1].x + a3*path[i+2].x + a4*path[i+3].x;
                double y = a1*path[i].y + a2*path[i+1].y + a3*path[i+2].y + a4*path[i+3].y;
                smoothed_path.push_back(Point(x,y));
                // smoothed_path.push_back(path[i+1]);
                // cout << "t=" << t << ", x=" << x << ", y=" << y << endl;
                // cout << "newly add: " << smoothed_path[smoothed_path.size()-1].x << ", " << smoothed_path[smoothed_path.size()-1].y << endl;
            }
            // cout << "point(" << path[i].x << ", " << path[i].y << ") changes to point(" << smoothed_path[smoothed_path.size()-1].x << ", " << smoothed_path[smoothed_path.size()-1].y << ")" << endl;
        }

    }
    smoothed_path.push_back(path[path.size()-1]);
    path = smoothed_path;
    cout << "Cubic Spline Fit Done!" << endl;
}
