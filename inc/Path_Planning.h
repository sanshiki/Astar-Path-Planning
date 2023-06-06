#ifndef __PATH_PLANNING_H__
#define __PATH_PLANNING_H__

#include "include.h"

using namespace std;

enum DIRECTION {
    UP = 0,
    DOWN,
    LEFT,
    RIGHT,
    UP_LEFT,
    UP_RIGHT,
    DOWN_LEFT,
    DOWN_RIGHT
};

class Point {
    public:
        Point() {}
        Point(int x, int y) : x(x), y(y), direction(UP) {}
        Point(double x,double y) : x((int)x), y((int)y), direction(UP) {}
        Point(int x,int y, int direction) : x(x), y(y), direction(direction) {}
        Point(const Point& p) : x(p.x), y(p.y) {}
        int x;
        int y;
        int direction;
        Point operator+(const Point& p) {
            int dir = 0;
            if(p.x == 0 && p.y == -1) dir = UP;
            else if(p.x == 0 && p.y == 1) dir = DOWN;
            else if(p.x == -1 && p.y == 0) dir = LEFT;
            else if(p.x == 1 && p.y == 0) dir = RIGHT;
            else if(p.x == -1 && p.y == -1) dir = UP_LEFT;
            else if(p.x == 1 && p.y == -1) dir = UP_RIGHT;
            else if(p.x == -1 && p.y == 1) dir = DOWN_LEFT;
            else if(p.x == 1 && p.y == 1) dir = DOWN_RIGHT;
            return Point(x + p.x, y + p.y, dir);
        }
        Point operator*(const int& i) {
            return Point(x * i, y * i);
        }
        bool operator==(const Point& p) {
            return x == p.x && y == p.y;
        }
        bool operator!=(const Point& p) {
            return x != p.x || y != p.y;
        }
        Point& operator=(const Point& p) {
            x = p.x;
            y = p.y;
            return *this;
        }
        bool operator<(const Point& p) const {
            return x < p.x || (x == p.x && y < p.y);
        }
};

double heuristic(Point node, Point end);

class Path {
    protected:
        vector<Point> path;
        int length;

    public:
        Path() : length(0) {}
        Path(const Path& p) : path(p.path), length(p.length) {}
        Path(const vector<Point>& p) : path(p), length(p.size()) {}
        Path(int size, const Point& p) : path(size, p), length(size) {}
        Path& operator=(const Path& p) {
            path = p.path;
            length = p.length;
            return *this;
        }
        Path& operator=(const vector<Point>& p) {
            path = p;
            length = p.size();
            return *this;
        }
        Point& operator[](int i) {
            return path[i];
        }
        int size() const {
            return length;
        }
        // void push_back(const Point& p) {
        //     if(path.size() == 0) length++;
        //     else length += heuristic(path[path.size() - 1], p);
        //     path.push_back(p);
        // }
        void push_back(const Point p) {
            if(path.size() == 0) length++;
            else length += heuristic(path[path.size() - 1], p);
            path.push_back(p);
        }
        void push_back(int x, int y) {
            if(path.size() == 0) length++;
            else length += heuristic(path[path.size() - 1], Point(x, y));
            path.push_back(Point(x, y));
        }
        Point pop_back() {
            Point p = path.back();
            path.pop_back();
            length--;
            return p;
        }
        void push_front(const Point& p) {
            path.insert(path.begin(), p);
            length++;
        }
        void clear() {
            path.clear();
            length = 0;
        }
        void resize(int len) {
            path.resize(len);
            length = len;
        }
        void reserve(int len) {
            path.reserve(len);
        }
        void erase(int i) {
            path.erase(path.begin() + i);
            length--;
        }
        Point& front() {
            return path.front();
        }
        Point& end() {
            return path[path.size() - 1];
        }
        void reverse() {
            int i = 0;
            int j = path.size() - 1;
            while(i < j) {
                Point tmp = path[i];
                path[i] = path[j];
                path[j] = tmp;
                i++;
                j--;
            }
        }
        int getLength() const {
            return length;
        }

};

//上一版的PathRecord用的是两个vector，一个存path，一个存came_from_list，这样在查找时需要遍历path，效率较低。第二版使用map，效率更高。
class PathRecordv2 : public Path {
    private:
        map<Point, Point> came_from_list;
    public:
    PathRecordv2() : Path(), came_from_list() {}
    Point& operator[](const Point &node)
    {
        return came_from_list[node];
    }
    Point& operator=(const Point &node)
    {
        return came_from_list[node];
    }
    int size() const {
        return came_from_list.size();
    }
};

// struct Node {
//     int row;
//     int col;
//     double g_score;
//     double f_score;
//     Node* parent;

//     Node(int r, int c, double g, double f, Node* p) : row(r), col(c), g_score(g), f_score(f), parent(p) {}
// };

// class Node {
//     public:
//         Node() {}
//         Node(int r, int c, double g, double f, Node* p) : row(r), col(c), g_score(g), f_score(f), parent(p) {}
//         Node(const Node& n) : row(n.row), col(n.col), g_score(n.g_score), f_score(n.f_score), parent(n.parent) {}
//         Node(int r, int c) : row(r), col(c), g_score(0.0), f_score(0.0), parent(nullptr) {}
//         Node& operator=(const Node& n) {
//             row = n.row;
//             col = n.col;
//             g_score = n.g_score;
//             f_score = n.f_score;
//             parent = n.parent;
//             return *this;
//         }
//         int row;
//         int col;
//         double g_score;
//         double f_score;
//         Node* parent;
// };

class Node {
    public:
        Node() {}
        Node(int priority,Point pos) : priority(priority), pos(pos) {}
        int priority;
        Point pos;
};

// class Node {
//     public:
//         int row;
//         int col;
//         int g_score;
//         int f_score;
//         Node* parent;

//         Node(int r, int c, int g, int f, Node* p) : row(r), col(c), g_score(g), f_score(f), parent(p) {}
//         Node(const Node& n) : row(n.row), col(n.col), g_score(n.g_score), f_score(n.f_score), parent(n.parent) {}
//         Node(int r, int c) : row(r), col(c), g_score(0), f_score(0), parent(nullptr) {}

// };

class Cost {
    public:
        vector<Point> pos_list;
        vector<int> cost_list;
        int& operator[](Point pos) {
            for (int i = 0; i < pos_list.size(); i++) {
                if (pos_list[i].x == pos.x && pos_list[i].y == pos.y) {
                    return cost_list[i];
                }
            }
            pos_list.push_back(pos);
            cost_list.push_back(0);
            return cost_list[cost_list.size() - 1];
        }
        int find(Point pos) {
            for (int i = pos_list.size(); i >= 0 ; i--) {
                if (pos_list[i].x == pos.x && pos_list[i].y == pos.y) {
                    return i;
                }
            }
            return -1;
        }
        
};

//上一版的Cost用的是线性表存储数据，这一版用map存储数据
class Costv2 {
    public:
        map<Point,int> cost_map;
        int& operator[](Point pos) {
            // if (cost_map.find(pos) == cost_map.end()) {
            //     cost_map[pos] = 0;
            // }
            return cost_map[pos];
        }
        int find(Point pos) {
            if (cost_map.find(pos) != cost_map.end()) {
                return cost_map[pos];
            }
            return -1;
        }
        
};




double heuristic(Point a, Point b);
Path AStar(const Point& start, const Point& end, const Eigen::MatrixXd& grid,const int safety_zone=0);
std::vector<Node*> reconstructPath(Node* current);
std::vector<Node*> getNeighbours(const Node& node, const Eigen::MatrixXd& grid);
void rect_inflation(Eigen::MatrixXd& grid,int x,int y,int d);
bool rect_inflation_search(const Eigen::MatrixXd& grid,int x,int y,int d);
bool circle_inflation_search(const Eigen::MatrixXd& grid,int x,int y,int d);
void inflation(Eigen::MatrixXd& grid,int d,bool use_search);
void path_pruning(Path& path);
void bezier_curve_fit(Path& path);
void cubic_spline_fit(Path& path);

#endif