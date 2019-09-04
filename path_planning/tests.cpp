#include"GridMap.h"
#include"astar.h"
#include <Eigen/Dense>
#include <iostream>
#include <string.h>
#include<set>
#include<queue>


class compare_values {
    bool operator()(const std::pair<int, float>& a, const std::pair<int, float>& b) const {
        return a.second < b.second;
    }
};


int main(int argc, char *argv[]) {

//    std::set<int> s;
//    s.insert(1);
//    s.insert(5);
//    s.insert(7);
//
//    for (auto const& element : s)
//    {
//        std::cout << element << '\n';
//    }
//
//    s.erase(1);
//    s.erase(2);
//
//    for (auto const& element : s)
//    {
//        std::cout << element << '\n';
//    }


//    std::set<std::pair<int, float>> chk;
//    chk.insert({1,3.0});



//    std::set<std::pair<int, float>, compare_values> fValuesTest;
//      not sure why I get an error when inserting (related to custom compare function)
//    fValuesTest.insert({1,3.0});
//    fValuesTest.insert({2,1.0});
//    fValuesTest.insert({3,2.0});
//    fValuesTest.insert({4,0.5});
//
//    for (const std::pair<int, float>& element : fValuesTest)
//    {
//        std::cout << element.first << "\t" << element.second << '\n';
//    }

    // note that this doesn't enforce uniqueness
//    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> q;
//    q.push({3.0, 1});
//    q.push({2.0, 7});
//    q.push({6.0, 2});
//    q.push({5.0, 10});
//
//
//    while (!q.empty())
//    {
//        std::pair<float, int> element = q.top();
//        std::cout << element.first << "\t" << element.second << '\n';
//        q.pop();
//    }

    {
        int numPoints = 100;
        int numSteps = 10;
        if (argc == 3) {
            numPoints = std::stoi(argv[1]);
            numSteps = std::stoi(argv[2]);
        }

        Eigen::MatrixXf mockPcd = Eigen::MatrixXf::Random(numPoints, 3);
//    Eigen::MatrixXf mockPcd2 = Eigen::MatrixXf::Random(numPoints,3);
        float stepSize = (mockPcd.maxCoeff() - mockPcd.minCoeff()) / (float) numSteps;
        GridMap map(stepSize, mockPcd);
        std::cout << map;
        map.printMapValues();

        int startIdx = 5;
        int goalIdx = 89;
        std::vector<int> path;
        bool res = PathPlanning::a_star(map, startIdx, goalIdx, path);


        std::cout << "Planning result " << res << ", for " << startIdx << " to " << goalIdx << "\n";
        if (res) {
            Eigen::Vector2i p;
            for (auto const &e: path) {
                map.getXY(e,p);
                std::cout << e << ": " << p.transpose() << " : " << map.cellData(e, GridMap::MEAN) << "\n";
            }
        }
    }

    std::cout << "\n\n\n";
    {
        int numPoints = 100;
        int numSteps = 10;
        if (argc == 3) {
            numPoints = std::stoi(argv[1]);
            numSteps = std::stoi(argv[2]);
        }

        Eigen::MatrixXf mockPcd = Eigen::MatrixXf::Random(numPoints, 3);
//    Eigen::MatrixXf mockPcd2 = Eigen::MatrixXf::Random(numPoints,3);
        float stepSize = (mockPcd.maxCoeff() - mockPcd.minCoeff()) / (float) numSteps;
        GridMap map(stepSize, mockPcd);
        std::cout << map;
        map.printMapValues();

        int startIdx = 5;
        int goalIdx = 89;
        std::vector<int> path;
        bool res = PathPlanning::a_star(map, startIdx, goalIdx, path);

        std::cout << "Planning result " << res << ", for " << startIdx << " to " << goalIdx << "\n";
        Eigen::Vector2i p;
        if (res) {
            for (auto const &e: path) {
                map.getXY(e, p);
                std::cout << e << ": " << p.transpose() << " : " << map.cellData(e, GridMap::MEAN) << "\n";
            }
        }

        path.clear();
        res = PathPlanning::a_star(map, startIdx, goalIdx, PathPlanning::Heuristic::ZERO, path);

        std::cout << "Planning result " << res << ", for " << startIdx << " to " << goalIdx << "\n";
        if (res) {
            for (auto const &e: path) {
                map.getXY(e, p);
                std::cout << e << ": " << p.transpose() << " : " << map.cellData(e, GridMap::MEAN) << "\n";
            }
        }
    }


    std::cout << "\n\n\n";
    {
        int numPoints = 1000;
        int numSteps = 10;
        if (argc == 3) {
            numPoints = std::stoi(argv[1]);
            numSteps = std::stoi(argv[2]);
        }

        Eigen::MatrixXf mockPcd = Eigen::MatrixXf::Random(numPoints, 3);
//    Eigen::MatrixXf mockPcd2 = Eigen::MatrixXf::Random(numPoints,3);
        float stepSize = (mockPcd.maxCoeff() - mockPcd.minCoeff()) / (float) numSteps;
        GridMap map(stepSize, mockPcd);
        std::cout << map;
        map.printMapValues();

        int startIdx = 5;
        int goalIdx = 89;
        std::vector<int> path;
        bool res = PathPlanning::a_star(map, startIdx, goalIdx, PathPlanning::Heuristic::DIAGONAL, path);

        std::cout << "Planning result " << res << ", for " << startIdx << " to " << goalIdx << "\n";
        Eigen::Vector2i p;
        if (res) {
            for (auto const &e: path) {
                map.getXY(e, p);
                std::cout << e << ": " << p.transpose() << " : " << map.cellData(e, GridMap::MEAN) << "\n";
            }
        }

        path.clear();
        res = PathPlanning::a_star(map, startIdx, goalIdx, PathPlanning::Heuristic::DIAGONAL,
                                    GridMap::STAT::STAT_MEAN, GridMap::COST_TYPE::ABS_CHANGE, path);

        std::cout << "Planning result " << res << ", for " << startIdx << " to " << goalIdx << "\n";
        if (res) {
            for (auto const &e: path) {
                map.getXY(e, p);
                std::cout << e << ": " << p.transpose() << " : " << map.cellData(e, GridMap::MEAN) << "\n";
            }
        }

        path.clear();
        res = PathPlanning::a_star(map, startIdx, goalIdx, PathPlanning::Heuristic::DIAGONAL,
                                   GridMap::STAT::STAT_STDEV, GridMap::COST_TYPE::TO_VALUE, path);

        std::cout << "Planning result " << res << ", for " << startIdx << " to " << goalIdx << "\n";
        if (res) {
            for (auto const &e: path) {
                map.getXY(e, p);
                std::cout << e << ": " << p.transpose() << " : " << map.cellData(e, GridMap::STDEV) << "\n";
            }
        }
    }

}
