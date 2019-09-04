#include"GridMap.h"
#include"astar.h"
#include <Eigen/Dense>
#include <iostream>
#include <string.h>
#include<set>
#include<queue>
#include <MockMap.h>


class compare_values {
    bool operator()(const std::pair<int, float>& a, const std::pair<int, float>& b) const {
        return a.second < b.second;
    }
};


int main(int argc, char *argv[]) {


    Eigen::MatrixXf mockCellData(228,3);
    Eigen::VectorXf tmp(228);
    tmp << 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 125.848 , 0.000 , 0.000 , 0.000 , 33.407 ,
            0.000 , 0.000 , 37.206 , 0.000 , -66.893 , 188.334 , 97.944 , 110.233 , 0.000 , 85.043 , 84.322 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , -31.861 ,
            0.000 , 0.000 , 53.415 , 76.885 , 115.815 , 109.391 , 101.586 , 95.636 , 81.786 , 60.877 , 0.000 , 0.000 , 0.000 , 0.000 , -34.249 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 125.899 , 123.997 , 109.039 , 111.000 , 102.994 , 104.932 , 102.991 , 83.434 , 74.630 , 34.259 , 0.763 , -83.253 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 0.000 , 85.956 , 107.736 , 127.365 , 110.455 , 97.370 , 67.194 , 48.525 , -74.780 , -37.386 , -35.826 , 246.275 , -52.276 , 0.000 , -50.289 , 0.000 , 0.000 ,
            -19.296 , 0.000 , -28.658 , 123.018 , 109.643 , 128.686 , 111.142 , 94.899 , 82.870 , 0.000 , -36.516 , -39.052 , -39.546 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 68.063 , 130.392 , 158.501 , 94.239 , 108.891 , 100.108 , 60.286 , 59.494 , 37.754 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 130.219 , 133.186 , 126.931 , 0.000 , 115.267 , 144.152 , 61.477 , 37.846 , 0.000 , 0.000 , -43.321 , 0.000 , -55.030 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 105.891 , 86.782 , 82.610 , 82.945 , 0.000 , 0.000 , 108.709 , 71.302 , 23.276 , 82.357 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , -1.771 , 1.243 , -31.746 , -18.377 , 60.543 , 0.000 , -4.833 , 8.424 , 0.000 , 40.942 , 10.262 , 0.000 , 114.890 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , -131.895 , -147.537 , -129.210 , -75.091 , 19.279 , 10.617 , 7.312 , 30.712 , 14.943 , 0.000 , 0.000 , 0.000 , -172.218 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 52.731 , -4.742 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000;
    mockCellData.setZero();
    mockCellData.col(0) = tmp;

    std::cout << "Mock map\n";



    MockMap map3;
    map3.initialise(100, 19, 12, mockCellData);

    std::cout << map3 << "\n";

//    std::cout << map3.cellData << "\n";
    map3.printMapValues(false);

    int startIdx = map3.getIndex(0,9);
    int goalIdx = map3.getIndex(18,9);
    std::vector<int> path;
    bool res = PathPlanning::a_star(map3, startIdx, goalIdx, path);


    std::cout << "Planning result " << res << ", for " << startIdx << " to " << goalIdx << "\n";
    if (res) {
        Eigen::Vector2i p;
        for (auto const &e: path) {
            map3.getXY(e, p);
            std::cout << e << ": " << p.transpose() << " : " << map3.cellData(e, GridMap::MEAN) << "\n";
        }
    }


    std::cout << "startIdx " << startIdx << "\n";
    int neighbours[8];
    map3.getAllNeighbours(startIdx, neighbours);
    for(int i=0;i<8;i++) std::cout << neighbours[i] << "\n";





}