#include"GridMap.h"
#include "MockMap.h"
#include <Eigen/Dense>
#include <iostream>
#include <string.h>

// create some random point clouds
// test how long processing takes
// how does this scale with map size?
// how does this scale with resolution?



int main(int argc, char *argv[]) {
    int numPoints = 100; // really doing this to prevent any kind of static allocation of mockPcd (although I don't know if it actually does)
    int numSteps = 10;
    if(argc == 3){
        numPoints = std::stoi(argv[1]);
        numSteps = std::stoi(argv[2]);
    }



    Eigen::MatrixXf mockPcd = Eigen::MatrixXf::Random(numPoints,3);
    mockPcd.col(1) *= 1.5;
    float stepSize = (mockPcd.maxCoeff() - mockPcd.minCoeff()) / (float)numSteps;
    GridMap map(stepSize, mockPcd);
    std::cout << map;
//    std::cout << map.cellData;
    std::cout << map.bbMin << "\n";
    std::cout << map.bbMax << "\n";
    std::cout << map.bbRange << "\n";
//    std::cout << map.cellNormals << "\n";



    int neighbours[8];
    int countneighbours;

    countneighbours = map.getAllNeighbours(0, neighbours);
    std::cout << "countNeighbours of idx 0: " << countneighbours << "\n";
    for(int i=0; i<8; i++){
        std::cout << neighbours[i] << "\n";
    }

    countneighbours = map.getAllNeighbours(50, neighbours);
    std::cout << "countNeighbours of idx 50: " << countneighbours << "\n";
    for(int i=0; i<8; i++){
        std::cout << neighbours[i] << "\n";
    }

    countneighbours = map.getAllNeighbours(1000, neighbours);
    std::cout << "countNeighbours of idx 1000: " << countneighbours << "\n";
    for(int i=0; i<8; i++){
        std::cout << neighbours[i] << "\n";
    }

    for(int i=0; i<map.numCells; i++) {
        Eigen::Vector2i p;
        map.getXY(i, p);
        int newI;
        newI = map.getIndex(p);
        Eigen::Vector2f cellCentre;
        map.getCellCentre(i, cellCentre);


//        if(i != newI) {
        std::cout << "Index in: " << i
                  << ", xy positions: " << p.transpose()
                  << ", index out: " << newI
                  << ", cell centre: " << cellCentre(0) << ", " << cellCentre(1) << "\n";

//        }
    }

    std::cout << "Transition cost:\n";
    float tcost = map.getTransitionCost(0,1);
    std::cout << map.cellData(0, 0) << "\t" << map.cellData(1, 0) << "\t" << tcost << "\n";



    // or create an empty map object first and then initialise
//    GridMap map2;
//    map2.initialise(stepSize, mockPcd);
//    std::cout << map2;
//
//    GridMap map3;
//    map3 = GridMap(stepSize, mockPcd);
//    std::cout << map3;

//    Eigen::RowVector3f robotPos(1,-1,1.5);
//    GridMap map4(stepSize, robotPos, mockPcd);
//    std::cout << map4;


//    Eigen::Vector2f cellCentre;
//    int cIdx = 0;
//    map.getCellCentre(cIdx, cellCentre);
//    std::cout << "Centre of cell " <<  cIdx << ": " << cellCentre(0) << ", " << cellCentre(1) << "\n";
//
//    cIdx = 20;
//    map.getCellCentre(cIdx, cellCentre);
//    std::cout << "Centre of cell " <<  cIdx << ": " << cellCentre(0) << ", " << cellCentre(1) << "\n";
//
//    cIdx = 80;
//    map.getCellCentre(cIdx, cellCentre);
//    std::cout << "Centre of cell " <<  cIdx << ": " << cellCentre(0) << ", " << cellCentre(1) << "\n";


//    GridMap map5;
//    float limit = 1.0;
//    map5.initialise(stepSize, robotPos, limit, mockPcd);
//    std::cout << map5;
//
//    GridMap map6;
//    robotPos = Eigen::RowVector3f(0,0,0);
//    map6.initialise(stepSize, robotPos, limit, mockPcd);
//    std::cout << map6;

    numPoints = 10;
    Eigen::MatrixXf mockPcd2 = Eigen::MatrixXf::Random(numPoints,3);
    float stepSize2 = 0.2;
    GridMap map2(stepSize2, mockPcd);
    std::cout << map2;
//    std::cout << map.cellData;
    std::cout << map2.bbMin << "\n";
    std::cout << map2.bbMax << "\n";
    std::cout << map2.bbRange << "\n";
//    std::cout << map.cellNormals << "\n";


//    map.printMapValues();
//    std::cout << "\n";
//    map.printMapValues(false);

    Eigen::MatrixXf mockCellData(228,5);
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

//    std::cout << map3.cellData << "\n";
    map3.printMapValues(false);

    int x = 0;
    int y = 9;
    std::cout << "start XY " << x << ", " << y << "\n";
    int startIdx = map3.getIndex(0,9);
//    int goalIdx = map3.getIndex(18,9);
    std::cout << "startIdx " << startIdx << "\n";
    int neighbours2[8];
    map3.getAllNeighbours(startIdx, neighbours2);
    for(int i=0;i<8;i++) std::cout << neighbours2[i] << "\n";

    Eigen::Vector2i p;
    map3.getXY(startIdx, p);
    std::cout << "XY " << p.transpose() << "\n";
    int newX = p(0) + 1;
    int newY = p(1);
    std::cout << "new XY " << newX << ", " << newY << "\n";

    int nIdx = map3.getIndexUnsafe(newX, newY);
    std::cout << "nIdx " << nIdx << "\n";


    map.printMapValues("testoutputfile.txt", GridMap::MEAN, false);



    std::cout << "Test imputation\n";
    numPoints = 10;
    numSteps = 6;
    Eigen::MatrixXf mockPcd4 = Eigen::MatrixXf::Random(numPoints,3);
    stepSize = (mockPcd.maxCoeff() - mockPcd.minCoeff()) / (float)numSteps;

    GridMap map4(stepSize, mockPcd4);
    std::cout << map4;
//    std::cout << map4.cellPointCount << "\n";
    std::cout << "Before\n";
    map4.printMapValues(GridMap::MEAN, false);
    std::cout << "Transition costs\n";
    std::cout << map4.getTransitionCost(2,5) << "\n";
    std::cout << map4.getTransitionCost(2,5) << "\n";
    std::cout << map4.getTransitionCost(2,3) << "\n";
    std::cout << map4.getTransitionCost(7,11) << "\n";


    std::cout << "After\n";
    map4.impute(5,2);
    map4.printMapValues(GridMap::MEAN, false);

    std::cout << "Transition costs\n";
    std::cout << map4.getTransitionCost(2,5) << "\n";
    std::cout << map4.getTransitionCost(2,5) << "\n";
    std::cout << map4.getTransitionCost(2,3) << "\n";
    std::cout << map4.getTransitionCost(7,11) << "\n";



    Eigen::MatrixXf mockCellData5(289,5);
    Eigen::VectorXf tmp5(289);

    // Assumes the top line is by the starting area
    // (And the goal is probably on or near the bottom line)

    tmp5 << 9,9,9,9,9,10.125,5.625,4.5,4.5,4.5,4.5,5.625,5.625,10.125,9,7.875,7.875,
            9,9,9,9,9,10.125,5.625,4.5,4.5,4.5,4.5,5.625,5.625,10.125,9,7.875,7.875,
            6.75,7.875,7.875,6.75,6.75,10.125,5.625,4.5,4.5,4.5,4.5,5.625,5.625,10.125,9,7.875,7.875,
            6.75,7.875,7.875,6.75,6.75,10.125,5.625,4.5,4.5,4.5,4.5,5.625,5.625,10.125,9,7.875,7.875,
            6.75,6.75,7.875,7.875,7.875,10.125,5.625,4.5,4.5,4.5,4.5,5.625,5.625,5.625,10.125,7.875,7.875,
            6.75,6.75,7.875,7.875,7.875,9,10.125,5.625,5.625,4.5,4.5,4.5,4.5,5.625,5.625,9,9,
            6.75,6.75,9,9,9,9,9,10.125,5.625,4.5,4.5,4.5,4.5,5.625,5.625,10.125,7.875,
            6.75,6.75,7.875,9,9,7.875,9,10.125,9,5.625,5.625,5.625,4.5,4.5,4.5,5.625,10.125,
            6.75,6.75,7.875,9,7.875,7.875,9,10.125,9,5.625,5.625,5.625,4.5,4.5,4.5,5.625,10.125,
            6.75,6.75,9,9,7.875,7.875,9,10.125,9,7.875,5.625,5.625,4.5,4.5,4.5,5.625,10.125,
            6.75,6.75,10.125,9,7.875,7.875,9,10.125,9,5.625,4.5,4.5,4.5,5.625,5.625,10.125,7.875,
            6.75,6.75,9,7.875,7.875,7.875,9,10.125,9,5.625,4.5,4.5,4.5,5.625,5.625,10.125,7.875,
            6.75,6.75,7.875,7.875,7.875,7.875,9,9,5.625,5.625,4.5,4.5,4.5,5.625,5.625,10.125,7.875,
            6.75,6.75,7.875,7.875,7.875,7.875,9,9,5.625,5.625,4.5,4.5,4.5,5.625,5.625,9,7.875,
            6.75,6.75,7.875,7.875,9,9,6.75,5.625,4.5,4.5,4.5,5.625,5.625,5.625,9,7.875,7.875,
            6.75,6.75,9,7.875,9,9,6.75,5.625,4.5,4.5,4.5,5.625,5.625,9,7.875,7.875,7.875,
            6.75,6.75,9,7.875,9,9,6.75,5.625,4.5,4.5,4.5,5.625,5.625,9,7.875,7.875,7.875;
    mockCellData5.setZero();
    mockCellData5.col(0) = tmp5;
//    std::cout << mockCellData.col(0) << "\n";

    std::cout << "Mock map\n";
    MockMap map5;
    map5.initialise(4.5, 17, 17, mockCellData);
    //map5.impute(2,1);
    std::cout << "Transition costs\n";
    std::cout << map5.getTransitionCost(2,5) << "\n";
    std::cout << map5.getTransitionCost(2,5) << "\n";
    std::cout << map5.getTransitionCost(2,3) << "\n";
    std::cout << map5.getTransitionCost(7,11) << "\n";


    std::cout << map.getTransitionCost(7, 11) << "\n";
    std::cout << map.getTransitionCost(7, 11, GridMap::STAT::STAT_MEAN, GridMap::COST_TYPE::ABS_CHANGE) << "\n";
    std::cout << map.getTransitionCost(7, 11, GridMap::STAT::STAT_STDEV, GridMap::COST_TYPE::ABS_CHANGE) << "\n";
    std::cout << map.getTransitionCost(7, 11, GridMap::STAT::STAT_MIN, GridMap::COST_TYPE::ABS_CHANGE) << "\n";
    std::cout << map.getTransitionCost(7, 11, GridMap::STAT::STAT_MAX, GridMap::COST_TYPE::ABS_CHANGE) << "\n";
    std::cout << map.getTransitionCost(7, 11, GridMap::STAT::STAT_RANGE, GridMap::COST_TYPE::ABS_CHANGE) << "\n";


//    std::cout << "End of main\n";
}
