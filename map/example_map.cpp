#include"GridMap.h"
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
    float stepSize = (mockPcd.maxCoeff() - mockPcd.minCoeff()) / (float)numSteps;
    GridMap map(stepSize, mockPcd);
    std::cout << map;

    map.printMapValues();





}
