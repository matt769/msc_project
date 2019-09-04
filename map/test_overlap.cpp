#include"OverlappingGridMap.h"
#include <Eigen/Dense>
#include <iostream>
#include <string.h>


int main(int argc, char *argv[]) {
    int numPoints = 10; // really doing this to prevent any kind of static allocation of mockPcd (although I don't know if it actually does)
    int numSteps = 10;
    if(argc == 3){
        numPoints = std::stoi(argv[1]);
        numSteps = std::stoi(argv[2]);
    }



    Eigen::MatrixXf mockPcd = Eigen::MatrixXf::Random(numPoints,3);
    mockPcd.col(1) *= 1.5;
    float stepSize = (mockPcd.maxCoeff() - mockPcd.minCoeff()) / (float)numSteps;

    OverlappingGridMap map(stepSize, mockPcd);
    std::cout << map;
//    std::cout << map.cellData;
    std::cout << map.bbMin << "\n";
    std::cout << map.bbMax << "\n";
    std::cout << map.bbRange << "\n";
//    std::cout << map.cellNormals << "\n";

    std::cout << "\n";
    if(numPoints < 15) {
        map.printMapValues(false);
    }


    std::cout << " Now try an actual overlapping map.\n";
    int cellAreaOverlapFactor = 1;
    OverlappingGridMap map2(stepSize, cellAreaOverlapFactor, mockPcd);
    std::cout << map2;
//    std::cout << map.cellData;
    std::cout << map2.bbMin << "\n";
    std::cout << map2.bbMax << "\n";
    std::cout << map2.bbRange << "\n";
//    std::cout << map.cellNormals << "\n";


    // I'm suspicious of these values
    if(numPoints < 15) {
        map2.printMapValues(false);
    }







}
