#include "MockMap.h"

bool MockMap::initialise(const float stepSizeIn, const int resXIn, const int resYIn, const Eigen::MatrixXf& cellDataIn) {
    imputationRun = false;

    stepSize = stepSizeIn;
    cellData = cellDataIn;
    numPoints = cellData.rows();
    bbMin = Eigen::RowVector3f(-1,-1,-1);
    bbMax = Eigen::RowVector3f(1,1,1);
    bbRange = bbMax - bbMin;
    numCells = cellData.rows();
    resX = resXIn;
    resY = resYIn;
    cellNormals.resize(numCells, 3);
    cellNormals.setZero();
    cellPointCount.resize(numCells);
    cellPointCount.setOnes();

    return true;
}
