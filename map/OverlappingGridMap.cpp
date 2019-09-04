#include<vector>
#include <Eigen/Dense>
#include <iostream>
#include "OverlappingGridMap.h"


// keep the previous forms
// if no cellCoverageStep is provided then build the map exactly like GridMap

OverlappingGridMap::OverlappingGridMap(const float stepSizeIn, const Eigen::MatrixXf& pcd)
        : GridMap(stepSizeIn, pcd), cellCoverageFactor(0) {}

OverlappingGridMap::OverlappingGridMap(const float stepSizeIn, const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd)
        : GridMap(stepSizeIn, robotPos, pcd), cellCoverageFactor(0) {}

OverlappingGridMap::OverlappingGridMap(const float stepSizeIn, const Eigen::RowVector3f& robotPos, const float limit, const Eigen::MatrixXf& pcd)
        : GridMap(stepSizeIn, robotPos, limit, pcd), cellCoverageFactor(0) {}


// if we get the cellCoverageStep parameter, then just call the GridMap default parameter, and then initialise using
//      the OverlappingGridMap functions
OverlappingGridMap::OverlappingGridMap(const float stepSizeIn, const int cellCoverageFactorIn, const Eigen::MatrixXf& pcd)
        : GridMap(), cellCoverageFactor(cellCoverageFactorIn)
{
    initialise(stepSizeIn, cellCoverageFactorIn, pcd);
}


OverlappingGridMap::OverlappingGridMap(const float stepSizeIn, const int cellCoverageFactorIn,
                                        const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd)
        : GridMap(), cellCoverageFactor(cellCoverageFactorIn)
{
    initialise(stepSizeIn, cellCoverageFactorIn, robotPos, pcd);
}

OverlappingGridMap::OverlappingGridMap(const float stepSizeIn, const int cellCoverageFactorIn,
                                        const Eigen::RowVector3f& robotPos, const float limit, const Eigen::MatrixXf& pcd)
        : GridMap(), cellCoverageFactor(cellCoverageFactorIn)
{
    initialise(stepSizeIn, cellCoverageFactorIn, robotPos, limit, pcd);
}



bool OverlappingGridMap::initialise(const float stepSizeIn, const int cellCoverageFactorIn, const Eigen::MatrixXf& pcd) {
//    std::cout << "In OverlappingGridMap::initialise\n";
    imputationRun = false;
    stepSize = stepSizeIn;
    numPoints = pcd.rows();
    setBoundingBox(pcd);
    setResolution(pcd);
    assignPointsToCell(pcd);
    calculateCellStatistics(pcd);
//    calculateCellNormals(pcd);

    return true;
}

//// this also take the robot position and extends the grid to ensure the robot is included
bool OverlappingGridMap::initialise(const float stepSizeIn, const int cellCoverageFactorIn,
                                    const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd) {

    stepSize = stepSizeIn;
    numPoints = pcd.rows();
    setBoundingBox(pcd);
    adjustBoundingBoxForRobot(robotPos);
    setResolution(pcd);
    assignPointsToCell(pcd);
    calculateCellStatistics(pcd);
//    calculateCellNormals(pcd);

    return true;
}

bool OverlappingGridMap::initialise(const float stepSizeIn, const int cellCoverageFactorIn,
                                    const Eigen::RowVector3f& robotPos, const float limit, const Eigen::MatrixXf& pcd) {

    stepSize = stepSizeIn;
    Eigen::MatrixXf trimmedPcd;
    trimPointCloud(limit, robotPos, pcd, trimmedPcd);
    numPoints = trimmedPcd.rows();
    setBoundingBox(trimmedPcd);
    adjustBoundingBoxForRobot(robotPos);
    setResolution(trimmedPcd);
    assignPointsToCell(trimmedPcd);
    calculateCellStatistics(trimmedPcd);
//    calculateCellNormals(trimmedPcd);

    return true;
}


// Change back to using default
//void OverlappingGridMap::setBoundingBox(const Eigen::MatrixXf& pcd) {
//    std::cout << "Start OverlappingGridMap::setBoundingBox\n";
//    bbMin = pcd.colwise().minCoeff();
//    bbMax = pcd.colwise().maxCoeff();
//    // constrict (increase min, decrease max) to account for overlap
//    float adj = (cellCoverageFactor-1)*stepSize/2.0;
//    Eigen::RowVector3f adjBB{adj, adj, 0};
//    bbMin += adjBB;
//    bbMax -= adjBB;
//    bbRange = bbMax - bbMin;
//    std::cout << "End OverlappingGridMap::setBoundingBox " << bbRange << "\n";
//}



void OverlappingGridMap::getNRingNeighbours(int cellIdx, int nRing, std::vector<int>& neighbourIdxs) {

    neighbourIdxs.clear();
    int numCellsOnSide = 2 * nRing + 1;
//    neighbourIdxs.resize(numCellsOnSide*numCellsOnSide);

    Eigen::Vector2i p;
    getXY(cellIdx, p);
    // get min corner of overlap area in terms of the cell at that corner
    //  (may not be a valid one, in which case row/col idx < 0)
    p(X) -= nRing;
    p(Y) -= nRing;

    // could either work with these XY coords, or go back to index
    // let's use index, since we can work out all the other indices directly rather than converting all from XY
//    int baseCellIdx = getIndexUnsafe(p);
//    for(int relYIdx = 0; relYIdx < numCellsOnSide; relYIdx++) {
//        if(p(Y)+relYIdx < 0 || p(Y)+relYIdx >= resY) continue; // skip whole row
//        for(int relXIdx = 0; relXIdx < numCellsOnSide; relXIdx++) {
//            if(p(X)+relXIdx < 0 || p(X)+relXIdx >= resX) continue; // skip cell
//            neighbourIdxs.push_back(baseCellIdx + relXIdx);
//        }
//        baseCellIdx += resX; // move up 1 row (if 'row' is along x axis)
//    }


    // need to handle the edges differently since there won't be a valid cell available
    // it may be simpler to work with XY because then can just go outside the valid range

    for(int yIdx = p(Y); yIdx < p(Y) + numCellsOnSide; yIdx++) {
        if(yIdx < 0 || yIdx >= resY) continue; // skip whole row
        for(int xIdx = p(X); xIdx < p(X) + numCellsOnSide; xIdx++) {
            if(xIdx < 0 || xIdx >= resX) continue; // skip cell
            neighbourIdxs.push_back(getIndexUnsafe(xIdx,yIdx));
        }
    }


//    std::cout << "cellIdx " << cellIdx << "\n";
//    for(int nIdx: neighbourIdxs) {
//        std::cout << "\tnIdx " << nIdx << "\n";
//    }

    // I think that all results should be valid
}


void OverlappingGridMap::assignPointsToCell(const Eigen::MatrixXf& pcd) {

//    std::cout << "Start OverlappingGridMap::assignPointsToCell()\n";

    cellPointMap.reserve(numPoints);

    // Because the BB has been reduced, we will now get points outside the valid grid
    // TODO how to deal?
    //  this is actually quite awkward
    // Would it have been easier to keep the original bounding box
    //  assign cells normally
    // and then shrink the grid

    for(int pointIdx = 0; pointIdx < numPoints; pointIdx++) {
        int cellIdx = getIndexUnsafe(pcd(pointIdx,X), pcd(pointIdx,Y));
//        cellPointMap[pointIdx] = std::pair<int,int>{cellIdx, pointIdx}; // seems can't do this
        cellPointMap.push_back(std::pair<int,int>{cellIdx, pointIdx});
//        std::cout << "cellIdx " << cellIdx << ", pointIdx " << pointIdx << "\n";
    }

    auto compare = [](const std::pair<int, int>& a ,const std::pair<int, int>& b) {return a.first < b.first; };

    std::sort(cellPointMap.begin(), cellPointMap.end(), compare);

//    std::cout << "End OverlappingGridMap::assignPointsToCell()\n";
//    std::cout << "cellPointMap size " << cellPointMap.size() << "\n";
//    for(auto i: cellPointMap) {
//        std::cout << i.first << ", " << i.second << "\n";
//    }
}


void OverlappingGridMap::calculateCellStatistics(const Eigen::MatrixXf& pcd) {

//    std::cout << "Start OverlappingGridMap::calculateCellStatistics()\n";

    Eigen::VectorXf zAve = Eigen::VectorXf::Zero(numCells);
    Eigen::VectorXf zStDev = Eigen::VectorXf::Zero(numCells);
    Eigen::VectorXi zCount = Eigen::VectorXi::Zero(numCells); // although we already have the final version of this, we need to calculate a running total
    Eigen::VectorXf zM = Eigen::VectorXf::Zero(numCells);

    // cellIdx is ordered, so only get the neighbouring cells when we reach a new range(else use previous list)
    std::vector<int> neighbourIdxs; // note that getNRingNeighbours will clear and size this as required
    int lastCellIdx = -1;
    for (auto cellPointPair: cellPointMap) {
        int cellIdx = cellPointPair.first;
        int pointIdx = cellPointPair.second;
        if(cellIdx != lastCellIdx) {
            getNRingNeighbours(cellIdx, cellCoverageFactor, neighbourIdxs);
        }
        // now perform the operation for each cell that overlaps the 'central' one for this point
        // TODO confirm safe to use these indicies without checking
        for(auto nIdx: neighbourIdxs) {
            if(!isValidIndex(nIdx)) {
                std::cout << "invalid index: " << nIdx << "\n";
            }
            zCount(nIdx) += 1;
            double changeA = pcd(pointIdx, Z) - zAve(nIdx); // take the difference before and after the mean is updated
            zAve(nIdx) += changeA / zCount(nIdx); // (new val - prev mean) / num samples
            double changeB = pcd(pointIdx, Z) - zAve(nIdx); // take the difference before and after the mean is updated
            zM(nIdx) += changeA*changeB;
        }
    }


    // And now calculate the final statistics
    for(int cIdx = 0; cIdx < zAve.rows(); cIdx++)
    {
        int count = zCount(cIdx);
        if(count == 0)
        {
            zAve(cIdx) = 0;

        }
        // Note that for cells with 0 or 1 points, the standard deviation will be returned as zero
        // which isn't technically accurate, better to give a NaN (but I had some funny errors when trying to do this)
        if(count > 1)
        {
            zStDev(cIdx) =  sqrt(zM(cIdx) / (count - 1));
        }

    }

    cellData.resize(numCells, 3);
    cellData.col(MEAN) = zAve;
    cellData.col(STDEV) = zStDev;

//    std::cout << "End OverlappingGridMap::calculateCellStatistics()\n";
}








