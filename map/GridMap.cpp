#include"GridMap.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>


// TODO add flags to control which stats actually get created
// And probably split cellData into vectors so don't use the full space if not required


// NOTE there is an implicit grid structure
// Imagine that grid cells are created moving along X axis and then up one step in Y when reach end of row


GridMap::GridMap() = default;



// until scale is handled properly, may need to calculate required stepsize for point cloud before calling function
GridMap::GridMap(const float stepSizeIn, const Eigen::MatrixXf& pcd) {
    initialise(stepSizeIn, pcd);

}

GridMap::GridMap(const float stepSizeIn, const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd) {
    initialise(stepSizeIn, robotPos, pcd);

}

GridMap::GridMap(const float stepSizeIn, const Eigen::RowVector3f& robotPos, const float limit, const Eigen::MatrixXf& pcd) {
    initialise(stepSizeIn, robotPos, limit, pcd);

}


bool GridMap::initialise(const float stepSizeIn, const Eigen::MatrixXf& pcd) {
    imputationRun = false;

    stepSize = stepSizeIn;
    numPoints = pcd.rows();
    setBoundingBox(pcd);
    setResolution(pcd);
    assignPointsToCell(pcd);
    calculateCellStatistics(pcd);
    calculateCellCentroid(pcd);
    calculateCellNormals(pcd);

    return true;
}

// this also take the robot position and extends the grid to ensure the robot is included
bool GridMap::initialise(const float stepSizeIn, const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd) {
    imputationRun = false;

    stepSize = stepSizeIn;
    numPoints = pcd.rows();
    setBoundingBox(pcd);
    adjustBoundingBoxForRobot(robotPos);
    setResolution(pcd);
    assignPointsToCell(pcd);
    calculateCellStatistics(pcd);
    calculateCellCentroid(pcd);
    calculateCellNormals(pcd);

    return true;
}

bool GridMap::initialise(const float stepSizeIn, const Eigen::RowVector3f& robotPos, const float limit, const Eigen::MatrixXf& pcd) {
    imputationRun = false;

    stepSize = stepSizeIn;
    Eigen::MatrixXf trimmedPcd;
    trimPointCloud(limit, robotPos, pcd, trimmedPcd);
    numPoints = trimmedPcd.rows();
    std::cout << numPoints << "\n";
    setBoundingBox(trimmedPcd);
    adjustBoundingBoxForRobot(robotPos);
    setResolution(trimmedPcd);
    assignPointsToCell(trimmedPcd);
    calculateCellStatistics(trimmedPcd);
    calculateCellCentroid(pcd);
    calculateCellNormals(trimmedPcd);

    return true;
}


void GridMap::trimPointCloud(const float limit, const Eigen::RowVector3f robotPos,
                             const Eigen::MatrixXf& pcd, Eigen::MatrixXf& trimmedPcd) {
//    std::cout << "Limit: " << limit << "\n";
    trimmedPcd.resize(pcd.rows(), 3);
//    float limitSq = limit*limit;
    int trimmedIdx = 0;
    for(int idx = 0; idx< pcd.rows(); idx++) {
//        std::cout << "Point " << idx << ": " << pcd.row(idx) << ", distance " << (robotPos - pcd.row(idx)).squaredNorm() << "\n";
//        if((robotPos - pcd.row(idx)).squaredNorm() < limitSq) {
//            trimmedPcd.row(trimmedIdx) = pcd.row(idx);
//            trimmedIdx++;
//        }

        if(std::fabs(robotPos(X) - pcd(idx, X)) < limit && std::fabs(robotPos(Y) - pcd(idx, Y)) < limit) {
            trimmedPcd.row(trimmedIdx) = pcd.row(idx);
            trimmedIdx++;
        }

    }
    trimmedPcd.conservativeResize(trimmedIdx, 3);
//    std::cout << "Orig points: " << pcd.rows() << ", trimmed points: " << trimmedPcd.rows() << "\n";
}





void GridMap::setBoundingBox(const Eigen::MatrixXf& pcd) {
//    std::cout << "Start setBoundingBox\n";
    bbMin = pcd.colwise().minCoeff();
    bbMax = pcd.colwise().maxCoeff();
    bbRange = bbMax - bbMin;

//    std::cout << "End setBoundingBox " << bbRange << "\n";
}

void GridMap::adjustBoundingBoxForRobot(const Eigen::RowVector3f robotPos) {

//    std::cout << "BB\n";
//    std::cout << bbMin << "\n";
//    std::cout << bbMax << "\n";
    bbMin = bbMin.cwiseMin(robotPos);
    bbMax = bbMax.cwiseMax(robotPos);
    bbRange = bbMax - bbMin;
//    std::cout << "BBadj\n";
//    std::cout << bbMin << "\n";
//    std::cout << bbMax << "\n";

}


void GridMap::setResolution(const Eigen::MatrixXf& pcd) {
//    std::cout << "Start setResolution\n";
//    std::cout << "bbRange(X) / stepSize " << bbRange(X) / stepSize << "\n";
    resX = static_cast<int>(bbRange(X) / stepSize) + 1; // add one to make sure the full range is covered
    resY = static_cast<int>(bbRange(Y) / stepSize) + 1;
    numCells = resX*resY;
//    std::cout << "End setResolution " << resX << "," << resY << "," << numCells << "\n";
}


// is this really necessary?
// keep for now but maybe later
// imagine the grid cells are labelled on their lower left corner (min x, min y)
void GridMap::assignPointsToCell(const Eigen::MatrixXf& pcd) {
//    std::cout << "Start assignPointsToCell\n";
    pointCellIdx.resize(numPoints);
    cellPointCount = Eigen::VectorXi::Zero(numCells);

    for (int pIdx = 0; pIdx < numPoints; pIdx++)
    {
        Eigen::RowVector3f point = pcd.row(pIdx);
        point -= bbMin;
        int xPos = static_cast<int>(point(X) / stepSize);
        int yPos = static_cast<int>(point(Y) / stepSize);
//        std::cout << "End setResolution " << resX << "," << resY << "," << numCells << "\n";
        Eigen::Vector2i pointIdxs{xPos, yPos};
        int cIdx = getIndex(pointIdxs);
        pointCellIdx(pIdx) = cIdx;
        cellPointCount(cIdx)++;
//        std::cout << "pIdx->cIdx: " << pIdx << "\t->\t" << cIdx << "\n";
    }
//    std::cout << "End assignPointsToCell\n";
}

// X, Y grid indicies to index
int GridMap::getIndex(const Eigen::Vector2i& p) const {
//    std::cout << "Start getIndex for " << p.x << "," << p.y << "\n";
    int cellIdx =  p(Y)*(resX) + p(X);

    if (p(X) < 0 || p(X) >= resX || p(Y) < 0 || p(Y) >= resY) {
        std::cout << "Indices (" << p(X) << ", " << p(Y) << ") outside range ("
                  << resX << ", " << resY << ")\n";
        exit(1);
    }

    if (cellIdx >= numCells) {
        std::cout << "Cell index " << cellIdx << "outside range " << numCells << "\n";
        exit(1);
    }
//    std::cout << "End getIndex with " << cellIdx << "\n";
    return cellIdx;
}

// X, Y grid indicies to index
int GridMap::getIndex(const int x, const int y) const {
    Eigen::Vector2i p{x, y};
    return getIndex(p);
}

// X, Y coordinates to index
int GridMap::getIndex(const float x, const float y) const {
    float xAdj = x - bbMin(X);
    float yAdj = y - bbMin(Y);
    int xPos = static_cast<int>(xAdj / stepSize);
    int yPos = static_cast<int>(yAdj / stepSize);
    return getIndex(xPos, yPos);
}


// This will still check if valid but instead of exiting program, will just return -1
//  which of course should not be used as an actual index!
//  i.e. the return value must be checked before being used
int GridMap::getIndexUnsafe(const Eigen::Vector2i& p) const {
//    std::cout << "Start getIndex for " << p.x << "," << p.y << "\n";
    int cellIdx =  p(Y)*(resX) + p(X);

    if (p(X) < 0 || p(X) >= resX || p(Y) < 0 || p(Y) >= resY) {
//        std::cout << "Indices (" << p(X) << ", " << p(Y) << ") outside range ("
//                  << resX << ", " << resY << ")\n";
        cellIdx = -1;
    }

    if (cellIdx >= numCells) {
//        std::cout << "Cell index " << cellIdx << "outside range " << numCells << "\n";
        cellIdx = -1;
    }
//    std::cout << "End getIndex with " << cellIdx << "\n";
    return cellIdx;
}

int GridMap::getIndexUnsafe(const int x, const int y) const {
    Eigen::Vector2i p{x, y};
    return getIndexUnsafe(p);
}

// X, Y coordinates to index
int GridMap::getIndexUnsafe(const float x, const float y) const {
    float xAdj = x - bbMin(X);
    float yAdj = y - bbMin(Y);
    int xPos = static_cast<int>(xAdj / stepSize);
    int yPos = static_cast<int>(yAdj / stepSize);
    return getIndexUnsafe(xPos, yPos);
}


// Index to X, Y grid position
void GridMap::getXY(const int idx, Eigen::Vector2i& p) const {
    p(X) = idx % resX;
    p(Y) = idx / resX; // integer division
}

void GridMap::getCellCentre(const int idx, Eigen::Vector2f& cc) const {
    Eigen::Vector2i cellIdxs;
    getXY(idx, cellIdxs);
    cc(X) = ((float)cellIdxs(X) + 0.5) * stepSize + bbMin(X);
    cc(Y) = ((float)cellIdxs(Y) + 0.5) * stepSize + bbMin(Y);
}



// could do these calcs when assigning points to cells
// to prevent looping through multiple times
// but it looks a bit nicer with them separated
// so keep like this unless a performance issue

// using Welford's algorithm for online calculation
// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
void GridMap::calculateCellStatistics(const Eigen::MatrixXf& pcd) {

//    std::cout << "Start calculateCellStatistics()\n";

    Eigen::VectorXf zAve = Eigen::VectorXf::Zero(numCells);
    Eigen::VectorXf zStDev = Eigen::VectorXf::Zero(numCells);
    Eigen::VectorXi zCount = Eigen::VectorXi::Zero(numCells); // although we already have the final version of this, we need to calculate a running total
    Eigen::VectorXf zM = Eigen::VectorXf::Zero(numCells);
    Eigen::VectorXf zMin = Eigen::VectorXf::Zero(numCells);
    Eigen::VectorXf zMax = Eigen::VectorXf::Zero(numCells);
//    Eigen::VectorXf zRange = Eigen::VectorXf::Zero(numCells); don't need temp var

    // update intermediate grid cell statistics with each map point
    for (int pIdx = 0; pIdx < numPoints; pIdx++)
    {
        int cIdx = pointCellIdx(pIdx);
        // count
        zCount(cIdx) += 1;

        // mean and intermediate values for stdev
        double changeA = pcd(pIdx, Z) - zAve(cIdx); // take the difference before and after the mean is updated
        zAve(cIdx) += changeA / zCount(cIdx); // (new val - prev mean) / num samples
        double changeB = pcd(pIdx, Z) - zAve(cIdx); // take the difference before and after the mean is updated
        zM(cIdx) += changeA*changeB;

        // max
        if(zCount(cIdx) == 1) {
            zMax(cIdx) = pcd(pIdx, Z);
            zMin(cIdx) = pcd(pIdx, Z);
        }
        else {
            if(pcd(pIdx, Z) > zMax(cIdx)) {
                zMax(cIdx) = pcd(pIdx, Z);
            }
            if(pcd(pIdx, Z) < zMin(cIdx)) {
                zMin(cIdx) = pcd(pIdx, Z);
            }
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

    cellData.resize(numCells, 5);
    cellData.col(MEAN) = zAve;
    cellData.col(STDEV) = zStDev;
    cellData.col(MIN) = zMin;
    cellData.col(MAX) = zMax;
    cellData.col(RANGE) = zMax - zMin;

//    std::cout << "End calculateCellStatistics()\n";
}



// fill in all (single value) statistics with some combination (e.g. average) of neighbours
// can add other stats (e.g.) normals later if required
// currently this uses any neighbour, but originally I planned to only use adjacent neighbours
//    leave it like this for now
void GridMap::impute(const int nCellsReqInterior, const int nCellsReqExterior) {
    // to be valid for imputation, a cell on the edge needs nCellsReqExterior non-zero neighbours
    // and a cell not on the edge needs nCellsReqInterior non-zero neighbours
    // nCellsReqExterior must always be <= nCellsReqInterior
    imputedInd.resize(numCells);
    imputedInd.setConstant(false);

    int neighbours[8];

    for(int cIdx = 0; cIdx < numCells; cIdx++) {
        // stats will only be missing if there were less than 2 points falling in that cell
//        std::cout << "\t" << cIdx << ": " << cellPointCount(cIdx) << ", " << cellData(cIdx, MEAN) << "\n";
        if(cellPointCount(cIdx) == 0) {
            // get neighbours
            int numNeighbours = getAllNeighbours(cIdx, neighbours); // some of these neighbours may not have data

//            std::cout << "\tnumNeighbours " << numNeighbours << "\n";

            Eigen::Matrix<float, 1, 5> trackNeighbourVals = Eigen::Matrix<float, 1, 5>::Zero();

            // get values of neighbours (if valid)
            int validStDevCount = numNeighbours; // although we check that neighbour contains points, it still may
                                                    // not have a valid stdev if it only had a single point
            for(int nIdx = 0; nIdx < 8; nIdx++) {
                int nCellIdx = neighbours[nIdx];
//                    std::cout << "\tnIdx " << nIdx << ", nCellIdx " << nCellIdx<< "\n";
                if(nCellIdx >= 0) {
//                        std::cout << "\t\tneighbours[nIdx] >= 0" << "\n";
                    if(cellPointCount(nCellIdx) > 0) {
//                            std::cout << "\t\tcellPointCount(nIdx) > 0" << "\n";
                        trackNeighbourVals += cellData.row(nCellIdx);
                        if (cellPointCount(nCellIdx) == 1) {
//                                std::cout << "\t\tcellPointCount(nIdx) == 1" << "\n";
                            validStDevCount--;
                        }
                    }
                    else {
                        numNeighbours--;
//                            std::cout << "\t\tnumNeighbours--" << "\n";
                    }
                }
            }

            // check if on edge
            bool onEdge = isOnEdge(cIdx);
//            std::cout << "\tonEdge " << onEdge << "\n";
//            std::cout << "\tnumNeighbours with data " << numNeighbours << "\n";
            if(numNeighbours >= nCellsReqInterior || (onEdge && (numNeighbours >= nCellsReqExterior))) {
//                    std::cout << "\tImpute " << "\n";
                // ok to impute
                // for all stats we are just going to take the (weighted average)
                // in theory we could do a more rigorous cal for stdev
                // and there are other options for min/max
                trackNeighbourVals(MEAN) /= numNeighbours;
                trackNeighbourVals(MIN) /= numNeighbours;
                trackNeighbourVals(MAX) /= numNeighbours;
                trackNeighbourVals(STDEV) /= validStDevCount;
                trackNeighbourVals(RANGE) = trackNeighbourVals(MAX) - trackNeighbourVals(MIN);
                cellData.row(cIdx) = trackNeighbourVals;
                imputedInd(cIdx) = true;
            } // else can't impute

        }
    }
    imputationRun = true; // flag that imputedInd can be accessed
}

// assumes it's given a valid index
bool GridMap::isOnEdge(const int cellIdx) {
    Eigen::Vector2i p;
    getXY(cellIdx, p);
    return (p(X) == 0 || p(X) == (resX-1) || p(Y) == 0 || p(Y) == (resY-1));
}


// this sets missing values to something representing non-traversability
// actually this is currently being handled in getTransitionCost()
void GridMap::handleMissingValues() {

}




void GridMap::calculateCellCentroid(const Eigen::MatrixXf& pcd) {

    centroid.resize(numCells, 3);

    for (int pIdx = 0; pIdx < numPoints; pIdx++) {
        int cIdx = pointCellIdx(pIdx);
        centroid.row(cIdx) += pcd.row(pIdx);
//        count(cIdx)++;
    }

    for (int cIdx = 0; cIdx < numCells; cIdx++) {
        if(cellPointCount(cIdx) > 0) {
            centroid.row(cIdx) /= cellPointCount(cIdx);
        }
    }

}




void GridMap::calculateCellNormals(const Eigen::MatrixXf& pcd) {

//    std::cout << "Start calculateCellNormals()\n";

    cellNormals.resize(numCells, 3);
//    Eigen::RowVector3f viewpoint = Eigen::RowVector3f::Zero(); // use to orient normals

    std::vector<Eigen::Matrix<float, 3, 3>> cov(numCells);
//    Eigen::VectorXi count = Eigen::VectorXi::Zero(numCells);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3, 3>> solver;

    // add contribution of each point in the grid cell to the covariance matrix
    for (int pIdx = 0; pIdx < numPoints; pIdx++)
    {
        int cIdx = pointCellIdx(pIdx);
        Eigen::RowVector3f diff = pcd.row(pIdx) - centroid.row(cIdx);
        cov[cIdx] += diff.transpose() * diff;
    }

    // solve for normals
    for (int cIdx = 0; cIdx < numCells; cIdx++) {
//        std::cout << "cIdx: " << cIdx << ", points: " << count(cIdx) << "\n";
//        std::cout << cov[cIdx] << "\n";

        if(cellPointCount(cIdx) > 2) {
            solver.compute(cov[cIdx]);
            if (solver.info() == Eigen::Success) {
//                std::cout << "\n" << solver.eigenvectors() << "\n";
//                std::cout << "\n" << solver.eigenvalues() << "\n";
                // the eigenvectors appear to be in ascending order of eigenvalue
                Eigen::RowVector3f normal = solver.eigenvectors().col(0).transpose();
                // flip if pointing 'wrong' way
                Eigen::RowVector3f up; // TODO is this the correct vector to use? What coord system will the map be in when I run this? optical or NED?
                up << 0,0,1;
                Eigen::RowVector3f upMinusPoint = up - centroid.row(cIdx);
                double tmp = normal[0]*upMinusPoint[0] + normal[1]*upMinusPoint[1] + normal[2]*upMinusPoint[2];
                if(tmp<0) normal = -normal;
                cellNormals.row(cIdx) = normal;
            }
            // else solving failed
            else {
                cellNormals.row(cIdx) << 0.0, 0.0, 0.0;
            }
        }
        // else 0 or 1 points in the cell
        else {
            cellNormals.row(cIdx) << 0.0, 0.0, 0.0;
        }

//        std::cout << "\n" << normals.row(cIdx) << "\n\n";
    }


//    std::cout << "End calculateCellNormals()\n";
}


// Should I include all 8 neighbours but with an indicator when there is no neighbour?
// Or only as many valid neighbours as actually exist?
// And do diagonal neighbours count?
// this actually duplicates the checks done by getIndex - maybe should change something somewhere?
int GridMap::getAllNeighbours(const int idx, int neighbourIdxs[8]) const {
    Eigen::Vector2i p;
    getXY(idx, p);
    int x = p(X);
    int y = p(Y);

    // first 4 are the directly adjacent cells, moving clockwise
    neighbourIdxs[0] = getIndexUnsafe(x+1, y);
    neighbourIdxs[1] = getIndexUnsafe(x, y+1);
    neighbourIdxs[2] = getIndexUnsafe(x-1, y);
    neighbourIdxs[3] = getIndexUnsafe(x, y-1);
    // next 4 are the diagonal cells, moving clockwise
    neighbourIdxs[4] = getIndexUnsafe(x+1, y+1);
    neighbourIdxs[5] = getIndexUnsafe(x-1, y+1);
    neighbourIdxs[6] = getIndexUnsafe(x-1, y-1);
    neighbourIdxs[7] = getIndexUnsafe(x+1, y-1);

    int validNeighbourCount = 0;
    for(int i=0; i<8; i++){
        if(neighbourIdxs[i] >= 0){
            validNeighbourCount++;
        }
    }

    return validNeighbourCount;
}




//void GridMap::dropPointData(){
//    mapPoints.resize(0,0);
//    // set a flag to indicate that this is no longer usable?
//}

// just returning difference in height atm
// bit of a placeholder
// if there are no points in the cell and we didn't impute any value for it,
//  mark as non-traversible by returning a massive cost
float GridMap::getTransitionCost(const int fromIdx, const int toIdx) const {
    float cost;
    bool traversible = true;

    if(cellPointCount(fromIdx) == 0) {
        // NOTE checking imputationRun before trying to access imputedInd
//        std::cout << "a\n";
        if(!imputationRun || (imputationRun && !imputedInd(fromIdx))) {
            traversible = false;
//            std::cout << "b\n";
        }
    }

    if(cellPointCount(toIdx) == 0) {
        // NOTE checking imputationRun before trying to access imputedInd
//        std::cout << "c\n";
        if(!imputationRun || (imputationRun && !imputedInd(toIdx))) {
            traversible = false;
//            std::cout << "d\n";
        }
    }

    if(traversible) {
        cost = std::abs(cellData(fromIdx, MEAN) - cellData(toIdx, MEAN));
//        std::cout << "e\n";
    }
    else {
            cost = 99999.9;
    }
    return cost;
}



float GridMap::getTransitionCost(const int fromIdx, const int toIdx, STAT stat, COST_TYPE costType) const {
    float cost;
    bool traversible = true;

    if(cellPointCount(fromIdx) == 0) {
        // NOTE checking imputationRun before trying to access imputedInd
//        std::cout << "a\n";
        if(!imputationRun || (imputationRun && !imputedInd(fromIdx))) {
            traversible = false;
//            std::cout << "b\n";
        }
    }

    if(cellPointCount(toIdx) == 0) {
        // NOTE checking imputationRun before trying to access imputedInd
//        std::cout << "c\n";
        if(!imputationRun || (imputationRun && !imputedInd(toIdx))) {
            traversible = false;
//            std::cout << "d\n";
        }
    }

    if(traversible) {
        switch(costType) {
            case ABS_CHANGE:
                cost = std::abs(cellData(fromIdx, (int)stat) - cellData(toIdx, (int)stat));
                break;
            case CHANGE:
                cost = cellData(fromIdx, (int)stat) - cellData(toIdx, (int)stat);
                break;
            case TO_VALUE:
                cost = cellData(toIdx, (int)stat);
                break;
            default:
                cost = -1.0;
        }
    }
    else {
        cost = 99999.9;
    }
    return cost;
}


void  GridMap::printMapValues(bool flip) {
    // show the last row first so that x0 y0 appears at the bottom left of the screen
    // start off showing average height, but might want to show other metrics later

    if(flip) {
        for (int rowIdx = resY - 1; rowIdx >= 0; rowIdx--) {
            for (int colIdx = 0; colIdx < resX; colIdx++) {
                int idx = getIndex(colIdx, rowIdx);
                std::cout << std::fixed << std::setprecision(3) << cellData(idx, MEAN) << "\t";
            }
            std::cout << "\n";
        }
    }
    else {
        for (int rowIdx = 0; rowIdx <= resY - 1; rowIdx++) {
            for (int colIdx = 0; colIdx < resX; colIdx++) {
                int idx = getIndex(colIdx, rowIdx);
                std::cout << std::fixed << std::setprecision(3) << cellData(idx, MEAN) << "\t";
            }
            std::cout << "\n";
        }
    }

}


// stat needs to be one of the integers defined in the header e.g. MEAN, STDEV
void  GridMap::printMapValues(int stat, bool flip) {
    // show the last row first so that x0 y0 appears at the bottom left of the screen
    // start off showing average height, but might want to show other metrics later

    if(stat != MEAN && stat != STDEV && stat != MAX && stat != MIN && stat != RANGE) {
        std::cout << "Bad stat index.\n";
        return;
    }

    if(flip) {
        for (int rowIdx = resY - 1; rowIdx >= 0; rowIdx--) {
            for (int colIdx = 0; colIdx < resX; colIdx++) {
                int idx = getIndex(colIdx, rowIdx);
                std::cout << std::fixed << std::setprecision(3) << cellData(idx, stat) << "\t";
            }
            std::cout << "\n";
        }
    }
    else {
        for (int rowIdx = 0; rowIdx <= resY - 1; rowIdx++) {
            for (int colIdx = 0; colIdx < resX; colIdx++) {
                int idx = getIndex(colIdx, rowIdx);
                std::cout << std::fixed << std::setprecision(3) << cellData(idx, stat) << "\t";
            }
            std::cout << "\n";
        }
    }

}



void  GridMap::printMapValues(std::string filename, int stat, bool flip) {
    // show the last row first so that x0 y0 appears at the bottom left of the screen
    // start off showing average height, but might want to show other metrics later

    std::ofstream fileOut(filename);

    if(stat != MEAN && stat != STDEV && stat != MAX && stat != MIN && stat != RANGE) {
        std::cout << "Bad stat index.\n";
        return;
    }

    if(numCells <= 0) {
        std::cout << "No data in map.\n";
        return;
    }

    if(flip) {
        for (int rowIdx = resY - 1; rowIdx >= 0; rowIdx--) {
            for (int colIdx = 0; colIdx < resX; colIdx++) {
                int idx = getIndex(colIdx, rowIdx);
                fileOut << std::fixed << std::setprecision(3) << cellData(idx, stat) << "\t";
            }
            fileOut << "\n";
        }
    }
    else {
        for (int rowIdx = 0; rowIdx <= resY - 1; rowIdx++) {
            for (int colIdx = 0; colIdx < resX; colIdx++) {
                int idx = getIndex(colIdx, rowIdx);
                fileOut << std::fixed << std::setprecision(3) << cellData(idx, stat) << "\t";
            }
            fileOut << "\n";
        }
    }

    fileOut.close();
}




bool GridMap::isValidIndex(const int idx) {
    return !(idx < 0 || idx >= numCells);
}

bool GridMap::isValidXY(const Eigen::Vector2i& p) {
    return !(p(X) < 0 || p(X) >= resX || p(Y) < 0 || p(Y) >= resY);
}

bool GridMap::isValidXY(const int xIdx, const int yIdx) {
    return !(xIdx < 0 || xIdx >= resX || yIdx < 0 || yIdx >= resY);
}





// NON-MEMBER FUNCTIONS

std::ostream& operator<<(std::ostream& os, const GridMap& map) {
    os << "GridMap with" << "\n"
       << "\tPoints: " << map.numPoints << "\n"
       << "\tCells: " << map.numCells << "\n"
       << "\tDimensions: " << map.resX << "," << map.resY << "\n"
       << "\tStepsize: " << map.stepSize << "\n";
    return os;
}
