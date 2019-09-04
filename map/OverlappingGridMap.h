#ifndef OVERLAPPINGGRIDMAP_H
#define OVERLAPPINGGRIDMAP_H

#include<vector>
#include <Eigen/Dense>
#include "GridMap.h"

// Conceptually this map isn't really any different to the standard GridMap class
//  except that when stats are calculated, points from a larger area than the cell itself are used

// like GridMap, there is a cell of size stepSize*stepSize
//  all points within this area belong to that cell
//  but the cell also has cellCoverageStep which defines a larger (concentric) area around the cell
//  and any point in this area cellCoverageStep*cellCoverageStep will contribute to the cell statistic


// The bounding box and resolution will initially be calculated as if the grid centres go to the bounding edge of the input point cloud
//  but this is just so all points can be assigned a cell for calculations
//  the cells on the edge (whose overlap region extends outside the point cloud boundary) will be removed at the end
// TODO actually might just leave them for the moment, and accept that they will be based on a smaller area



class OverlappingGridMap : public GridMap {
public:
    // need to modify the initialise functions
    // assignPointToCell
    //      with each point going to multiple cells, could get expensive
    //      helps if we know how many cells a point could go into

    // will the bounding box be extended to include the full range of the cells on the edge?
    //   or reduced so that the 'standard' BB is the same as the outermost cell edges?
    // Neither exactly. The BB will encompass the cell 'centres' only - this will preserve all main indexing logic etc
    // The cells on the edges will go over the edge of the recorded bounding box.
    // This does mean that the recorded BB will not be the point cloud BB

    // how to determine in which cell a point is? same as before, and it's basically whichever cell the point is most central for


    float cellCoverageFactor;
    std::vector<std::pair<int, int>> cellPointMap;

    // These just give something equivalent to GridMap
    OverlappingGridMap(float stepSizeIn, const Eigen::MatrixXf& pcd);
    OverlappingGridMap(float stepSizeIn, const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd);
    OverlappingGridMap(float stepSizeIn, const Eigen::RowVector3f& robotPos, float limit, const Eigen::MatrixXf& pcd);

    // These give the actual overlapping map
    OverlappingGridMap(float stepSizeIn, int cellCoverageFactorIn, const Eigen::MatrixXf& pcd);
    OverlappingGridMap(float stepSizeIn, int cellCoverageFactorIn, const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd);
    OverlappingGridMap(float stepSizeIn, int cellCoverageFactorIn, const Eigen::RowVector3f& robotPos, float limit, const Eigen::MatrixXf& pcd);

    bool initialise(float stepSizeIn, int cellCoverageFactorIn, const Eigen::MatrixXf& pcd);
    bool initialise(float stepSizeIn, int cellCoverageFactorIn, const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd);
    bool initialise(float stepSizeIn, int cellCoverageFactorIn, const Eigen::RowVector3f& robotPos, float limit, const Eigen::MatrixXf& pcd);


//    void setBoundingBox(const Eigen::MatrixXf& pcd);
    void assignPointsToCell(const Eigen::MatrixXf& pcd);
    void calculateCellStatistics(const Eigen::MatrixXf& pcd);


    // new function
    // could template the class to get a fixed overlap instead of using vector
    void getNRingNeighbours(int cellIdx, int nRing, std::vector<int>& neighbourIdxs);



//    GridMap();
//    GridMap(const float stepSizeIn, const Eigen::MatrixXf& pcd);
//    GridMap(const float stepSizeIn, const Eigen::RowVector3f robotPos, const Eigen::MatrixXf& pcd);
//    GridMap(float stepSizeIn, const Eigen::RowVector3f robotPos, const float limit, const Eigen::MatrixXf& pcd);
//    bool initialise(const float stepSizeIn, const Eigen::MatrixXf& pcd);
//    bool initialise(const float stepSizeIn, const Eigen::RowVector3f robotPos, const Eigen::MatrixXf& pcd);
//    bool initialise(float stepSizeIn, const Eigen::RowVector3f robotPos, const float limit, const Eigen::MatrixXf& pcd);

    // All these functions will be the same
//    int getIndex(const Eigen::Vector2i& p) const;
//    int getIndex(const int x, const int y) const;
//    int getIndex(const float x, const float y) const;
//    int getIndexUnsafe(const Eigen::Vector2i& p) const;
//    int getIndexUnsafe(const int x, const int y) const;
//    int getIndexUnsafe(const float x, const float y) const;
//    void getXY(const int idx, Eigen::Vector2i& p) const;
//    void getCellCentre(const int idx, Eigen::Vector2f& cc) const;
//    int getAllNeighbours(const int idx, int neighbourIdxs[]) const;
//    void dropPointData();
//    float getTransitionCost(const int fromIdx, const int toIdx) const;
//    void printMapValues(bool flip = true);

//    void setResolution(const Eigen::MatrixXf& pcd);
//    void adjustBoundingBoxForRobot(const Eigen::RowVector3f robotPos);


    // But these (may) need to be modified

    //    GridMap();
//    GridMap(const float stepSize, const Eigen::MatrixXf& pcd);
//    GridMap(const float stepSize, const Eigen::RowVector3f robotPos, const Eigen::MatrixXf& pcd);
//    GridMap(float stepSizeIn, const Eigen::RowVector3f robotPos, const float limit, const Eigen::MatrixXf& pcd);
//    bool initialise(const float stepSizeIn, const Eigen::MatrixXf& pcd);
//    bool initialise(const float stepSizeIn, const Eigen::RowVector3f robotPos, const Eigen::MatrixXf& pcd);
//    bool initialise(float stepSizeIn, const Eigen::RowVector3f robotPos, const float limit, const Eigen::MatrixXf& pcd);

    //    void trimPointCloud(const float limit, const Eigen::RowVector3f robotPos,
//                        const Eigen::MatrixXf& pcd, Eigen::MatrixXf& trimmedPcd);

//    void setBoundingBox(const Eigen::MatrixXf& pcd);
//    void assignPointsToCell(const Eigen::MatrixXf& pcd);
//    void calculateCellStatistics(const Eigen::MatrixXf& pcd);
//    void calculateCellNormals(const Eigen::MatrixXf& pcd);
//    void calculateLocalNormals(); // probably remove
//    void calculateUDP(); // probably remove



};


#endif
