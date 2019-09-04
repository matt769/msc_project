#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <Eigen/Dense>
#include <vector>


// TODO consider storage order

class GridMap {
public:
    // use these for indexing
    static const int MEAN = 0;
    static const int STDEV = 1;
    static const int MIN = 2;
    static const int MAX = 3;
    static const int RANGE = 4;

    static const int X = 0;
    static const int Y = 1;
    static const int Z = 2;


    // some duplication here
    // at some point go back and remove/change the static consts
    enum STAT {
        STAT_MEAN = 0,
        STAT_STDEV = 1,
        STAT_MIN = 2,
        STAT_MAX = 3,
        STAT_RANGE = 4
    };

    enum COST_TYPE {
        ABS_CHANGE,
        CHANGE,
        TO_VALUE
    };



    // start off by leaving space for mean, stdev, number of points
    // later might want to expand to hold slope (normal) and other stuff
    typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> MatrixX3_RM;
    Eigen::Matrix<float, Eigen::Dynamic, 5> cellData;
//    MatrixX3_RM mapPoints;
    MatrixX3_RM cellNormals;
    Eigen::MatrixXf centroid;
    Eigen::VectorXi pointCellIdx;
    Eigen::VectorXi cellPointCount;
    Eigen::Matrix<bool, Eigen::Dynamic, 1>  imputedInd;
//    Eigen::Matrix<bool, Eigen::Dynamic, 1>  notTraversible;
    float stepSize;
    int resX; // number of cells along X axis
    int resY; // number of cells along Y axis
    int numCells;
    int numPoints;
    Eigen::RowVector3f bbMin;
    Eigen::RowVector3f bbMax;
    Eigen::RowVector3f bbRange;
    bool imputationRun;

    //GridMap(float stepSize, Eigen::RowVector3d bbMin, Eigen::RowVector3d bbMax);

    // calculate bounding box and required grid, then assign points to cells
    // should I copy pcd or use whatever was provided (assuming it won't go out of scope)?
    // make a copy here for now
    GridMap();
    GridMap(float stepSize, const Eigen::MatrixXf& pcd);
    GridMap(float stepSize, const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd);
    GridMap(float stepSizeIn, const Eigen::RowVector3f& robotPos, float limit, const Eigen::MatrixXf& pcd);
    bool initialise(float stepSizeIn, const Eigen::MatrixXf& pcd);
    bool initialise(float stepSizeIn, const Eigen::RowVector3f& robotPos, const Eigen::MatrixXf& pcd);
    bool initialise(float stepSizeIn, const Eigen::RowVector3f& robotPos, float limit, const Eigen::MatrixXf& pcd);

    int getIndex(const Eigen::Vector2i& p) const;
    int getIndex(int x, int y) const;
    int getIndex(float x, float y) const;
    int getIndexUnsafe(const Eigen::Vector2i& p) const;
    int getIndexUnsafe(int x, int y) const;
    int getIndexUnsafe(float x, float y) const;
    void getXY(int idx, Eigen::Vector2i& p) const;
    void getCellCentre(int idx, Eigen::Vector2f& cc) const;
    int getAllNeighbours(int idx, int neighbourIdxs[]) const;
    void trimPointCloud(float limit, const Eigen::RowVector3f robotPos,
                                 const Eigen::MatrixXf& pcd, Eigen::MatrixXf& trimmedPcd);
    void setResolution(const Eigen::MatrixXf& pcd);
    void setBoundingBox(const Eigen::MatrixXf& pcd);
    void adjustBoundingBoxForRobot(const Eigen::RowVector3f robotPos);
    void assignPointsToCell(const Eigen::MatrixXf& pcd);
    void calculateCellStatistics(const Eigen::MatrixXf& pcd);

    void impute(int nCellsReqInterior, int nCellsReqExterior);
    bool isOnEdge(int cellIdx);
    void handleMissingValues();
    void calculateCellCentroid(const Eigen::MatrixXf& pcd);
    void calculateCellNormals(const Eigen::MatrixXf& pcd);
    void calculateLocalNormals(); // probably remove
    void calculateUDP(); // probably remove
//    void dropPointData();
    float getTransitionCost(int fromIdx, int toIdx) const;
    float getTransitionCost(const int fromIdx, const int toIdx, STAT stat, COST_TYPE cost) const;
    void printMapValues(bool flip = true);
    void printMapValues(int stat, bool flip = true);
    void printMapValues(std::string filename, int stat, bool flip);
    bool isValidIndex(int idx);
    bool isValidXY(const Eigen::Vector2i& p);
    bool isValidXY(int xIdx, int yIdx);


    // for visualisation only
    // create grid vertices

};

std::ostream& operator<<(std::ostream& os, const GridMap& map);

#endif
