#ifndef MOCKMAP_H
#define MOCKMAP_H

#include"GridMap.h"

class MockMap : public GridMap {
public:
    bool initialise(const float stepSizeIn, const int resXIn, const int resYIn, const Eigen::MatrixXf& cellDataIn);

};

#endif