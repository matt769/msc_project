#ifndef A_STAR_H
#define A_STAR_H

#include"GridMap.h"
#include<vector>

namespace PathPlanning {


    enum Heuristic {
        ZERO,
        DIAGONAL,
        EUCLIDEAN
    };


    enum Result {
        SUCCESS,
        FAILURE,    // e.g. no path found
        ERROR       // shouldn't get this
    };

    bool a_star(const GridMap& map, int startIdx, int goalIdx, std::vector<int>& path);
    bool a_star(const GridMap& map, int startIdx, int goalIdx, Heuristic hType, std::vector<int>& path);
    bool a_star(const GridMap& map, int startIdx, int goalIdx, Heuristic hType, GridMap::STAT stat, GridMap::COST_TYPE costType, std::vector<int>& path);

}

#endif
