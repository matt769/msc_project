#include"GridMap.h"
#include"astar.h"
#include<set>
#include<map>
#include <Eigen/Dense>
#include<queue>

// TODO add function to Map class that will return the edge/transition cost between two neighbours
//  for the moment use a placeholder that returns a constant cost

// use std::map to store cell index and value?
// or priority_queue<pair> with custom comparator to only compare the value

// start off with any old implementation
// just get it working
// and then change if really bad

// could have a list the size of numCells
// and then can just index for the values
// memory intensive if a lot of cells



namespace PathPlanning {


    float estimatedCostToGoal(const GridMap &map, const int idxA, const int idxB, const Heuristic hType) {
        float cost;
        Eigen::Vector2i xyA, xyB;

        switch(hType) {
            case Heuristic::ZERO:
                cost = 0.0;
                break;
            case Heuristic::DIAGONAL:
                // doesn't really seem optimal but hey
                map.getXY(idxA, xyA);
                map.getXY(idxB, xyB);
                // since the grid is even, we don't care about the actual step size
                cost = (float)std::max(abs(xyA(0) - xyB(0)), abs(xyA(1) - xyB(1)));  // 'Diagonal' distance
                break;
            case Heuristic::EUCLIDEAN:
                map.getXY(idxA, xyA);
                map.getXY(idxB, xyB);
                cost = (float)sqrt((xyA(0) - xyB(0)) * (xyA(0) - xyB(0)) + (xyA(1) - xyB(1)) * (xyA(1) - xyB(1)));
                break;
            default:
                cost = -1.0;
        }

        return cost;
    }

    // PLACEHOLDER UNTIL THIS IS ADDED TO MAP CLASS
//    float getTransitionCost(const int fromIdx, const int toIdx) {
//        return 1.0;
//
//    }

    // couldn't get it to compile with custom comparator for set<pair<int, float>>
    class compare_values {
        bool operator()(std::pair<int, float>& a, std::pair<int, float>& b) const {
            return a.second < b.second;
        }
    };


    bool a_star(const GridMap &map, const int startIdx, const int goalIdx, std::vector<int>& path) {
        return a_star(map, startIdx, goalIdx, Heuristic::DIAGONAL, path);
    }


    bool a_star(const GridMap& map, int startIdx, int goalIdx, Heuristic hType, std::vector<int>& path) {
        return a_star(map, startIdx, goalIdx,
                      hType, GridMap::STAT::STAT_MEAN, GridMap::COST_TYPE::ABS_CHANGE, path);
    }



    bool a_star(const GridMap& map, int startIdx, int goalIdx,
            Heuristic hType, GridMap::STAT stat, GridMap::COST_TYPE costType,
            std::vector<int>& path) {

        bool reachedGoal = false;


        // list of open nodes
        std::set<int> open;
        // insert starting node
        open.insert(startIdx);

        // queue of open nodes and their fValues
        // this will sort lowest values first (to appear as top())
        typedef std::pair<float, int> valKeyPair;
        std::priority_queue<valKeyPair, std::vector<valKeyPair>, std::greater<valKeyPair>> fValues;
        // insert starting node
        float h = estimatedCostToGoal(map, startIdx, goalIdx, hType);



        fValues.push({h, startIdx});

        // list of g values
        std::map<int, float> gValues;
        // insert starting node
        float g = 0.0;
        gValues.insert({startIdx, g});

        // create list of closed nodes (empty to start with)
        std::set<int> closed;

        // create list of preceding nodes
        //   add null node to it (that 'precedes' the start node)
        std::map<int, int> precedingNodes;
        // insert starting node
        int pNode = -1;
        precedingNodes.insert({startIdx, pNode});

        // while open list is no empty
        while (!open.empty()) {
            //      take node with lowest f value
            valKeyPair currentNodeValKey = fValues.top();
            fValues.pop(); // remove from queue
            int currentIdx = currentNodeValKey.second;
            //float currentFValue = currentNodeValKey.first;

            //      check if it's the goal (if so, finish)
            if(currentIdx == goalIdx) {
                reachedGoal = true;
                // TODO need to record some info here, right?
                //  and what's the preceding node?
                //  actually no, when this node got added the preceding info was added too
            }

            //      also remove from open list
            open.erase(currentIdx);
            //      add to closed list
            closed.insert(currentIdx);

            //      get all neighbours of current node
            int neighbours[8];
            map.getAllNeighbours(currentIdx, neighbours);
            for(int i=0; i < 8; i++) {
                int nIdx = neighbours[i];
                // Check they are valid (should probably add function in Map class for this)
                if(nIdx<0) {
                    continue;
                }

                // add to open list (unless they're in the closed list)
                if(closed.count(nIdx) == 0) {
                    open.insert(nIdx); // TODO don't forget to add to fValues too
                }

                //      calculate their POSSIBLE g value as the cost from start to current + current node to neighbour transition cost
                //      if this g value is less than an existing g value (from some other path) for that neighbour then
                //          override the g value with the lower one
                //          set the preceding node of the neighbour to be the current one
                //          calculate the f value of the neighbour as g value + estimated cost to goal
                float gNeighourNew = gValues.at(currentIdx) + map.getTransitionCost(currentIdx, nIdx, stat, costType);
                // should be guaranteed to have the currentIdx in gValues (and other lists), but not necessarily nIdx
                if(gValues.find(nIdx) == gValues.end()) {
                    // need to add for first time
                    gValues.insert({nIdx, gNeighourNew});
                    precedingNodes.insert({nIdx, currentIdx});
                    float f = gNeighourNew + estimatedCostToGoal(map, currentIdx, nIdx, hType);
                    fValues.push({f, nIdx});
                }
                // if it was already present, then check if new value is lower
                else if(gNeighourNew < gValues.at(nIdx)) {
                    // and if so, update
                    gValues.at(nIdx) = gNeighourNew;
                    precedingNodes.insert({nIdx, currentIdx});
                    float f = gNeighourNew + estimatedCostToGoal(map, currentIdx, nIdx, hType);
                    fValues.push({f, nIdx});
                }

            }


        }

        // if list empty but goal was not reached, return false (failure)
        // if goal reached, then
        //     go through list of preceding nodes (starting at goal) and build full path back to start
        if(reachedGoal) {
            path.push_back(goalIdx);
            int priorNode = precedingNodes.at(goalIdx);
            while(priorNode >= 0) {
                path.push_back(priorNode);
                priorNode = precedingNodes.at(priorNode);
            }
            // reverse it so we start at the origin point
            std::reverse(path.begin(), path.end());
        }



        return reachedGoal;
    }

}

