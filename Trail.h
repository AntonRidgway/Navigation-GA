#ifndef TRAIL_H
#define TRAIL_H

#include "main.h"

class Trail
{
    public:
        Trail(int* posList, bool* switchList, int mSizeIn, bool** cMap);
        Trail(int* xList, int* yList, int mSizeIn, bool** cMap);
        ~Trail();
        int** distArray();
        int** depthArray();
        vector< pair<int,int> >* getPath();
        int getTurns();
        int getNumSteps();
        int numStepsAtCollisions(bool** collisionMap);
        int manDistsAtCollisions(bool** collisionMap);
        bool isFeasible(bool** collisionMap);
        double getEfficiency();
    private:
        vector< pair<int,int> >* myPath;
        int mapSize;
        int numSteps; //numSteps counts the steps in the trail, but does not count the starting position as a step.
        int feasible;
        double efficiency;
};

#endif
