#include "main.h"

Trail::Trail(int* posList, bool* switchList, int mSizeIn, bool** cMap)
{
    myPath = new vector< pair<int,int> >;
    mapSize = mSizeIn;
    feasible = -1;
    efficiency = 0;

    bool yMonotone = true; int x0 = 0; int y0 = 0; numSteps = 0;
    myPath->push_back(pair<int,int>(0,0));
    for(int i = 0; i < mapSize-1; i++)
    {
        int x1, y1;
        if(i == mapSize-2)
        {
            x1 = mapSize-1;
            y1 = mapSize-1;
        }
        else
        {
            if(switchList[i]) yMonotone = !yMonotone;
            if(yMonotone)
            {
                x1 = posList[i];    y1 = i+1;
            }
            else
            {
                x1 = i+1;           y1 = posList[i];
            }
        }
        numSteps += abs(x1-x0)+abs(y1-y0);
        efficiency += (i/(mapSize-2))*(abs(x1-x0)+abs(y1-y0));

        if ((x0 == x1 && y0 != y1) || (x0 != x1 && y0 == y1)) //straight line
        {
            //cout<<"("<<x1<<","<<y1<<") is in a line from "<<x0<<","<<y0<<")"<<endl;
            myPath->push_back(pair<int,int>(x1,y1));
        }
        else if (x0 != x1 && y0 != y1) //needs path with turn
        {
            //cout<<"("<<x1<<","<<y1<<") is not in a line from "<<x0<<","<<y0<<")"<<endl;
            //first, get the magnitude
            int xDirection = x1-x0;
            int yDirection = y1-y0;
            //then, divide from the difference to normalize the direction
            if(xDirection > 0) xDirection = 1;
            else if(xDirection < 0) xDirection = -1;
            if(yDirection > 0) yDirection = 1;
            else if(yDirection < 0) yDirection = -1;

            bool tryAgain = false;
            for(int j = xDirection; j != (abs(x1-x0)+1)*xDirection; j+=xDirection) //check for collisions on initial path guess
            {
                if(cMap[x0+j][y0]) //collision with obstacle
                    tryAgain = true;
            }
            for(int j = yDirection; j != (abs(y1-y0)+1)*yDirection; j+=yDirection)
            {
                if(cMap[x1][y0+j]) //collision with obstacle
                    tryAgain = true;
            }

            if(!tryAgain) //Plan A: move horizontally, then vertically
            {
                myPath->push_back(pair<int,int>(x1,y0));
            }
            else //Plan B: move vertically, then horizontally
            {
                myPath->push_back(pair<int,int>(x0,y1));
            }
            myPath->push_back(pair<int,int>(x1,y1));
        }
        //else do nothing; same point twice.
        x0 = x1; y0 = y1;
    }
    /* Loop to print points added to the path
    cout<<endl;
    for(size_t i = 0; i < myPath->size(); i++)
        cout<<"("<<myPath->at(i).first<<","<<myPath->at(i).second<<")";*/
}

Trail::Trail(int* xList, int* yList, int mSizeIn, bool** cMap)
{
    myPath = new vector< pair<int,int> >;
    mapSize = mSizeIn;
    feasible = -1;

    int x0 = 0; int y0 = 0; numSteps = 0;
    myPath->push_back(pair<int,int>(0,0));
    for(int i = 0; i < mapSize-1; i++)
    {
        int x1, y1;
        if(i == mapSize-2)
        {
            x1 = mapSize-1;
            y1 = mapSize-1;
        }
        else
        {
            x1 = xList[i]; y1 = yList[i];
        }
        numSteps += abs(x1-x0)+abs(y1-y0);
        efficiency += (i/(mapSize-2))*(abs(x1-x0)+abs(y1-y0));

        if ((x0 == x1 && y0 != y1) || (x0 != x1 && y0 == y1)) //straight line
        {
            //cout<<"("<<x1<<","<<y1<<") is in a line from "<<x0<<","<<y0<<")"<<endl;
            myPath->push_back(pair<int,int>(x1,y1));
        }
        else if (x0 != x1 && y0 != y1) //needs path with turn
        {
            //cout<<"("<<x1<<","<<y1<<") is not in a line from "<<x0<<","<<y0<<")"<<endl;
            //first, get the magnitude
            int xDirection = x1-x0;
            int yDirection = y1-y0;
            //then, divide from the difference to normalize the direction
            if(xDirection > 0) xDirection = 1;
            else if(xDirection < 0) xDirection = -1;
            if(yDirection > 0) yDirection = 1;
            else if(yDirection < 0) yDirection = -1;

            bool tryAgain = false;
            for(int j = xDirection; j != (abs(x1-x0)+1)*xDirection; j+=xDirection) //check for collisions on initial path guess
            {
                if(cMap[x0+j][y0]) //collision with obstacle
                    tryAgain = true;
            }
            for(int j = yDirection; j != (abs(y1-y0)+1)*yDirection; j+=yDirection)
            {
                if(cMap[x1][y0+j]) //collision with obstacle
                    tryAgain = true;
            }

            if(!tryAgain) //Plan A: move horizontally, then vertically
            {
                myPath->push_back(pair<int,int>(x1,y0));
            }
            else //Plan B: move vertically, then horizontally
            {
                myPath->push_back(pair<int,int>(x0,y1));
            }
            myPath->push_back(pair<int,int>(x1,y1));
        }
        //else do nothing; same point twice.
        x0 = x1; y0 = y1;
    }
    /* Loop to print points added to the path
    cout<<endl;
    for(size_t i = 0; i < myPath->size(); i++)
        cout<<"("<<myPath->at(i).first<<","<<myPath->at(i).second<<")";*/
}

Trail::~Trail()
{
    delete myPath;
}

int** Trail::distArray()
{
    int** distMap = new int*[mapSize];
    for(int i = 0; i < mapSize; i++)
    {
        distMap[i] = new int[mapSize];
        for(int j = 0; j < mapSize; j++)
            distMap[i][j] = 0;
    }

    int distSoFar = 1;
    for(size_t i = 0; i < myPath->size()-1; i++)
    {
        int x0 = myPath->at(i).first;      int y0 = myPath->at(i).second;
        int x1 = myPath->at(i+1).first;    int y1 = myPath->at(i+1).second;
        if(x0 == x1)
        {
            int direction = y1-y0;
            if (direction != 0) direction /= abs(y1-y0);
            for(int j = y0; j != y1; j += direction)
            {
                distMap[x0][j] = distSoFar;
                distSoFar++;
            }
        }
        else if (y0 == y1)
        {
            int direction = x1-x0;
            if (direction != 0) direction /= abs(x1-x0);
            for(int j = x0; j != x1; j += direction)
            {
                distMap[j][y0] = distSoFar;
                distSoFar++;
            }
        }
        else
        {
            cout<<"Error with path structure; non-grid-based movement."<<endl;
        }
    }
    distMap[mapSize-1][mapSize-1] = distSoFar;
    return distMap;
}

int** Trail::depthArray()
{
    int** depthMap = new int*[mapSize];
    for(int i = 0; i < mapSize; i++)
    {
        depthMap[i] = new int[mapSize];
        for(int j = 0; j < mapSize; j++)
        {
            depthMap[i][j] = 0;
        }

    }

    for(size_t i = 0; i < myPath->size()-1; i++)
    {
        int x0 = myPath->at(i).first;      int y0 = myPath->at(i).second;
        int x1 = myPath->at(i+1).first;    int y1 = myPath->at(i+1).second;
        if(x0 == x1)
        {
            int direction = y1-y0;
            if (direction < 0) direction = -1;
            else if (direction > 0) direction = 1;
            for(int j = y0; j != y1; j += direction)
                depthMap[x0][j]++;
        }
        else if (y0 == y1)
        {
            int direction = x1-x0;
            if (direction < 0) direction = -1;
            else if (direction > 0) direction = 1;
            for(int j = x0; j != x1; j += direction)
                depthMap[j][y0]++;
        }
        else
        {
            cout<<"Error with path structure; non-grid-based movement."<<endl;
        }
    }
    depthMap[mapSize-1][mapSize-1]++;
    return depthMap;
}

vector< pair<int,int> >* Trail::getPath()
{
    return myPath;
}

int Trail::getTurns()
{
    vector< pair<int,int> >* myPath = getPath();
    if(myPath->size() < 2) return -1;

    int currDirection = -1;
    if(myPath->at(0).first == myPath->at(1).first)
        currDirection = 0;
    else if (myPath->at(0).second == myPath->at(1).second)
        currDirection = 1;
    else
    {
        cout<<"Path is ill-formed."<<endl;
        return -1;
    }

    int turns = 0;
    for(size_t i = 1; i < myPath->size()-1; i++)
    {
        //x values match
        if(myPath->at(i).first == myPath->at(i+1).first)
        {
            if(currDirection != 0)
            {
                currDirection = 0;
                turns++;
            }
        }

        else if (myPath->at(i).second == myPath->at(i+1).second)
        {
            if(currDirection != 1)
            {
                currDirection = 1;
                turns++;
            }
        }
        else
        {
            cout<<"Path is ill-formed."<<endl;
            return -1;
        }
    }
    return turns;
}
int Trail::getNumSteps()
{
    return numSteps;
}
int Trail::numStepsAtCollisions(bool** collisionMap)
{
    int stepsAtC = 0;
    int stepsRemaining = mapSize*mapSize-1;
    for(size_t i = 0; i < myPath->size()-1; i++)
    {
        int x0 = myPath->at(i).first;      int y0 = myPath->at(i).second;
        int x1 = myPath->at(i+1).first;    int y1 = myPath->at(i+1).second;
        if(x0 == x1)
        {
            int direction = y1-y0;
            if (direction != 0) direction /= abs(y1-y0);
            for(int j = y0+direction; j != y1+direction; j += direction)
            {
                if(collisionMap[x0][j]) stepsAtC += stepsRemaining;
                stepsRemaining--;
            }
        }
        else if (y0 == y1)
        {
            int direction = x1-x0;
            if (direction != 0) direction /= abs(x1-x0);
            for(int j = x0+direction; j != x1+direction; j += direction)
            {
                if(collisionMap[j][y0]) stepsAtC += stepsRemaining;
                stepsRemaining--;
            }
        }
        else
        {
            cout<<"Error with path structure; non-grid-based movement."<<endl;
        }
    }
    if(feasible == -1)
    {
        if(stepsAtC>0) feasible = 0;
        else feasible = 1;
    }
    //cout <<"Steps so far: "<<stepsRemaining<<endl;
    //cout <<"Steps at C: "<<stepsAtC<<endl;

    return stepsAtC;
}
int Trail::manDistsAtCollisions(bool** collisionMap)
{
    int manDistsAtC = 0;
    for(size_t i = 0; i < myPath->size()-1; i++)
    {
        int x0 = myPath->at(i).first;      int y0 = myPath->at(i).second;
        int x1 = myPath->at(i+1).first;    int y1 = myPath->at(i+1).second;
        if(x0 == x1)
        {
            int direction = y1-y0;
            if (direction != 0) direction /= abs(y1-y0);
            for(int j = y0+direction; j != y1+direction; j += direction)
            {
                if(collisionMap[x0][j])
                    manDistsAtC += abs(mapSize-1-x0) + abs(mapSize-1-j);
            }
        }
        else if (y0 == y1)
        {
            int direction = x1-x0;
            if (direction != 0) direction /= abs(x1-x0);
            for(int j = x0+direction; j != x1+direction; j += direction)
            {
                if(collisionMap[j][y0])
                    manDistsAtC += abs(mapSize-1-j) + abs(mapSize-1-y0);
            }
        }
        else
        {
            cout<<"Error with path structure; non-grid-based movement."<<endl;
        }
    }
    if(feasible == -1)
    {
        if(manDistsAtC>0) feasible = 0;
        else feasible = 1;
    }
    //cout <<"manDists at C: "<<manDistsAtC<<endl;

    return manDistsAtC;
}
bool Trail::isFeasible(bool** collisionMap)
{
    if(feasible == -1)
    {
        for(size_t i = 0; i < myPath->size()-1; i++)
        {
            int x0 = myPath->at(i).first;      int y0 = myPath->at(i).second;
            int x1 = myPath->at(i+1).first;    int y1 = myPath->at(i+1).second;
            if(x0 == x1)
            {
                int direction = y1-y0;
                if (direction != 0) direction /= abs(y1-y0);
                for(int j = y0+direction; j != y1+direction; j += direction)
                {
                    if(collisionMap[x0][j])
                    {
                        feasible = 0;
                        return feasible;
                    }
                }
            }
            else if (y0 == y1)
            {
                int direction = x1-x0;
                if (direction != 0) direction /= abs(x1-x0);
                for(int j = x0+direction; j != x1+direction; j += direction)
                {
                    if(collisionMap[j][y0])
                    {
                        feasible = 0;
                        return feasible;
                    }
                }
            }
            else
            {
                cout<<"Error with path structure; non-grid-based movement."<<endl;
            }
        }
    }
    return feasible;
}
double Trail::getEfficiency()
{
    return efficiency;
}
