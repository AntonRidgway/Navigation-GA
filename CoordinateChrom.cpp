#include "main.h"

CoordinateChrom::CoordinateChrom(int mSIn, bool** cMap)
{
    mapSize = mSIn;
    myMap = cMap;
    myFitness = -1;
    myTrail = NULL;
    xValues = new int[mSIn-2];
    yValues = new int[mSIn-2];

    for(int i = 0; i < mapSize-2; i++)
    {
        xValues[i] = rand()%mapSize;
        yValues[i] = rand()%mapSize;
    }
}

CoordinateChrom::CoordinateChrom(CoordinateChrom* c)
{
    mapSize = c->getMapSize();
    myFitness = -1;
    myTrail = NULL;
    xValues = new int[mapSize-2];
    yValues = new int[mapSize-2];

    for(int i = 0; i < mapSize-2; i++)
    {
        xValues[i] = c->getXValue(i);
        yValues[i] = c->getYValue(i);
    }
}

CoordinateChrom::~CoordinateChrom()
{
    delete xValues;
    delete yValues;
    if(myTrail != NULL)
        delete myTrail;
}

/**
 * getTrail is a method that generates the navigation trail
 * from a given CoordinateChrom, in terms of X and Y coordinates.
 *
 * @param c the CoordinateChrom to make the trail from
 * @return trail the trail
 */
Trail* CoordinateChrom::getTrail()
{
    if(myTrail == NULL)
    {
        myTrail = new Trail(xValues,yValues,mapSize,myMap);
    }
    return myTrail;
}

int CoordinateChrom::getFirstBadPt()
{
    int x0 = 0; int y0 = 0;
    for(int i = 0; i < mapSize-1; i++)
    {
        int x1, y1;
        if(i < mapSize-2)
        {
            x1 = xValues[i];
            y1 = yValues[i];
        }
        else
        {
            x1 = mapSize-1;
            y1 = mapSize-1;
        }

        //first, get the magnitude
        int xDirection = abs(x1-x0);
        int yDirection = abs(y1-y0);
        //then, divide from the difference to normalize the direction
        if(xDirection != 0)
            xDirection = (x1-x0)/xDirection;
        if(yDirection != 0)
            yDirection = (y1-y0)/yDirection;

        bool tryAgain = false;
        for(int j = 1; j <= abs(x1-x0); j++) //check for collisions on initial path guess
        {
            if(myMap[x0+j*xDirection][y0]) //collision with obstacle
                tryAgain = true;
        }
        for(int j = 1; j <= abs(y1-y0); j++)
        {
            if(myMap[x1][y0+j*yDirection]) //collision with obstacle
                tryAgain = true;
        }
        if(!tryAgain) //Plan A: move horizontally, then vertically
        {
            for(int j = 1; j <= abs(x1-x0); j++)
            {
                if(myMap[x0+j*xDirection][y0] == true) return i;
            }
            for(int j = 1; j <= abs(y1-y0); j++)
            {
                if(myMap[x1][y0+j*yDirection] == true) return i;
            }
        }
        else //Plan B: move vertically, then horizontally
        {
            for(int j = 1; j <= abs(y1-y0); j++)
            {
                if(myMap[x0][y0+j*yDirection] == true) return i;
            }

            for(int j = 1; j <= abs(x1-x0); j++)
            {
                if(myMap[x0+j*xDirection][y1] == true) return i;
            }
        }
        x0 = x1;
        y0 = y1;
    }

    return -1;
}

/**
 * getString is a method that generates a string from a given CoordinateChrom.
 *
 * @param c the CoordinateChrom to make the string from
 * @return temp the string
 */
string CoordinateChrom::getString()
{
    stringstream temp1;
    stringstream temp2;
    for(int i = 0; i < mapSize-3; i++)
    {
        temp1<<xValues[i]<<",";
        temp2<<yValues[i]<<",";
    }
    temp1<<xValues[mapSize-3];
    temp2<<yValues[mapSize-3];
    return ""+temp1.str()+"\n"+temp2.str();
}


 /**
 * getChart is a method that maps a string from a given CoordinateChrom.
 *
 * @param c the CoordinateChrom to make the string from
 * @return temp the string
 */
string CoordinateChrom::getChart()
{
    string temp = "";
    Trail* theTrail = getTrail();
    int** trail = theTrail->depthArray();

    for(int i = 0; i < mapSize+2; i++)
        temp+="/";
    temp+="\n";
    for(int i = 0; i < mapSize; i++ )
    {
        temp+="/";
        for(int j = 0; j < mapSize; j++)
        {
            if (myMap[j][i] && trail[j][i]>0)
                temp += C_ERROR;
            else if (myMap[j][i] && trail[j][i]==0)
                temp += C_BLOCK;
            else if (!myMap[j][i] && trail[j][i]>0)
                temp += '@'+trail[j][i];
            else
                temp += C_CLEAR;
        }
        temp += "/\n";
    }
    for(int i = 0; i < mapSize+2; i++)
        temp+="/";
    temp+="\n";
    for(int i = 0; i < mapSize; i++)
        delete trail[i];
    delete trail;
    return temp;
}

bool CoordinateChrom::isFeasible()
{
    Trail* theTrail = getTrail();
    int** temp = theTrail->depthArray();

    bool feasible = true;
    for(int i = 0; i < mapSize; i++)
    {
        for(int j = 0; j < mapSize; j++)
        {
            if(myMap[i][j] && temp[i][j] > 0)
                feasible = false;
        }
    }
    for(int i = 0; i < mapSize; i++)
        delete temp[i];
    delete temp;
    return feasible;
}

/**
 * fitness represents the following function:
 *
 * -path length
 * -feasibility
 *  -significant penalty for paths that collide
 *
 * @return the fitness of the CoordinateChrom
 */
double CoordinateChrom::fitness()
{
    if(myFitness == -1)
    {
        double maxFitness = 1000;
        double stepWeight = .1;
        double turnWeight = .2;
        double collisionWeight = .7;
        double divisionFactor = 2;

        Trail* theTrail = getTrail();
        int** chromPath = theTrail->depthArray();
        int numSteps = theTrail->getNumSteps();
        int numTurns = theTrail->getTurns();
        double sMax = mapSize*mapSize-1;
        double tMax = (2*mapSize)+(mapSize%2);
        double pOfCmax = sMax*(sMax+1)/2;

        myFitness = maxFitness;

        double numerator = 1;
        numerator -= collisionWeight*(1.-theTrail->numStepsAtCollisions(myMap)/pOfCmax);
        if(theTrail->isFeasible(myMap))
        {
            numerator -= stepWeight*numSteps/sMax + turnWeight*numTurns/tMax;
            myFitness *= numerator;
        }
        else
        {
            numerator -= stepWeight + turnWeight;
            myFitness *= numerator/divisionFactor;
        }
        //Enforce the minimum in case something goes wrong.
        if(myFitness < 0) myFitness = 0;

        //Cleanup
        for(int i = 0; i < mapSize; i++) delete chromPath[i];
        delete chromPath;
    }
    return myFitness;
}
void CoordinateChrom::setXValue(int i, int value)
{
    xValues[i] = value;
    myFitness = -1;
    if(myTrail != NULL) delete myTrail;
}
void CoordinateChrom::setYValue(int i, int value)
{
    yValues[i] = value;
    myFitness = -1;
    if(myTrail != NULL) delete myTrail;
}
int CoordinateChrom::getXValue(int i)
{
    return xValues[i];
}
bool CoordinateChrom::getYValue(int i)
{
    return yValues[i];
}
int CoordinateChrom::getMapSize()
{
    return mapSize;
}
void CoordinateChrom::takePoint(int i, CoordinateChrom* src)
{
    xValues[i] = src->getXValue(i);
    yValues[i] = src->getYValue(i);
    myFitness = -1;
    if(myTrail != NULL) delete myTrail;
}

/**
 * nPointMutation mutates the given CoordinateChrom by inverting n random bits.
 *
 * @param c the CoordinateChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::nPointMutation( int n )
{
    int spot;
    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = rand()%(mapSize-2);
        setXValue(spot, rand()%mapSize);
        spot = rand()%(mapSize-2);
        setYValue(spot, rand()%mapSize);
    }
}

/**
 * nPointLocalMutation mutates the given CoordinateChrom by inverting n random bits.
 *
 * @param c the CoordinateChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::nPointLocalMutation( int n )
{
    int spot;
    int val;
    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = rand()%(mapSize-2);
        val = getXValue(spot)+(rand()%5)-2;
        if(val > mapSize-1)
            setXValue(spot, mapSize-1);
        else if (getXValue(spot) < 0)
            setXValue(spot, 0);
        else
            setXValue(spot, val);

        spot = rand()%(mapSize-2);
        val = getYValue(spot)+(rand()%5)-2;
        if(val > mapSize-1)
            setYValue(spot, mapSize-1);
        else if (getYValue(spot) < 0)
            setYValue(spot, 0);
        else
            setYValue(spot, val);
    }
}

/**
 * nPointSyncMutation mutates the given CoordinateChrom by randomizing n random xValues in the CoordinateChrom.
 * The randomized xValues are used for both portions of the CoordinateChrom.
 *
 * @param c the CoordinateChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::nPointSyncMutation( int n )
{
    int spot;
    //randomize these indices
    for(int i = 0; i < n; i++)
    {
        spot = rand()%(mapSize-2);
        setXValue(spot, rand()%mapSize);
        setYValue(spot, rand()%mapSize);
    }
}

/**
 * nPointpostMutation mutates the given CoordinateChrom by inverting n random bits.
 *
 * @param c the CoordinateChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::nPointPostMutation( int n )
{
    int spot;

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = 0;
    else if(fBP == mapSize-2) fBP = mapSize-3;

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(mapSize-2-fBP))+fBP;
        setXValue(spot, rand()%mapSize);

        spot = (rand()%(mapSize-2-fBP))+fBP;
        setYValue(spot, rand()%mapSize);
    }
}

/**
 * nPointPostSyncMutation mutates the given CoordinateChrom by inverting n random bits.
 *
 * @param c the CoordinateChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::nPointPostSyncMutation( int n )
{
    int spot;

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = 0;
    else if(fBP == mapSize-2) fBP = mapSize-3;

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(mapSize-2-fBP))+fBP;
        setXValue(spot, rand()%(mapSize));
        setYValue(spot, rand()%(mapSize));
    }
}

/**
 * nPointPreMutation mutates the given CoordinateChrom by inverting n random bits.
 *
 * @param c the CoordinateChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::nPointPreMutation( int n )
{
    int spot;

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = mapSize-3; //open whole chromosome for mutation
    else if(fBP == mapSize-2) fBP = mapSize-3; //should this happen?

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(fBP+1));
        setXValue(spot, rand()%mapSize);

        spot = (rand()%(fBP+1));
        setYValue(spot, rand()%mapSize);
    }
}

/**
 * nPointPreRangeMutation mutates the given CoordinateChrom by inverting n random bits.
 *
 * @param c the CoordinateChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::nPointPreRangeMutation( int n )
{
    int spot;

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = mapSize-3; //open whole chromosome for mutation
    else if(fBP == mapSize-2) fBP = mapSize-3; //should this happen?

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(fBP+1));
        setXValue(spot, rand()%mapSize);

        spot = (rand()%(fBP+1));
        setYValue(spot, rand()%mapSize);
    }
}

/**
 * nPointPreSyncMutation mutates the given CoordinateChrom by inverting n random bits.
 *
 * @param c the CoordinateChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::nPointPreSyncMutation( int n )
{
    int spot;

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = mapSize-3;
    else if(fBP == mapSize-2) fBP = mapSize-3;

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(fBP+1));
        setXValue(spot, rand()%mapSize);
        setYValue(spot, rand()%mapSize);
    }
}

void CoordinateChrom::nPointProbMutation( int n )
{

}

/**
 * randomPostMutation mutates the given CoordinateChrom by inverting n random bits.
 *
 * @param c the CoordinateChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::randomPostMutation( int n )
{
    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = 0;
    else if(fBP == mapSize-2) fBP = mapSize-3;

    //invert at these indices
    for(int i = fBP; i < mapSize-2; i++)
    {
        setXValue(i, rand()%mapSize);
        setYValue(i, rand()%mapSize);
    }
}

/**
 * invertMutation mutates the given CoordinateChrom by inverting the entire bit string.
 *
 * @param c the CoordinateChrom to mutate
 * @return c the mutated CoordinateChrom
 */
void CoordinateChrom::invertMutation( )
{
    for(int i = 0; i < mapSize-2; i++)
    {
        setXValue(i, mapSize-(getXValue(i))-1);
        setYValue(i, mapSize-(getYValue(i))-1);
    }
}

CoordinateChrom* CoordinateChrom::permute( int choice )
{
    CoordinateChrom* permutedChrom = new CoordinateChrom(this);
    int numPoints = (rand()%3)+2;
    if(choice) permutedChrom->nPointSyncMutation(numPoints);
    else permutedChrom->nPointProbMutation(numPoints);
    return permutedChrom;
}
