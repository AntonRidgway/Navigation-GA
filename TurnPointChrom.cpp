#include "main.h"

TurnPointChrom::TurnPointChrom(int mSIn, bool** cMap)
{
    mapSize = mSIn;
    myMap = cMap;
    myFitness = -1;
    myTrail = NULL;
    positions = new int[mSIn-2];
    switchPt = new bool[mSIn-2];

    for(int i = 0; i < mapSize-2; i++)
    {
        positions[i] = rand()%mapSize;
        switchPt[i] = rand()%2;
    }
}

TurnPointChrom::TurnPointChrom(TurnPointChrom* c)
{
    mapSize = c->getMapSize();
    myMap = c->getMap();
    myFitness = -1;
    myTrail = NULL;
    positions = new int[mapSize-2];
    switchPt = new bool[mapSize-2];

    for(int i = 0; i < mapSize-2; i++)
    {
        positions[i] = c->getPosition(i);
        switchPt[i] = c->getSwitchPt(i);
    }
}

TurnPointChrom::~TurnPointChrom()
{
    delete positions;
    delete switchPt;
    if(myTrail != NULL)
        delete myTrail;
}

/**
 * getTrail is a method that generates the navigation trail
 * from a given TurnPointChrom, in terms of X and Y coordinates.
 *
 * @param c the TurnPointChrom to make the trail from
 * @return trail the trail
 */
Trail* TurnPointChrom::getTrail()
{
    if(myTrail == NULL)
    {
        myTrail = new Trail(positions,switchPt,mapSize,myMap);
    }
    return myTrail;
}

int TurnPointChrom::getFirstBadPt()
{
    bool yMonotone = true;

    int x0 = 0; int y0 = 0;
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
            if(switchPt[i]) yMonotone = !yMonotone;
            if(yMonotone)
            {
                x1 = positions[i];
                y1 = i+1;
            }
            else
            {
                x1 = i+1;
                y1 = positions[i];
            }
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
 * getString is a method that generates a string from a given TurnPointChrom.
 *
 * @param c the TurnPointChrom to make the string from
 * @return temp the string
 */
string TurnPointChrom::getString()
{
    stringstream temp1;
    stringstream temp2;
    for(int i = 0; i < mapSize-3; i++)
    {
        temp1<<positions[i]<<",";
        temp2<<switchPt[i]<<",";
    }
    temp1<<positions[mapSize-3];
    temp2<<switchPt[mapSize-3];
    return ""+temp1.str()+"\n"+temp2.str();
}


 /**
 * getChart is a method that maps a string from a given TurnPointChrom.
 *
 * @param c the TurnPointChrom to make the string from
 * @return temp the string
 */
string TurnPointChrom::getChart()
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

bool TurnPointChrom::isFeasible()
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
 * @return the fitness of the TurnPointChrom
 */
double TurnPointChrom::fitness()
{
    if(myFitness == -1)
    {
        double maxFitness = 1000;
        double stepWeight = .1;
        double turnWeight = .2;
        double collisionWeight = .6;
        double efficiencyWeight = .1;
        double divisionFactor = 2.;

        double explorationWeight = .6;
        double distanceWeight = 1.-explorationWeight;

        Trail* theTrail = getTrail();
        int** chromPath = theTrail->depthArray();
        double numSteps = theTrail->getNumSteps();
        double numTurns = theTrail->getTurns();
        double sMax = mapSize*mapSize-1;
        double tMax = (2*mapSize)+(mapSize%2);
        double pOfCmax = sMax*(sMax-1)/2;
        double dOfCmax = (mapSize-2)*pow(mapSize-1,2);//sum of manhattan distances from goal at every position

        myFitness = maxFitness;
        double pOfC = theTrail->numStepsAtCollisions(myMap);
        double dOfC = theTrail->manDistsAtCollisions(myMap);
        double avgRatio = (explorationWeight*pOfC/pOfCmax + distanceWeight*dOfC/dOfCmax);
        double numerator = 1. - collisionWeight*avgRatio;
        if(theTrail->isFeasible(myMap))
        {
            numerator -= stepWeight*numSteps/sMax + turnWeight*numTurns/tMax;
            myFitness *= numerator;
        }
        else
        {
            double eValue = theTrail->getEfficiency()/(numSteps-.5*(mapSize-3));
            numerator -= stepWeight + turnWeight + efficiencyWeight*eValue;
            myFitness *= numerator/divisionFactor;
        }
        if(numerator > 1)
        {
            cout<<"numerator: "<<numerator<<endl<<"myFitness"<<myFitness<<endl;
            cout <<"numSteps: "<<numSteps<<"   Max: "<<sMax<<endl<<"numTurns: "<<numTurns<<"   Max: "<<tMax<<endl<<"pOfC: "<<pOfC<<"  Max: "<<pOfCmax<<endl;
            cout<<"pofcmax: "<<pOfCmax<<endl;
        }

        //Enforce the minimum in case something goes wrong.
        if(myFitness < 0) myFitness = 0;

        //Cleanup
        for(int i = 0; i < mapSize; i++) delete chromPath[i];
        delete chromPath;
    }
    return myFitness;
}
void TurnPointChrom::setPosition(int i, int value)
{
    if(i > mapSize || i < 0 || value < 0 || value >= mapSize)
    {
        cout<<"Invalid parameters to set position: "<<i<<", "<<value<<endl;
        return;
    }
    positions[i] = value;
    myFitness = -1;
    if(myTrail != NULL) delete myTrail;
}
void TurnPointChrom::setSwitchPt(int i, bool value)
{
    if(i > mapSize || i < 0 )
    {
        cout<<"Invalid parameters to set position: "<<i<<", "<<value<<endl;
        return;
    }
    switchPt[i] = value;
    myFitness = -1;
    if(myTrail != NULL) delete myTrail;
}
int TurnPointChrom::getPosition(int i)
{
    return positions[i];
}
bool TurnPointChrom::getSwitchPt(int i)
{
    return switchPt[i];
}
int TurnPointChrom::getMapSize()
{
    return mapSize;
}
bool** TurnPointChrom::getMap()
{
    return myMap;
}
void TurnPointChrom::takePoint(int i, TurnPointChrom* src)
{
    positions[i] = src->getPosition(i);
    switchPt[i] = src->getSwitchPt(i);
    myFitness = -1;
    if(myTrail != NULL) delete myTrail;
}

/**
 * nPointMutation mutates the given TurnPointChrom by inverting n random bits.
 *
 * @param c the TurnPointChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::nPointMutation( int n )
{
    int spot;
    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = rand()%(mapSize-2);
        setPosition(spot, rand()%(mapSize));

        spot = rand()%(mapSize-2);
        setSwitchPt(spot, rand()%2);
    }
}

/**
 * nPointLocalMutation mutates the given TurnPointChrom by inverting n random bits.
 *
 * @param c the TurnPointChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::nPointLocalMutation( int n )
{
    int spot;
    int val;
    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = rand()%(mapSize-2);
        val = (rand()%5)-2;
        setPosition(spot, getPosition(spot)+val);
        if(getPosition(spot) > mapSize-1)
            setPosition(spot, mapSize-1);
        else if (getPosition(spot) < 0)
            setPosition(spot, 0);

        spot = rand()%(mapSize-2);
        setSwitchPt(spot, rand()%2);
    }
}

/**
 * nPointSyncMutation mutates the given TurnPointChrom by randomizing n random positions in the TurnPointChrom.
 * The randomized positions are used for both portions of the TurnPointChrom.
 *
 * @param c the TurnPointChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::nPointSyncMutation( int n )
{
    int spot;
    //randomize these indices
    for(int i = 0; i < n; i++)
    {
        spot = rand()%(mapSize-2);
        setPosition(spot, rand()%(mapSize));
        setSwitchPt(spot, rand()%2);
    }
}

/**
 * nPointpostMutation mutates the given TurnPointChrom by inverting n random bits.
 *
 * @param c the TurnPointChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::nPointPostMutation( int n )
{
    int spot;
    int val;

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = 0;
    else if(fBP == mapSize-2) fBP = mapSize-3;

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(mapSize-2-fBP))+fBP;
        val = rand()%(mapSize);
        setPosition(spot, val);

        spot = (rand()%(mapSize-2-fBP))+fBP;
        setSwitchPt(spot, rand()%2);
    }
}

/**
 * nPointPostSyncMutation mutates the given TurnPointChrom by inverting n random bits.
 *
 * @param c the TurnPointChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::nPointPostSyncMutation( int n )
{
    int spot;
    int val;

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = 0;
    else if(fBP == mapSize-2) fBP = mapSize-3;

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(mapSize-2-fBP))+fBP;
        val = rand()%(mapSize);
        setPosition(spot, val);
        setSwitchPt(spot, rand()%2);
    }
}

/**
 * nPointPreMutation mutates the given TurnPointChrom by inverting n random bits.
 *
 * @param c the TurnPointChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::nPointPreMutation( int n )
{
    int spot;
    int val;

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = mapSize-3; //open whole chromosome for mutation
    else if(fBP == mapSize-2) fBP = mapSize-3; //should this happen?

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(fBP+1));
        val = rand()%(mapSize);
        setPosition(spot, val);

        spot = (rand()%(fBP+1));
        setSwitchPt(spot, rand()%2);
    }
}

/**
 * nPointPreSyncMutation mutates the given TurnPointChrom by inverting n random bits.
 *
 * @param c the TurnPointChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::nPointPreSyncMutation( int n )
{
    int spot;
    int val;

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = mapSize-3;
    else if(fBP == mapSize-2) fBP = mapSize-3;

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(fBP+1));
        val = rand()%(mapSize);
        setPosition(spot, val);
        setSwitchPt(spot, rand()%2);
    }
}

/**
 * nPointPreRangeMutation mutates the given TurnPointChrom by inverting n random bits.
 *
 * @param c the TurnPointChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::nPointPreRangeMutation( int n )
{
    int spot;
    int val;

//take a random range before the problem point and limit the mutations to that range.
//not implemented yet

    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = mapSize-3; //open whole chromosome for mutation
    else if(fBP == mapSize-2) fBP = mapSize-3; //should this happen?

    //invert at these indices
    for(int i = 0; i < n; i++)
    {
        spot = (rand()%(fBP+1));
        val = rand()%(mapSize);
        setPosition(spot, val);

        spot = (rand()%(fBP+1));
        setSwitchPt(spot, rand()%2);
    }
}

void TurnPointChrom::nPointProbMutation( int n )
{
    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = mapSize-3; //open whole chromosome for mutation
    else if(fBP == mapSize-2) fBP = mapSize-3; //should this happen?

    double stDev = 0.1*(mapSize-2);

    //use a normal distribution pdf to determine probability of mutation for each element
    double prob[mapSize-2];
    double totalProb = 0;
    for(int i = 0; i < mapSize-2; i++)
    {
        prob[i] = (exp(-pow(i-fBP,2)/(2*pow(stDev,2))))/(stDev*sqrt(2*4*atan(1)));
        totalProb += prob[i];
    }

    vector<int> indices;
    int iNeeded;
    if(n < mapSize-2)
        iNeeded = n;
    else
        iNeeded = mapSize - 3;

    for(int i = 0; i < iNeeded; i++)
    {
        double randVal = ((double)rand())/RAND_MAX;
        int spot;
        for(spot = 0; randVal > 0 && spot < mapSize-2; spot++)
        {
            randVal -= prob[spot]/totalProb;
        }
        if(spot == 0) spot = 1;
        while(find(indices.begin(), indices.end(), spot-1) != indices.end()) //indices contains guess
        {
            spot = (spot)%(mapSize-3)+1;
        }
        indices.push_back(spot-1);
        setPosition(spot-1, rand()%mapSize);
        setSwitchPt(spot-1, rand()%2);
    }
}

/**
 * randomPostMutation mutates the given TurnPointChrom by inverting n random bits.
 *
 * @param c the TurnPointChrom to mutate
 * @param n the number of bits to invert
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::randomPostMutation( int n )
{
    int fBP = getFirstBadPt();
    if(fBP < 0) fBP = 0;
    else if(fBP == mapSize-2) fBP = mapSize-3;

    //invert at these indices
    for(int i = fBP; i < mapSize-2; i++)
    {
        setPosition(i, rand()%(mapSize));
        setSwitchPt(i, rand()%2);
    }
}

/**
 * invertMutation mutates the given TurnPointChrom by inverting the entire bit string.
 *
 * @param c the TurnPointChrom to mutate
 * @return c the mutated TurnPointChrom
 */
void TurnPointChrom::invertMutation( )
{
    for(int i = 0; i < mapSize-2; i++)
    {
        setPosition(i, mapSize-(getPosition(i))-1);
        setSwitchPt(i, !getSwitchPt(i));
    }
}

TurnPointChrom* TurnPointChrom::permute( int choice )
{
    TurnPointChrom* permutedChrom = new TurnPointChrom(this);
    int numPoints = (rand()%3)+2;
    if(choice == 1) permutedChrom->nPointSyncMutation(numPoints);
    else if (choice == 2) permutedChrom->nPointPreSyncMutation(numPoints);
    else permutedChrom->nPointProbMutation(numPoints);
    return permutedChrom;
}
