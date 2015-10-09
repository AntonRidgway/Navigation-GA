#include "main.h"

int poolSize = 100;
int numGens = 1000;
double mutationRate = .01;
double crossoverRate = .90;
int mapSize = 0; //length of the square map, and the number of steps to take
bool** collisionMap = NULL;
int genOfBestFound = 0;

//User-selected parameters
int algorithmChoice = 0;
int selectionChoice = 0;
int crossoverChoice = 0;
int mutationChoice = 0;
int permutationChoice = 0;
double kValue;
int numMutPts;
int numSlicePts;

double getMaxFitness();
int* rouletteSelection( vector<TurnPointChrom*>* c );
int* rouletteRelativeSelection( vector<TurnPointChrom*>* c );
int* tournamentSelection( vector<TurnPointChrom*>* c, double k );
int* rankSelection( vector<TurnPointChrom*>* c );
vector<TurnPointChrom*>* nSliceCrossover( vector<TurnPointChrom*>* c, int* pI, int n, double xProb );
vector<TurnPointChrom*>* nSlicePreCrossover( vector<TurnPointChrom*>* c, int* pI, int n, double xProb );
vector<TurnPointChrom*>* nSlicePostCrossover( vector<TurnPointChrom*>* c, int* pI, int n, double xProb );
vector<TurnPointChrom*>* uniformCrossover( vector<TurnPointChrom*>* c, int* pI, double xProb );
double getPercentSame(vector<TurnPointChrom*>* c);
double getPercentBest(vector<TurnPointChrom*>* c, TurnPointChrom* bestChrom);
void testBattery();

void getInputs();
void loadFile();
vector<TurnPointChrom*>* mainGALoop(vector<TurnPointChrom*>* chromIn);
void printChromPool(vector<TurnPointChrom*>* chromIn);
void printResults(vector<TurnPointChrom*>* chromIn);
void printResults(TurnPointChrom* chromIn);

TurnPointChrom* annealing();

int main()
{
    srand(time(NULL));

    getInputs();
    loadFile();

    if(algorithmChoice == 0)
    {
        for(int j = 0; j < 5; j++)
        {
            vector<TurnPointChrom*>* chromPool = new vector<TurnPointChrom*>();
            for(int i = 0; i < poolSize; i++) chromPool->push_back(new TurnPointChrom(mapSize, collisionMap));
            vector<TurnPointChrom*>* endChroms = mainGALoop(chromPool);
            printResults(endChroms);
            for(int i = 0; i < poolSize; i++) delete endChroms->at(i);
            delete endChroms;
        }
    }
    else
    {
        TurnPointChrom* result = annealing();
        printResults(result);
        delete result;
    }
    for(int i = 0; i < mapSize; i++)
        delete collisionMap[i];
    delete collisionMap;
}

/**
 * Set up the GA with side-effects.
 */
void getInputs()
{
    bool checkVar = true;
    string input;

    while(checkVar)
    {
        cout << "Choose solution method (0 = GA, 1 = Annealing): " << endl;
        getline(cin, input);

        stringstream typeCheckStr(input);
        if(typeCheckStr >> algorithmChoice) checkVar = false;
        else cout << "Not a valid value; try again." << endl;
    }
    checkVar = true;

    if(algorithmChoice == 0)
    {
        while(checkVar)
        {
            cout << "Choose selection algorithm (0 = roulette, 1 = rank, 2 = tournament): " << endl;
            getline(cin, input);

            stringstream typeCheckStr(input);
            if(typeCheckStr >> selectionChoice) checkVar = false;
            else cout << "Not a valid value; try again." << endl;
        }
        checkVar = true;

        if(selectionChoice == 2)
        {
            while(checkVar)
            {
                cout << "Choose k-value for tournament selection (.60-.85): " << endl;
                getline(cin, input);
                stringstream typeCheckStr(input);
                if(typeCheckStr >> kValue) checkVar = false;
                else cout << "Not a valid value; try again." << endl;
            }
            checkVar = true;
        }

        while(checkVar)
        {
            cout << "Choose crossover algorithm (0 = n-slice, 1 = Local N-Slice, 2 = Uniform): " << endl;
            getline(cin, input);
            stringstream typeCheckStr(input);
            if(typeCheckStr >> crossoverChoice) checkVar = false;
            else cout << "Not a valid value; try again." << endl;
        }
        checkVar = true;

        if(crossoverChoice < 2)
        {
            while(checkVar)
            {
                cout << "Choose n-value for n-slice (usually 2-4): " << endl;
                getline(cin, input);
                stringstream typeCheckStr(input);
                if(typeCheckStr >> numSlicePts) checkVar = false;
                else cout << "Not a valid value; try again." << endl;
            }
            checkVar = true;
        }

        while(checkVar)
        {
            cout << "Choose mutation algorithm (0 = N-Point, 1 = Local N-Point, 2 = Gaussian N.P. Mutation): " << endl;
            getline(cin, input);
            stringstream typeCheckStr(input);
            if(typeCheckStr >> mutationChoice) checkVar = false;
            else cout << "Not a valid value; try again." << endl;
        }
        checkVar = true;

        while(checkVar)
        {
            cout << "Choose number of points (usually 1-4): " << endl;
            getline(cin, input);
            stringstream typeCheckStr(input);
            if(typeCheckStr >> numMutPts) checkVar = false;
            else cout << "Not a valid value; try again." << endl;
        }
        checkVar = true;

        while(checkVar)
        {
            cout << "Input population size (50-300): " << endl;
            getline(cin, input);
            stringstream typeCheckStr(input);
            if(typeCheckStr >> poolSize) checkVar = false;
            else cout << "Not a valid value; try again." << endl;
        }
        checkVar = true;

        while(checkVar)
        {
            cout << "Input number of generations: " << endl;
            getline(cin, input);
            stringstream typeCheckStr(input);
            if(typeCheckStr >> numGens) checkVar = false;
            else cout << "Not a valid value; try again." << endl;
        }
        checkVar = true;

        while(checkVar)
        {
            cout << "Input mutation rate (.005-.01): " << endl;
            getline(cin, input);
            stringstream typeCheckStr(input);
            if(typeCheckStr >> mutationRate) checkVar = false;
            else cout << "Not a valid value; try again." << endl;
        }
        checkVar = true;

        while(checkVar)
        {
            cout << "Input crossover rate (.80-.95): " << endl;
            getline(cin, input);
            stringstream typeCheckStr(input);
            if(typeCheckStr >> crossoverRate) checkVar = false;
            else cout << "Not a valid value; try again." << endl;
        }
        checkVar = true;
    }
    else
    {
        while(checkVar)
        {
            cout << "Choose permutation (0 = N-Point, 1 = Local NP, 2 = Gaussian NP): " << endl;
            getline(cin, input);

            stringstream typeCheckStr(input);
            if(typeCheckStr >> permutationChoice) checkVar = false;
            else cout << "Not a valid value; try again." << endl;
        }
        checkVar = true;
    }
}

void loadFile()
{
    bool checkVar = true;
    string fileName;
    bool canFinish = true;
    while(checkVar && canFinish)
    {
        cout << "Enter dataset name: " << endl;
        getline(cin, fileName);
        ifstream fileStream(fileName.c_str());
        if(fileStream.is_open())
        {
            if(fileStream >> mapSize)
            {
                char tempChar;
                collisionMap = new bool*[mapSize];
                for(int i = 0; i < mapSize; i++)
                    collisionMap[i] = new bool[mapSize];

                for(int i = 0; i < mapSize && canFinish; i++)
                {
                    for(int j = 0; j < mapSize && canFinish; j++)
                    {
                        if(fileStream >> tempChar)
                            collisionMap[j][i] = tempChar - '0';
                        else
                        {
                            for(int k = 0; k <= i; k++) delete collisionMap[k];
                            delete collisionMap;
                            canFinish = false;
                        }
                    }
                }
                checkVar = false;
            }
            else canFinish = false;
        }
        else cout << "'" << fileName << "' cannot be found; try again." << endl;
    }

    if(canFinish)
    {
        cout << "Got " << fileName << "." << endl;
        cout << "Navigation map:" << endl;
        for(int i = 0; i < mapSize+2; i++) cout << "/";
        cout<<endl;
        for(int i = 0; i < mapSize; i++)
        {
            cout << "/";
            for(int j = 0; j < mapSize; j++)
            {
                if(collisionMap[j][i])
                    cout << C_BLOCK;
                else
                    cout << C_CLEAR;
            }
            cout << "/" << endl;
        }
        for(int i = 0; i < mapSize+2; i++) cout << "/";
        cout << endl << endl;
    }
}

vector<TurnPointChrom*>* mainGALoop(vector<TurnPointChrom*>* chromIn)
{
    double maxFitnessSoFar = 0;
    double adaptiveMutationRate = mutationRate;

    for( int gens = 0; gens < numGens; gens++ )
    {
        if(numGens >= 10 && (gens+1) % (numGens/10) == 0)
        {
            cout << "Generation " << gens+1 << "..." << endl;
            cout << "Best Fitness: " << maxFitnessSoFar << endl;
            //cout << "Percent best: " << getPercentBest(chromIn,chromIn->at(0)) << endl;
            //cout << "Percent same: " << getPercentSame(chromIn) << endl;
            //cout << "Mutation rate: " << adaptiveMutationRate << endl;
            //for(int i = 0; i < poolSize; i+=3)
            //  cout << i << ": " << chromIn->at(i)->fitness() << endl;
        }
        adaptiveMutationRate = getPercentSame(chromIn);
        adaptiveMutationRate = mutationRate + .1*adaptiveMutationRate;
        if(adaptiveMutationRate < 0) adaptiveMutationRate = 0;
        else if(adaptiveMutationRate > .2) adaptiveMutationRate = .2;

        int* selectedParents;
        //biased selection
        if(selectionChoice == 0)
            selectedParents = rouletteSelection(chromIn);
        else if(selectionChoice == 1)
            selectedParents = rankSelection(chromIn);
        else
            selectedParents = tournamentSelection(chromIn, kValue);

        //crossover
        vector<TurnPointChrom*>* childPool;
        if(crossoverChoice == 0)
            childPool = nSliceCrossover(chromIn, selectedParents, numSlicePts, crossoverRate);
        if(crossoverChoice == 1)
            childPool = nSlicePreCrossover(chromIn, selectedParents, numSlicePts, crossoverRate);
        else
            childPool = uniformCrossover(chromIn, selectedParents, crossoverRate);
        delete selectedParents;

        //Mutation
        for(int i = 0; i < poolSize; i++)
        {
            if ( (((float)rand())/RAND_MAX) < adaptiveMutationRate)
            {
                if(mutationChoice == 0)
                    childPool->at(i)->nPointSyncMutation(numMutPts);
                else if(mutationChoice == 1)
                    childPool->at(i)->nPointPreSyncMutation(numMutPts);
                else
                    childPool->at(i)->nPointProbMutation(numMutPts);
            }
        }

        //Elitism: copy best previous solutions into child pool.
        int best = 0;
        int almostBest = 1;
        for(int i = 1; i < poolSize; i++)
        {
            if(chromIn->at(i)->fitness() > chromIn->at(best)->fitness())
            {
                almostBest = best;
                best = i;
            }
            else if (chromIn->at(i)->fitness() > chromIn->at(almostBest)->fitness())
            {
                almostBest = i;
            }
        }

        //Check for a new best solution.
        if(chromIn->at(best)->fitness() > maxFitnessSoFar)
        {
            maxFitnessSoFar = chromIn->at(best)->fitness();
            genOfBestFound = gens+1;
        }

        //Arbitrarily discard the first two child chromosomes in favor of the elite chromosomes,
        //and copy over their fitness.
        for(int i = 0; i < mapSize-2; i++)
        {
           childPool->at(0)->takePoint(i, chromIn->at(best));
           childPool->at(1)->takePoint(i, chromIn->at(almostBest));
        }

        for(size_t i = 0; i < chromIn->size(); i++)
            delete chromIn->at(i);
        delete chromIn;
        chromIn = childPool;

        //printChromPool(chromIn);
    }// End Main GA Loop
    return chromIn;
}

void printChromPool(vector<TurnPointChrom*>* chromIn)
{
    for(size_t i = 0; i < chromIn->size(); i++)
    {
        cout<<chromIn->at(i)->getChart();
        cout<<chromIn->at(i)->getString()<<endl;
        cout << "Fitness: " << chromIn->at(i)->fitness() << endl;
        cout << "FBP: " << chromIn->at(i)->getFirstBadPt() << "." << endl<<endl;
    }
    cout<<"Press Enter to continue.";
    int blah;
    cin>>blah;
    cout<<endl;
}

void printResults(vector<TurnPointChrom*>* chromIn)
{
    //Find the best (potentially found in the last generation, so not always the elitism-best)
    int bestFit = 0;
    for(int i = 1; i < poolSize; i++)
    {
        if(chromIn->at(i)->fitness() > chromIn->at(bestFit)->fitness()
           && chromIn->at(bestFit)->isFeasible())
            bestFit = i;
    }
    //Generate data to print.
    string bestSolStr = chromIn->at(bestFit)->getChart();
    string bestSolStr2 = chromIn->at(bestFit)->getString();
    cout << endl << "Fittest Solution: " << endl << bestSolStr << endl << bestSolStr2 << endl;
    cout << "Fitness: " << chromIn->at(bestFit)->fitness() << endl;
    cout << "Found on generation " << genOfBestFound << "." << endl;
    cout << "FBP: " << chromIn->at(bestFit)->getFirstBadPt() << "." << endl;
}

void printResults(TurnPointChrom* chromIn)
{
    //Generate data to print.
    string bestSolStr = chromIn->getChart();
    string bestSolStr2 = chromIn->getString();
    cout << endl << "End Solution: " << endl << bestSolStr << endl << bestSolStr2 << endl;
    cout << "Fitness: " << chromIn->fitness() << endl;
    cout << "Found at time " << genOfBestFound << "." << endl;
    cout << "FBP: " << chromIn->getFirstBadPt() << "." << endl;
}

/**
 * rouletteSelection is an algorithm that takes a set of chromosomes and
 * their fitnesses, and performs biased selection, according to their
 * fractional fitness.  It returns the indices of the selected chromosomes.
 * Repeats are allowed.
 *
 * @param c the set of chromosomes to select from
 * @param f the fitness of the chromosomes
 * @return selectedIndices, the indices of the selected chromosomes
 */
int* rouletteSelection( vector<TurnPointChrom*>* c )
{
    double totalFitness = 0;
    for(int i = 0; i < poolSize; i++) totalFitness += c->at(i)->fitness();

    //pick the parents based on their percent fitness
    int* selectedIndices = new int[poolSize];
    for(int i = 0; i < poolSize; i++)
    {
        //pick a spot on the roulette (from 0 to 1), and subtract the fractional fitness
        //until we find a roulette-selected parent.
        double theSpot = (((float)rand())/RAND_MAX);
        int j = 0;
        while(j < poolSize && theSpot > 0)
        {
            theSpot -= c->at(j)->fitness()/totalFitness;
            j++;
        }
        if(j == 0) j++; //if theSpot starts at 0.
        selectedIndices[i] = j-1;
    }
    return selectedIndices;
}

int* rouletteRelativeSelection( vector<TurnPointChrom*>* c )
{
    double worstFit = 1000.;
    for(int i = 0; i < poolSize; i++)
    {
        if(c->at(i)->fitness() < worstFit) worstFit = c->at(i)->fitness();
    }
    worstFit*=.98;//give smallest a small chance

    double totalFitness = 0;
    for(int i = 0; i < poolSize; i++) totalFitness += c->at(i)->fitness() - worstFit;

    //pick the parents based on their percent fitness
    int* selectedIndices = new int[poolSize];
    for(int i = 0; i < poolSize; i++)
    {
        //pick a spot on the roulette (from 0 to 1), and subtract the fractional fitness
        //until we find a roulette-selected parent.
        double theSpot = (((float)rand())/RAND_MAX);
        int j = 0;
        while(j < poolSize && theSpot > 0)
        {
            theSpot -= (c->at(j)->fitness()-worstFit)/totalFitness;
            j++;
        }
        if(j == 0) j++; //if theSpot starts at 0.
        selectedIndices[i] = j-1;
    }
    return selectedIndices;
}

/**
 * tournamentSelection is an algorithm that takes a set of chromosomes and
 * their fitnesses, and performs biased tournament-style selection, according
 * to their relative fitness.  It returns the indices of the selected
 * chromosomes. Repeats are allowed.
 *
 * @param c the set of chromosomes to select from
 * @param f the fitness of the chromosomes
 * @param k the k-value for what percent of the time the stronger chromosome
 * 			should be selected.
 * @return selectedIndices the indices of the selected chromosomes
 */
int* tournamentSelection( vector<TurnPointChrom*>* c, double k )
{
    int* selectedIndices = new int[poolSize];
    for( int i = 0; i < poolSize; i++ )
    {
        int ind1 = rand() % poolSize;
        int ind2 = rand() % poolSize;

        //pick out the stronger chromosome
        int stronger;
        if(c->at(ind1)->fitness() < c->at(ind2)->fitness())
            stronger = 0;
        else if (c->at(ind1)->fitness() > c->at(ind2)->fitness())
            stronger = 1;
        else //equal
            stronger = rand() % 2;

        double r = (((float)rand())/RAND_MAX);
        if(r < k) //select the stronger chromosome
        {
            if (stronger == 1) selectedIndices[i] = ind1;
            else selectedIndices[i] = ind2;
        }
        else //select the weaker chromosome
        {
            if (stronger == 1) selectedIndices[i] = ind2;
            else selectedIndices[i] = ind1;
        }
    }
    return selectedIndices;
}

int* rankSelection( vector<TurnPointChrom*>* c )
{
    int chromRanks[poolSize];
    double sumRanks = poolSize*(poolSize+1)/2;
    for(int i = 0; i < poolSize; i++)
    {
        chromRanks[i] = 1;
        for(int j = 0; j < i; j++)
        {
            if(c->at(j)->fitness() < c->at(i)->fitness())
                chromRanks[i]++;
            else
                chromRanks[j]++;
        }
    }

    //pick the parents based on their percent fitness
    int* selectedIndices = new int[poolSize];
    for(int i = 0; i < poolSize; i++)
    {
        //pick a spot on the roulette (from 0 to 1), and subtract the fractional fitness
        //until we find a roulette-selected parent.
        double theSpot = (((double)rand())/RAND_MAX);
        int j = 0;
        while(j < poolSize && theSpot > 0)
        {
            theSpot -= ((double)chromRanks[j])/sumRanks;
            j++;
        }
        if(j == 0) j++; //if theSpot starts at 0.
        selectedIndices[i] = j-1;
    }
    return selectedIndices;
}

/**
 * nSliceCrossover performs the the N-Slice Crossover on a set of chromosomes,
 * given the indices of the selected parents, and the number of slice points
 * to use.  Crossovers are performed sequentially, but because selection is
 * random, this does not have any effect on the randomness of the whole
 * process.
 *
 * @param c the set of chromosomes to crossover
 * @param pI the indices of the chromosomes to crossover
 * @param n the number of slice points to use
 * @param xProb the probability that crossover will occur
 * @return children the set of child chromosomes created in the crossover
 */
vector<TurnPointChrom*>* nSliceCrossover( vector<TurnPointChrom*>* c, int* pI, int n, double xProb )
{
    vector<TurnPointChrom*>* children = new vector<TurnPointChrom*>();
    for(int i = 0; i < poolSize; i++) children->push_back(new TurnPointChrom(mapSize, collisionMap));

    int par1;
    int par2;
    int chi1;
    int chi2;
    for(int i = 0; i < poolSize; i+=2)
    {
        par1 = pI[i];
        chi1 = i;
        if( i < poolSize-1 ) //deal with an odd-numbered pool-size
        {
            par2 = pI[i+1];
            chi2 = i+1;
        }
        else
        {
            par2 = pI[0];
            chi2 = 0; //overwrite the first child
        }

        if((((float)rand())/RAND_MAX) < xProb)
        {
            //Get n unique, random, sorted indices to slice. (Each slice-number represents the point after
            //chromosome index n and before n+1.) Stop n from exceeding mapSize-2, its max value.

            vector<int> indices;
            int iNeeded;
            if(n < mapSize-2)
                iNeeded = n;
            else
                iNeeded = mapSize - 3;

            for (int j = 0; j < iNeeded; j++)
            {
                int guess = rand()%(mapSize-3);
                while(find(indices.begin(), indices.end(), guess) != indices.end()) //indices contains guess
                {
                    guess = (guess+1)%(mapSize-3);
                }
                indices.push_back(guess);
            }

            bool takeFromPar1 = true;
            int j = 0;
            sort(indices.begin(), indices.end());
            for(vector<int>::iterator currSlice = indices.begin(); currSlice != indices.end(); currSlice++)
            {
                while(j <= *currSlice)
                {
                    if(takeFromPar1)
                    {
                        children->at(chi1)->takePoint(j, c->at(par1));
                        children->at(chi2)->takePoint(j, c->at(par2));
                    }
                    else
                    {
                        children->at(chi1)->takePoint(j, c->at(par2));
                        children->at(chi2)->takePoint(j, c->at(par1));
                    }
                    j++;
                }
                takeFromPar1 = !takeFromPar1;
            }
            while(j < mapSize-2)
            {
                if(takeFromPar1)
                {
                    children->at(chi1)->takePoint(j, c->at(par1));
                    children->at(chi2)->takePoint(j, c->at(par2));
                }
                else
                {
                    children->at(chi1)->takePoint(j, c->at(par2));
                    children->at(chi2)->takePoint(j, c->at(par1));
                }
                j++;
            }
        }
        else //skip this particular crossover
        {
            for(int i = 0; i < mapSize-2; i++)
            {
                children->at(chi1)->takePoint(i, c->at(par1));
                children->at(chi2)->takePoint(i, c->at(par2));
            }
        }
    }
    return children;
}


/**
 * nSliceCrossover performs the the N-Slice Crossover on a set of chromosomes,
 * given the indices of the selected parents, and the number of slice points
 * to use.  Crossovers are performed sequentially, but because selection is
 * random, this does not have any effect on the randomness of the whole
 * process.
 *
 * @param c the set of chromosomes to crossover
 * @param pI the indices of the chromosomes to crossover
 * @param n the number of slice points to use
 * @param xProb the probability that crossover will occur
 * @return children the set of child chromosomes created in the crossover
 */
vector<TurnPointChrom*>* nSlicePreCrossover( vector<TurnPointChrom*>* c, int* pI, int n, double xProb )
{
    vector<TurnPointChrom*>* children = new vector<TurnPointChrom*>();
    for(int i = 0; i < poolSize; i++) children->push_back(new TurnPointChrom(mapSize, collisionMap));

    int par1;
    int par2;
    int chi1;
    int chi2;
    for(int i = 0; i < poolSize; i+=2)
    {
        par1 = pI[i];
        chi1 = i;
        if( i < poolSize-1 ) //deal with an odd-numbered pool-size
        {
            par2 = pI[i+1];
            chi2 = i+1;
        }
        else
        {
            par2 = pI[0];
            chi2 = 0; //overwrite the first child
        }

        if((((float)rand())/RAND_MAX) < xProb)
        {
            //Get the first collision point
            int fBP = c->at(par1)->getFirstBadPt();
            int temp = c->at(par2)->getFirstBadPt();
            if(fBP < temp) fBP = temp; //pick second point in the two to cross over
            if((fBP < 0 && temp < 0) || fBP == mapSize-2) fBP = mapSize-3;
            //Get n unique, random, sorted indices to slice. (Each slice-number represents the point after
            //chromosome index n and before n+1.) Stop n from exceeding mapSize-2, its max value.
            vector<int> indices;
            int iNeeded;
            if(n <= fBP)
                iNeeded = n;
            else
                iNeeded = fBP;

            for (int j = 0; j < iNeeded; j++)
            {
                int guess = (rand()%fBP);
                while(find(indices.begin(), indices.end(), guess) != indices.end()) //indices contains guess
                {
                    if(guess < fBP-1) guess++;
                    else guess = 0;
                }
                indices.push_back(guess);
            }

            bool takeFromPar1 = true;
            int j = 0;
            sort(indices.begin(), indices.end());
            for(vector<int>::iterator currSlice = indices.begin(); currSlice != indices.end(); currSlice++)
            {
                while(j <= *currSlice)
                {
                    if(takeFromPar1)
                    {
                        children->at(chi1)->takePoint(j, c->at(par1));
                        children->at(chi2)->takePoint(j, c->at(par2));
                    }
                    else
                    {
                        children->at(chi1)->takePoint(j, c->at(par2));
                        children->at(chi2)->takePoint(j, c->at(par1));
                    }
                    j++;
                }
                takeFromPar1 = !takeFromPar1;
            }
            while(j < mapSize-2)
            {
                if(takeFromPar1)
                {
                    children->at(chi1)->takePoint(j, c->at(par1));
                    children->at(chi2)->takePoint(j, c->at(par2));
                }
                else
                {
                    children->at(chi1)->takePoint(j, c->at(par2));
                    children->at(chi2)->takePoint(j, c->at(par1));
                }
                j++;
            }
        }
        else //skip this particular crossover
        {
            for(int i = 0; i < mapSize-2; i++)
            {
                children->at(chi1)->setPosition(i, c->at(par1)->getPosition(i));
                children->at(chi1)->setSwitchPt(i, c->at(par1)->getSwitchPt(i));
                children->at(chi2)->setPosition(i, c->at(par2)->getPosition(i));
                children->at(chi2)->setSwitchPt(i, c->at(par2)->getSwitchPt(i));
            }
        }
    }
    return children;
}

/**
 * nSliceCrossover performs the the N-Slice Crossover on a set of chromosomes,
 * given the indices of the selected parents, and the number of slice points
 * to use.  Crossovers are performed sequentially, but because selection is
 * random, this does not have any effect on the randomness of the whole
 * process.
 *
 * @param c the set of chromosomes to crossover
 * @param pI the indices of the chromosomes to crossover
 * @param n the number of slice points to use
 * @param xProb the probability that crossover will occur
 * @return children the set of child chromosomes created in the crossover
 */
vector<TurnPointChrom*>* nSlicePostCrossover( vector<TurnPointChrom*>* c, int* pI, int n, double xProb )
{
    vector<TurnPointChrom*>* children = new vector<TurnPointChrom*>();
    for(int i = 0; i < poolSize; i++) children->push_back(new TurnPointChrom(mapSize, collisionMap));

    int par1;
    int par2;
    int chi1;
    int chi2;
    for(int i = 0; i < poolSize; i+=2)
    {
        par1 = pI[i];
        chi1 = i;
        if( i < poolSize-1 ) //deal with an odd-numbered pool-size
        {
            par2 = pI[i+1];
            chi2 = i+1;
        }
        else
        {
            par2 = pI[0];
            chi2 = 0; //overwrite the first child
        }

        if((((float)rand())/RAND_MAX) < xProb)
        {
            //Get the first collision point
            int fBP = c->at(par1)->getFirstBadPt();
            int temp = c->at(par2)->getFirstBadPt();
            if(fBP > temp) fBP = temp; //pick first point in the two to cross over
            if(fBP < 0) fBP = 0;
            else if(fBP == mapSize-2) fBP = mapSize-3;
            //Get n unique, random, sorted indices to slice. (Each slice-number represents the point after
            //chromosome index n and before n+1.) Stop n from exceeding mapSize-2, its max value.
            vector<int> indices;
            int iNeeded;
            if(n < mapSize-3-fBP)
                iNeeded = n;
            else
                iNeeded = mapSize - 4 - fBP;

            for (int j = 0; j < iNeeded; j++)
            {
                int guess = (rand()%(mapSize-3-fBP))+fBP;
                while(find(indices.begin(), indices.end(), guess) != indices.end()) //indices contains guess
                {
                    if(guess < mapSize-4) guess++;
                    else guess = fBP;
                }
                indices.push_back(guess);
            }

            bool takeFromPar1 = true;
            int j = 0;
            sort(indices.begin(), indices.end());
            for(vector<int>::iterator currSlice = indices.begin(); currSlice != indices.end(); currSlice++)
            {
                while(j <= *currSlice)
                {
                    if(takeFromPar1)
                    {
                        children->at(chi1)->takePoint(j, c->at(par1));
                        children->at(chi2)->takePoint(j, c->at(par2));
                    }
                    else
                    {
                        children->at(chi1)->takePoint(j, c->at(par2));
                        children->at(chi2)->takePoint(j, c->at(par1));
                    }
                    j++;
                }
                takeFromPar1 = !takeFromPar1;
            }
            while(j < mapSize-2)
            {
                if(takeFromPar1)
                {
                    children->at(chi1)->takePoint(j, c->at(par1));
                    children->at(chi2)->takePoint(j, c->at(par2));
                }
                else
                {
                    children->at(chi1)->takePoint(j, c->at(par2));
                    children->at(chi2)->takePoint(j, c->at(par1));
                }
                j++;
            }
        }
        else //skip this particular crossover
        {
            for(int i = 0; i < mapSize-2; i++)
            {
                children->at(chi1)->setPosition(i, c->at(par1)->getPosition(i));
                children->at(chi1)->setSwitchPt(i, c->at(par1)->getSwitchPt(i));
                children->at(chi2)->setPosition(i, c->at(par2)->getPosition(i));
                children->at(chi2)->setSwitchPt(i, c->at(par2)->getSwitchPt(i));
            }
        }
    }
    return children;
}

/**
 * uniformCrossover performs a Uniform Crossover on a set of chromosomes,
 * given the indices of the selected parents.  Crossovers are performed
 * sequentially, but because selection is random, this does not have any
 * effect on the randomness of the whole process.
 *
 * @param c the set of chromosomes to crossover
 * @param pI the indices of the chromosomes to crossover
 * @param xProb the probability that crossover will occur
 * @return children the set of child chromosomes created in the crossover
 */
vector<TurnPointChrom*>* uniformCrossover( vector<TurnPointChrom*>* c, int* pI, double xProb )
{
    vector<TurnPointChrom*>* children = new vector<TurnPointChrom*>();
    for(int i = 0; i < poolSize; i++) children->push_back(new TurnPointChrom(mapSize, collisionMap));

    for(int i = 0; i < poolSize; i+=2)
    {
        if((((float)rand())/RAND_MAX) < xProb)
        {
            for(int j = 0; j < mapSize-2; j++)
            {
                if(rand()%2) //don't crossover
                {
                    children->at(i)->takePoint(j, c->at(pI[i]));
                    if(i < poolSize-1)
                    {
                        children->at(i+1)->takePoint(j, c->at(pI[i+1]));
                    }
                    else
                    {
                        children->at(0)->takePoint(j, c->at(pI[0]));
                    }
                }
                else //crossover
                {
                    if(i < poolSize-1)
                    {
                        children->at(i)->takePoint(j, c->at(pI[i+1]));
                        children->at(i+1)->takePoint(j, c->at(pI[i]));
                    }
                    else
                    {
                        children->at(i)->takePoint(j, c->at(pI[0]));
                        children->at(0)->takePoint(j, c->at(pI[i]));
                    }
                }
            }
        }
        else
        {
            for(int j = 0; j < mapSize-2; j++)
            {
                children->at(i)->takePoint(j, c->at(pI[i]));
                if(i < poolSize-1)
                {
                    children->at(i+1)->takePoint(j, c->at(pI[i+1]));
                }
                else
                {
                    children->at(0)->takePoint(j, c->at(pI[0]));
                }
            }
        }
    }
    return children;
}

double getPercentSame(vector<TurnPointChrom*>* c)
{
    int numSame = 0;
    bool uniqueChrom[poolSize];

    for(int i = 0; i < poolSize; i++)
        uniqueChrom[i] = true;

    for(int i = 0; i < poolSize-1; i++)
    {
        for(int j = i+1; j < poolSize; j++)
        {
            if(c->at(i)->fitness() == c->at(j)->fitness())
            {
                uniqueChrom[i] = false;
                uniqueChrom[j] = false;
            }
        }
    }
    for(int i = 0; i < poolSize; i++)
    {
        if(!uniqueChrom[i])
            numSame++;
    }
    return (double)numSame/poolSize;
}

double getPercentBest(vector<TurnPointChrom*>* c, TurnPointChrom* bestChrom)
{
    int numSame = 0;
    bool isSameAsBest[poolSize];

    for(int i = 0; i < poolSize; i++)
        isSameAsBest[i] = true;

    for(int i = 0; i < poolSize-1; i++)
    {
        for(int j = 0; j < mapSize; j++)
        {
            if(c->at(i)->getPosition(j) != bestChrom->getPosition(j)) isSameAsBest[i] = false;
            else if(c->at(i)->getSwitchPt(j) != bestChrom->getSwitchPt(j)) isSameAsBest[i] = false;
        }
    }
    for(int i = 0; i < poolSize; i++)
    {
        if(isSameAsBest[i])
            numSame++;
    }
    return (double)numSame/poolSize;
}

/**
 * testBattery is a method to test each of the individual components of the GA.
 */
 /*
void testBattery()
{
    cout << "Testing fitness function..." << endl;
    vector<TurnPointChrom> testC(poolSize,mapSize);
    for(int i = 0; i < poolSize; i++)
    {
        testC[i].positions = new int[mapSize-2];
        testC[i].switchPt = new bool[mapSize-2];
    }
    double* testF = new double[poolSize];

    for(int i = 0; i < mapSize-2; i++)
    {
        testC[0].positions[i] = 0;
        testC[0].switchPt[i] = false;
        testC[1].positions[i] = mapSize-1;
        testC[1].switchPt[i] = true;
    }
    for(int i = 2; i < 5; i++)
    {
        for(int j = 0; j < mapSize-2; j++)
        {
            testC[i].positions[j] = rand()%mapSize;
            testC[i].switchPt[j] = rand()%2;
        }
    }
    for(int i = 5; i < poolSize; i++)
    {
        for(int j = 0; j < mapSize-2; j++)
        {
            testC[i].positions[j] = 0;
            testC[i].switchPt[j] = false;
        }
    }

    for(int i = 0; i < poolSize; i++)
    {
        testF[i] = testC[i].fitness();
    }

    cout << "Random chromosome: " << testC[4].chromToString() << endl;
    cout << "Fitness Function Output: " << testC[4].fitness() << endl;
    cout << "(Hand-calculate fitness and compare to test.)" << endl;

    cout << "\nTesting roulette selection..." << endl;
    double totalFitness = 0;
    for(int i = 0; i < 5; i++)
        totalFitness += testC[i].fitness();

    cout << "5 Random Test TurnPointChroms (the other poolSize-5 chromosomes are all-zero, for 0 fitness):" << endl;
    for(int i = 0; i < 5; i++)
    {
        cout << testC[i].chromToString() << " | Fractional Fitness: " << (testF[i]/totalFitness) << endl;
    }
    int* indices = rouletteSelection(testC);
    cout << "Selection indices:" << endl;
    for(int i = 0; i < poolSize; i++)
        cout << indices[i] << ", ";

    cout << "\n\nTesting tournament selection (K = " << kValue << ")...";
    for(int i = 0; i < 5; i++)
    {
        cout << testC[i].chromToString() << " | Fractional Fitness: " << (testF[i]/totalFitness) << endl;
    }
    delete indices;
    indices = tournamentSelection(testC,testF,kValue);
    cout << "Selection indices:" << endl;
    for(int i = 0; i < poolSize; i++)
        cout << indices[i] << ", ";

    delete indices;
    cout << "\n\nTesting n-slice crossover (n = " << numSlicePts << ")..." << endl;
    int* pInd = new int[poolSize];
    pInd[0] = 0;
    pInd[1] = 1;
    for(int i = 2; i < poolSize; i++)
        pInd[i] = 0;
    vector<TurnPointChrom> children;
    children = nSliceCrossover(testC,pInd,numSlicePts,1.1);
    cout << "Parent 1: " << testC[0].chromToString() << endl;
    cout << "Parent 2: " << testC[1].chromToString() << endl;
    cout << "Child 1: " << children[0].chromToString() << endl;
    cout << "Child 2: " << children[1].chromToString() << endl;
    for(int i = 0; i < poolSize; i++)
    {
        delete children[i].positions;
        delete children[i].switchPt;
    }
    delete children;

    cout << "\nTesting uniform crossover..." << endl;
    children = uniformCrossover(testC,pInd,1.1);
    cout << "Parent 1: " << testC[0].chromToString() << endl;
    cout << "Parent 2: " << testC[1].chromToString() << endl;
    cout << "Child 1: " << children[0].chromToString() << endl;
    cout << "Child 2: " << children[1].chromToString() << endl;
    for(int i = 0; i < poolSize; i++)
    {
        delete children[i].positions;
        delete children[i].switchPt;
    }
    delete children;

    cout << "\nTesting n-point mutation (n = " << numMutPts << ")..." << endl;
    int ind = poolSize-1;
    cout << "Initial TurnPointChrom: " << testC[ind].chromToString() << endl;
    testC[ind] = nPointMutation(testC[ind],numMutPts);
    cout << "Mutated TurnPointChrom: " << testC[ind].chromToString() << endl;

    cout << "\nTesting invert mutation..." << endl;
    cout << "Initial TurnPointChrom: " << testC[ind].chromToString() << endl;
    testC[ind] = invertMutation(testC[ind]);
    cout << "Mutated TurnPointChrom: " << testC[ind].chromToString() << endl;
    for(int i = 0; i < poolSize; i++)
    {
        delete testC[i].positions;
        delete testC[i].switchPt;
    }
    delete testC; delete testF; delete pInd;

    cout << "\nTesting is over... resuming normal programming." << endl << endl;
}
*/

TurnPointChrom* annealing()
{
    TurnPointChrom* solution = new TurnPointChrom(mapSize, collisionMap);
    double sFitness = solution->fitness();
    TurnPointChrom* bestSol = new TurnPointChrom(mapSize, collisionMap);
    double bestFitness = sFitness;

    double temperature = 1.0;
    double tempMin = temperature *.05;
    double numIterations = 3000;
    double aValue = 0.98;
    double bValue = 1.03;
    int time = 0;

    cout<<"Beginning annealing process."<<endl;
    while(temperature > tempMin)
    {
        for(int i = 0; i < numIterations; i++)
        {
            TurnPointChrom* newChrom = solution->permute(permutationChoice);
            double newFitness = newChrom->fitness();
            if( newFitness > sFitness || exp((newFitness-sFitness)/temperature) > ((double)rand())/RAND_MAX )
            {
                delete solution;
                solution = newChrom;
                sFitness = newFitness;
                if(newFitness > bestFitness)
                {
                    delete bestSol;
                    bestSol = new TurnPointChrom(newChrom);
                    bestFitness = newFitness;
                    genOfBestFound = time;
                }
            }
            else
                delete newChrom;

            time++;
        }
        temperature *= aValue;
        numIterations *= bValue;
        cout<<"Temp: "<<temperature<<" | Fitness: "<<sFitness<<" | Best: "<<bestFitness<<endl;
        //cout<<solution->chromToChart()<<endl<<bestSol->chromToChart()<<endl;
    }
    delete solution;
    return bestSol;
}
