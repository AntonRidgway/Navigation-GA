#ifndef CoordinateChrom_H
#define CoordinateChrom_H

#include "main.h"

class CoordinateChrom : public Chromosome
{
    public:
        CoordinateChrom(int mSIn, bool** cMap);
        CoordinateChrom(CoordinateChrom* c);
        virtual ~CoordinateChrom();
        virtual Trail* getTrail();
        virtual string getString();
        virtual string getChart();
        virtual bool isFeasible();
        virtual double fitness();
        virtual int getFirstBadPt();

        void setXValue(int i, int value);
        void setYValue(int i, int value);
        void takePoint(int i, CoordinateChrom* c);
        int getXValue(int i);
        bool getYValue(int i);
        int getMapSize();

        virtual void nPointMutation( int n );
        virtual void nPointLocalMutation( int n );
        virtual void nPointSyncMutation( int n );
        virtual void nPointPostMutation( int n );
        virtual void nPointPostSyncMutation( int n );
        virtual void nPointPreMutation( int n );
        virtual void nPointPreSyncMutation( int n );
        virtual void nPointPreRangeMutation( int n );
        virtual void nPointProbMutation( int n );
        virtual void randomPostMutation( int n );
        virtual void invertMutation( );
        virtual CoordinateChrom* permute( int n );

    private:
        int* xValues;
        int* yValues;
        bool** myMap;
        int mapSize;
        double myFitness;
        Trail* myTrail;
};

#endif // TurnPointChrom_H
