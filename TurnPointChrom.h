#ifndef TurnPointChrom_H
#define TurnPointChrom_H

#include "main.h"

class TurnPointChrom : public Chromosome
{
    public:
        TurnPointChrom(int mSIn, bool** cMap);
        TurnPointChrom(TurnPointChrom* c);
        virtual ~TurnPointChrom();
        virtual Trail* getTrail();
        virtual string getString();
        virtual string getChart();
        virtual bool isFeasible();
        virtual double fitness();
        virtual int getFirstBadPt();

        void setPosition(int i, int value);
        void setSwitchPt(int i, bool value);
        void takePoint(int i, TurnPointChrom* c);
        int getPosition(int i);
        bool getSwitchPt(int i);
        int getMapSize();
        bool** getMap();

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
        virtual TurnPointChrom* permute( int choice );

    private:
        int* positions;
        bool** myMap;
        bool* switchPt;
        int mapSize;
        double myFitness;
        Trail* myTrail;
};

#endif // TurnPointChrom_H
