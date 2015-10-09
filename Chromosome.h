#ifndef CHROMOSOME_H
#define CHROMOSOME_H

#include "main.h"

class Chromosome
{
public:
    virtual ~Chromosome() {};
    virtual Trail* getTrail() = 0;
    virtual string getString() = 0;
    virtual string getChart() = 0;
    virtual bool isFeasible() = 0;
    virtual double fitness() = 0;
    virtual int getFirstBadPt() = 0;

    virtual void nPointMutation( int n ) = 0;
    virtual void nPointLocalMutation( int n ) = 0;
    virtual void nPointSyncMutation( int n ) = 0;
    virtual void nPointPreMutation( int n ) = 0;
    virtual void nPointPostMutation( int n ) = 0;
    virtual void nPointPreSyncMutation( int n ) = 0;
    virtual void nPointPostSyncMutation( int n ) = 0;
    virtual void nPointPreRangeMutation( int n ) = 0;
    virtual void nPointProbMutation( int n ) = 0;
    virtual void randomPostMutation( int n ) = 0;
    virtual void invertMutation( ) = 0;
    virtual Chromosome* permute( int choice ) = 0;
};

#endif
