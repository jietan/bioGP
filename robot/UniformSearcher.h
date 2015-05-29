#ifndef _UNIFORM_SEARCHER_H
#define _UNIFORM_SEARCHER_H

#include "Searcher.h"


class UniformSearcher : public Searcher
{
public:
	UniformSearcher();
	UniformSearcher(int dim);
	virtual ~UniformSearcher();
	virtual int Search(CMAData* cData, double* argMin, int maxIterations);
	virtual int Search(CMAData* cData, double* lower_bound, double* upper_bound, double* argMin, int maxIterations);
};

#endif