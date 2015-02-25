#ifndef _CMA_SEARCH
#define _CMA_SEARCH

#include "ControllerData.h"
//double Evaluator(double* params, DecoScene* scene);

class CMASearcher
{
public:
	CMASearcher();
	CMASearcher(int dim);
	virtual ~CMASearcher();
	virtual void SetDimension(int dim);
	virtual void SetEvaluatorFunc(double (*Evaluate)(const ControllerData&, int, double*));
	virtual int Search(double* lower_bound, double* upper_bound, double* argMin, int maxIterations);
	virtual int RestartSearch(double* lower_bound, double* upper_bound, double* argMin, int maxIterations);
	void CreateMessageQueue();

protected:
	int mDim;
	double* mPrevSol;
	double* mStandardDeviation;
	int* mInfeasibleTime;
	double (*mEvaluator)(const ControllerData&, int, double*);
	bool isFeasible(double* value, double* lower_bound, double* upper_bound);
	void calculateSearchStandardDeviation(double* lower_bound, double* upper_boun);
	void clear();
	void setInitialGuess(double* lower_bound, double* upper_boun);
};




#endif