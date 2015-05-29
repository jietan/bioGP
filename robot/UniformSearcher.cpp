#include "UniformSearcher.h"
#include <glog/logging.h>
#include <algorithm>
using namespace std;
using namespace google;

UniformSearcher::UniformSearcher()
{

}
UniformSearcher::UniformSearcher(int dim) : Searcher(dim)
{
	CHECK(dim == 1) << "UniformSearcher only works on 1D problems.";
}
UniformSearcher::~UniformSearcher()
{

}
int UniformSearcher::Search(CMAData* cData, double* argMin, int maxIterations)
{
	int nParams = cData->GetNumParameters();
	CHECK(nParams == 1) << "UniformSearcher only works on 1D problems.";

	double* lb = new double[nParams];
	double* ub = new double[nParams];

	cData->GetParameterLowerBounds(lb);
	cData->GetParameterUpperBounds(ub);
	int ret = Search(cData, lb, ub, argMin, maxIterations);
	delete[] lb;
	delete[] ub;
	return ret;
}
int UniformSearcher::Search(CMAData* cData, double* lower_bound, double* upper_bound, double* argMin, int maxIterations)
{
	const double step = 0.01;
	int nSamples = round((upper_bound[0] - lower_bound[0]) / step);
	vector<double> values(nSamples + 1, 0);
	double timeElapsed = 0;
	double* params = new double[nSamples + 1];
	for (int i = 0; i <= nSamples; ++i)
	{
		params[i] = static_cast<double>(i) / nSamples;
		cData->FromParameterSetting(&(params[i]));
		values[i] = mEvaluator(cData, 0, &timeElapsed);
		double realParam = params[i] * (upper_bound[0] - lower_bound[0]) + lower_bound[0];
		LOG(INFO) << realParam << ": " << values[i];
	}
	vector<double>::iterator it = min_element(values.begin(), values.end());
	*argMin = params[it - values.begin()];
	LOG(INFO) << "best parameter: " << params[it - values.begin()] * (upper_bound[0] - lower_bound[0]) + lower_bound[0] << ": " << *it;
	delete[] params;
	return 1;
}
