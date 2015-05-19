#include "TimeSeries.h"
#include <fstream>
using namespace std;

#include "mathlib.h"

TimeSeries::TimeSeries()
{

}
TimeSeries::~TimeSeries()
{

}
void TimeSeries::ReadFromFile(const string& filename, int valueDim)
{
	mData.clear();
	ifstream inFile(filename.c_str());
	int nTimes = 0;
	inFile >> nTimes;
	mData.resize(nTimes);
	for (int i = 0; i < nTimes; ++i)
	{
		inFile >> mData[i].time;
		mData[i].value.resize(valueDim);
		for (int j = 0; j < valueDim; ++j)
		{
			inFile >> mData[i].value[j];
		}
	}

}
Eigen::VectorXd TimeSeries::GetValue(double time)
{
	int n = static_cast<int>(mData.size());
	int left = 0;
	int right = n;
	while (left <= right)
	{
		int mid = (left + right) / 2;
		if (mData[mid].time < time)
			left = mid + 1;
		else
			right = mid - 1;
	}
	int idx = left;
	if (idx >= n)
		return mData[n - 1].value;
	else if (idx == 0)
		return mData[0].value;
	else
	{
		return LinearInterpolate(mData[idx - 1].value, mData[idx].value, (time - mData[idx - 1].time) / (mData[idx].time - mData[idx - 1].time));
	}
	
}
