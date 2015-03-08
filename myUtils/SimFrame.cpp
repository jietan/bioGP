#include "SimFrame.h"
#include <iomanip>

istream& operator>> (istream& in, SimFrame& rhs)
{
	in >> rhs.mTime;
	if (!rhs.mPose.size())
	{
		LOG(FATAL) << "Memory is not allocated when reading SimFrame.";
	}
	int nDofs = static_cast<int>(rhs.mPose.size());
	for (int i = 0; i < nDofs; ++i)
	{
		in >> rhs.mPose[i];
	}
	return in;
}
ostream& operator<< (ostream& out, const SimFrame& rhs)
{
	out << rhs.mTime << " ";
	int nDofs = static_cast<int>(rhs.mPose.size());
	for (int i = 0; i < nDofs; ++i)
	{
		out << setprecision(16) << rhs.mPose[i] << " ";
	}
	return out;
}
