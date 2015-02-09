#ifndef _MISC_H
#define _MISC_H

#include <algorithm>

template <typename T>
T Clamp(const T& n, const T& lower, const T& upper) {
	return std::max(lower, std::min(n, upper));
}

#endif 