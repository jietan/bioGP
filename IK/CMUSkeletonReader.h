#ifndef _CMU_SKEL_READER
#define _CMU_SKEL_READER

#include "dart/dynamics/Skeleton.h"
#include <string>

dart::dynamics::Skeleton* ReadCMUSkeleton(const std::string& filename);

#endif