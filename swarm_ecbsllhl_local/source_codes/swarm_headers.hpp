#include "macro.hpp"

#ifdef SWARM
#define SWARM_ALIGNMENT 64
#include "swarm/api.h"
#include "swarm/algorithm.h"
#include "swarm/aligned.h"
#include "swarm/cps.h"
#endif
