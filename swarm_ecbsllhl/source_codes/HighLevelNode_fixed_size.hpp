#pragma once 

#include <boost/heap/d_ary_heap.hpp>

//struct PlanResult;
#include "planresult_fixed_size.hpp"
//struct Constraints;
#include "Constraints_fixed_size.hpp"

#include "macro.hpp"

struct HighLevelNode;

#ifdef NO_SWARM
#ifdef USE_FIBONACCI_HEAP
typedef typename boost::heap::fibonacci_heap<HighLevelNode> openSet_t;
typedef typename openSet_t::handle_type handle_t;
#else
typedef typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
				boost::heap::mutable_<true> >
				openSet_t;
				typedef typename openSet_t::handle_type handle_t;
#endif // USE_FIBONACCI_HEAP
#endif // NO_SWARM

struct HighLevelNode {
	PlanResult solution[ROBOT_NUMS];
	Constraints constraints[ROBOT_NUMS];

	Cost cost;
	Cost LB;  // sum of fmin of solution

	Cost focalHeuristic;
	int timestamp;

	int id;


#ifdef NO_SWARM
	handle_t handle;
	bool operator<(const HighLevelNode& n) const {
		// if (cost != n.cost)
		return cost > n.cost;
		// return id > n.id;
	}
#endif
	int parent_child_id;
	int own_child_id; //child_id
	int generation;
	Cost parentCost;
	Cost parentFH;
	Cost parentTimestamp;
	//char pad[56];
	//char pad[32];
	char pad[20];
};

#ifdef NO_SWARM
struct compareFocalHeuristic {
	bool operator()(const handle_t& h1, const handle_t& h2) const {
		// Our heap is a maximum heap, so we invert the comperator function here
#if 0
		if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
			return (*h1).focalHeuristic > (*h2).focalHeuristic;
		}
		return (*h1).cost > (*h2).cost;
#else
		return (ROBOT_NUMS * (*h1).generation + (*h1).focalHeuristic) > \
			(ROBOT_NUMS * (*h2).generation + (*h2).focalHeuristic);
#endif
	}
};

#ifdef USE_FIBONACCI_HEAP
typedef typename boost::heap::fibonacci_heap<
openSet_t, boost::heap::compare<compareFocalHeuristic> >
focalSet_t;
#else
typedef typename boost::heap::d_ary_heap<
handle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
	boost::heap::compare<compareFocalHeuristic> >
	focalSet_t;
#endif
#endif // NO_SWARM
