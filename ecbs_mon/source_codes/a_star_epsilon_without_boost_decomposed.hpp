#pragma once


#include <boost/heap/d_ary_heap.hpp>

#include <unordered_map>
#include <unordered_set>

#include "common_types.hpp"
#include "neighbor.hpp"
//#include "planresult.hpp"
#include "planresult_fixed_size.hpp"
#include "LowLevelEnvironment.hpp"

#include "HighLevelNode_fixed_size.hpp"

#include "ecbs_without_boost.hpp"


//template <typename State, typename Action, typename Cost, typename Environment,
//          typename StateHasher = std::hash<State> >
//class AStarEpsilon {
// public:
//  AStarEpsilon(Environment& environment, float w)
//      : m_env(environment), m_w(w) {}
using StateHasher = std::hash<State>;

struct AstarNode {
	AstarNode(const State& state, Cost fScore, Cost gScore, Cost focalHeuristic)
		: state(state),
		fScore(fScore),
		gScore(gScore),
		focalHeuristic(focalHeuristic),
		stat(0) {}
  
  AstarNode () {}
  
	bool operator<(const AstarNode& other) const {
		// Sort order
		// 1. lowest fScore
		// 2. highest gScore

		// Our heap is a maximum heap, so we invert the comperator function here
		if (fScore != other.fScore) {
			return fScore > other.fScore;
		} else {
			return gScore < other.gScore;
		}
	}

	friend std::ostream& operator<<(std::ostream& os, const AstarNode& AstarNode) {
		os << "state: " << AstarNode.state << " fScore: " << AstarNode.fScore
			<< " gScore: " << AstarNode.gScore << " focal: " << AstarNode.focalHeuristic \
			<< " stat: " << AstarNode.stat;
		return os;
	}

	State state;

	Cost fScore;
	Cost gScore;
	Cost focalHeuristic;
	int stat; //Open 0 or Close 1

};

void dump_openset(std::vector<AstarNode>& openSet) {
	auto iter = openSet.begin();
	auto iterEnd = openSet.end();
	std::cout << "----" << std::endl;
	std::cout << "dump_openset AstarNode: " << std::endl;
	int index = 0;
	for (; iter != iterEnd; ++iter) {
		if(iter->stat == 0) {
			std::cout << index << ": " << *iter << std::endl; 
		}
		index++;
	}
	std::cout << "----" << std::endl;
}

void dump_focalset(std::vector<AstarNode>& openSet, std::vector<int>& focalSet) {
	auto iter = focalSet.begin();
	auto iterEnd = focalSet.end();
	std::cout << "----" << std::endl;
	std::cout << "dump_focalset: " << std::endl;
	for (; iter != iterEnd; ++iter) {
		//std::cout << *iter << std::endl; 
		std::cout << "openset index: "<< *iter << ", AstarNode: " << openSet[*iter] << std::endl; 
	}
	std::cout << "----" << std::endl;
}

int openset_top(std::vector<AstarNode>& openSet) {

#ifdef TRACE
	dump_openset(openSet);
#endif
	AstarNode current = openSet[0];
	size_t current_idx = 0;
	auto iter = openSet.begin();
	auto iterEnd = openSet.end();
	// set initial AstarNode
	for (; iter != iterEnd; ++iter) {
		if (iter->stat == 1) // CLOSED
			continue;
		else {
			current = *iter;
			current_idx = std::distance(openSet.begin(), iter); 
			break;
		}
	}
	// search best AstarNode
	for (; iter != iterEnd; ++iter) {
		if (iter->stat == 1) // CLOSED
			continue;
		if (current.fScore != iter->fScore) {
			if (current.fScore > iter->fScore) {
				current = *iter;
				current_idx = std::distance(openSet.begin(), iter); 
			}
		}
		else {
			if (current.gScore < iter->gScore) {
				current = *iter;
				current_idx = std::distance(openSet.begin(), iter); 
			}
		}
	}

	return current_idx;
}

int focalset_top(std::vector<AstarNode>& openSet, std::vector<int>& focalSet) {
	int coi = focalSet[0]; // coi = current openset index
	AstarNode current = openSet[coi];
	int current_idx = 0;
	auto iter = focalSet.begin();
	auto iterEnd = focalSet.end();
	for (; iter != iterEnd; ++iter) {
		int n_coi = *iter;
		AstarNode n = openSet[n_coi];
		if (current.focalHeuristic != n.focalHeuristic) {
			if (current.focalHeuristic > n.focalHeuristic) {
				coi = n_coi;
				current = n;
				current_idx = std::distance(focalSet.begin(), iter); 
			}
		}
		else if (current.fScore != n.fScore) {
			if (current.fScore > n.fScore) {
				coi = n_coi;
				current = n;
				current_idx = std::distance(focalSet.begin(), iter); 
			}
		}
		else {
			if (current.gScore < n.gScore) {
				coi = n_coi;
				current = n;
				current_idx = std::distance(focalSet.begin(), iter); 
			}
		}
	}

	return current_idx;
}

bool openset_is_allclosed(std::vector<AstarNode>& openSet) {
	auto iter = openSet.begin();
	auto iterEnd = openSet.end();
	for (; iter != iterEnd; ++iter) {
		if(iter->stat == 0)
			return false;
	}
	return true;
}

#if 0
void HLSQueueNode(HighLevelNode* node, size_t i) {
			node->cost += node->solution[i].cost;
			node->LB += node->solution[i].fmin;
			node->focalHeuristic = focalHeuristic(node->solution);
			pls::enqueue(main_loop_task_para, 
					ROBOT_NUMS * node->generation + node->focalHeuristic, 
					EnqFlags::NOHINT, 
					node->id);
}
#endif

#if 1
  bool astar_search(LowLevelEnvironment& m_env, float m_w, 
			const State& startState, PlanResult& solution) {
		solution.clear();
		solution.add(StateCost(startState, 0));

		std::vector<AstarNode> openSet;
		std::vector<int> focalSet;

    std::unordered_map<State, int, StateHasher> stateToHeap;
    std::unordered_set<State, StateHasher> closedSet;
    std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                       StateHasher>
        cameFrom;

    openSet.push_back(
        AstarNode(startState, m_env.admissibleHeuristic(startState), 0, 0));
		int index = openSet.size() - 1;
    stateToHeap.insert(std::make_pair<>(startState, index));
    //(*handle).handle = handle;

    focalSet.push_back(index);

    std::vector<Neighbor<State, Action, Cost> > neighbors;
    neighbors.reserve(10);

    Cost bestFScore = openSet[focalSet[0]].fScore;

    // std::cout << "new search" << std::endl;

    //while (!focalSet.empty()) { // [TODO] this is right ??? => NG
		int which_iter = 0;
		while (!openset_is_allclosed(openSet)) {
#ifdef LLS_INCREMENTAL_DEBUG
			while(getchar() != '\n');
#endif
#ifdef TRACE
			std::cout << "LLS iter : [" << which_iter << "]" << std::endl;
			which_iter++;
#endif
// update focal list
//#ifdef REBUILT_FOCAL_LIST
#if 0
      focalSet.clear();
      const auto& top = openSet.top();
      Cost bestVal = top.fScore;
      auto iter = openSet.ordered_begin();
      auto iterEnd = openSet.ordered_end();
      for (; iter != iterEnd; ++iter) {
        Cost val = iter->fScore;
        if (val <= bestVal * m_w) {
          const auto& s = *iter;
          focalSet.push(s.handle);
        } else {
          break;
        }
      }
#else
      {
        Cost oldBestFScore = bestFScore;
        bestFScore = openSet[openset_top(openSet)].fScore;
#ifdef TRACE
				std::cout << "openSet.top() : " << openSet[openset_top(openSet)] << std::endl;
#endif

#ifdef TRACE
        std::cout << "bestFScore: " << bestFScore << std::endl;
        std::cout << "oldBestScore: " << oldBestFScore << std::endl;
				//std::cout << "At (1): " << std::endl;
				dump_focalset(openSet, focalSet);
#endif
        if (bestFScore > oldBestFScore) {
          // std::cout << "oldBestFScore: " << oldBestFScore << " newBestFScore:
          // " << bestFScore << std::endl;
          auto iter = openSet.begin();
          auto iterEnd = openSet.end();
          for (; iter != iterEnd; ++iter) {
						if (iter->stat == 1)
							continue;
            Cost val = iter->fScore;
            if (val > oldBestFScore * m_w && val <= bestFScore * m_w) {
							focalSet.push_back(std::distance(openSet.begin(), iter));
            }
						// this is for priority queue
            //if (val > bestFScore * m_w) {
            //  break;
            //}
          }
        }
      }
#endif
// check focal list/open list consistency
//#ifdef CHECK_FOCAL_LIST

			if (focalSet.empty()) 
				std::cout << "[ERROR] focalSet is empty." << std::endl;

#ifdef TRACE
			dump_focalset(openSet, focalSet);
#endif
			int focalset_top_index = focalset_top(openSet, focalSet);
			//AstarNode& current = openSet[focalSet[focalset_top_index]];
			AstarNode current = openSet[focalSet[focalset_top_index]];
			//std::cout << "current: " << current << std::endl;
			//dump_focalset(focalSet);
			if (current.stat == 1) {
				std::cout << "[ERROR] focalSet top is already closed" << std::endl;
				//dump_openset(openSet);
				//dump_focalset(focalSet);
			}
      //m_env.onExpandAstarNode(current.state, current.fScore, current.gScore);

      if (m_env.isSolution(current.state)) {
				solution.states_size = 0;
				solution.actions_size = 0;
        auto iter = cameFrom.find(current.state);
        while (iter != cameFrom.end()) {
					solution.add(
							StateCost(iter->first, std::get<3>(iter->second)));
					solution.add(
							ActionCost(
								std::get<1>(iter->second), std::get<2>(iter->second)));
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        solution.add(StateCost(startState, 0));
				solution.reverse_states();
				solution.reverse_actions();
        solution.cost = current.gScore;
        solution.fmin = openSet[openset_top(openSet)].fScore;

        return true;
      }

#if 0
      focalSet.pop();
      openSet.erase(currentHandle);
      stateToHeap.erase(current.state);
      closedSet.insert(current.state);
#else

#ifdef TRACE
			std::cout << "deleted AstarNode : " << openSet[focalSet[focalset_top_index]] << std::endl;
			std::cout << "deleted AstarNode : " << current << std::endl;
#endif
			openSet[focalSet[focalset_top_index]].stat = 1;
			focalSet.erase(focalSet.begin() + focalset_top_index);
			//openSet[focalSet[focalset_top_index]].stat = 1;
			//current.stat = 1;
      stateToHeap.erase(current.state);
      closedSet.insert(current.state);
			//dump_openset(openSet);
#endif

      // traverse neighbors
      neighbors.clear();
      m_env.getNeighbors(current.state, neighbors);
			//std::cout << "current: " << current << std::endl;
      for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {
        if (closedSet.find(neighbor.state) == closedSet.end()) { // not included in closedSet
					//std::cout << "current.gScore: " << current.gScore << std::endl;
					//std::cout << "neighbor.cost: " << neighbor.cost << std::endl;
          Cost tentative_gScore = current.gScore + neighbor.cost;
          auto iter = stateToHeap.find(neighbor.state);
          if (iter == stateToHeap.end()) {  // Discover a new AstarNode
            // std::cout << "  this is a new AstarNode" << std::endl;
						//std::cout << "tentative_gScore: " << tentative_gScore << std::endl;
						//std::cout << "admissibleHeuristic: " << m_env.admissibleHeuristic(neighbor.state) << std::endl;
            Cost fScore =
                tentative_gScore + m_env.admissibleHeuristic(neighbor.state);
            Cost focalHeuristic =
                current.focalHeuristic +
                m_env.focalStateHeuristic(neighbor.state, tentative_gScore) +
                m_env.focalTransitionHeuristic(current.state, neighbor.state,
                                               current.gScore,
                                               tentative_gScore);
						AstarNode tmp = AstarNode(neighbor.state, fScore, tentative_gScore, focalHeuristic);
            openSet.push_back(tmp);
            if (fScore <= bestFScore * m_w) {
              // std::cout << "focalAdd: " << *handle << std::endl;
							focalSet.push_back(openSet.size() - 1);
            }
            stateToHeap.insert(std::make_pair<>(neighbor.state, openSet.size() - 1));
          } else {
						int index = iter->second;
						AstarNode& n = openSet[index];
            // We found this AstarNode before with a better path
            if (tentative_gScore >= n.gScore) {
              continue;
            }
            Cost last_gScore = n.gScore;
            Cost last_fScore = n.fScore;
            // std::cout << "  this is an old AstarNode: " << tentative_gScore << ","
            // << last_gScore << " " << *handle << std::endl;
            // update f and gScore
            Cost delta = last_gScore - tentative_gScore;
            n.gScore = tentative_gScore;
            n.fScore -= delta;
            //openSet.increase(handle); update queue contents
            //m_env.onDiscover(neighbor.state, n.fScore,
            //                 n.gScore);
            if (n.fScore <= bestFScore * m_w &&
                last_fScore > bestFScore * m_w) {
              // std::cout << "focalAdd: " << *handle << std::endl;
							//focalSet.erase(focalSet.begin() + index); // [TODO] check don't need ???
							auto it = std::find(focalSet.begin(), focalSet.end(), index);
							auto focalset_index = std::distance(focalSet.begin(), it);
							//focalSet.erase(focalSet.begin() + focalset_index); // [TODO] check don't need ???
              focalSet.push_back(index);
            }
          }

          // Best path for this AstarNode so far
          // TODO: this is not the best way to update "cameFrom", but otherwise
          // default c'tors of State and Action are required
          cameFrom.erase(neighbor.state);
          cameFrom.insert(std::make_pair<>(
              neighbor.state,
              std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                tentative_gScore)));
        }
      }
    }

    return false;
  }
#endif

 //private:
//  struct AstarNode;
#if 0
  struct AstarNode {
    AstarNode(const State& state, Cost fScore, Cost gScore, Cost focalHeuristic)
        : state(state),
          fScore(fScore),
          gScore(gScore),
          focalHeuristic(focalHeuristic),
					stat(0) {}
    
    AstarNode::AstarNode() {}
    
    bool operator<(const AstarNode& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScore < other.gScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const AstarNode& AstarNode) {
      os << "state: " << AstarNode.state << " fScore: " << AstarNode.fScore
         << " gScore: " << AstarNode.gScore << " focal: " << AstarNode.focalHeuristic \
				 << " stat: " << AstarNode.stat;
      return os;
    }

    State state;

    Cost fScore;
    Cost gScore;
    Cost focalHeuristic;
		int stat; //Open 0 or Close 1

  };
#endif

#if 0
void dump_openset(std::vector<AstarNode>& openSet) {
	auto iter = openSet.begin();
	auto iterEnd = openSet.end();
	std::cout << "----" << std::endl;
	std::cout << "dump_openset AstarNode: " << std::endl;
	int index = 0;
	for (; iter != iterEnd; ++iter) {
		if(iter->stat == 0) {
			std::cout << index << ": " << *iter << std::endl; 
		}
		index++;
	}
	std::cout << "----" << std::endl;
}
#endif

#if 0
void dump_focalset(std::vector<AstarNode>& openSet, std::vector<int>& focalSet) {
	auto iter = focalSet.begin();
	auto iterEnd = focalSet.end();
	std::cout << "----" << std::endl;
	std::cout << "dump_focalset: " << std::endl;
	for (; iter != iterEnd; ++iter) {
		//std::cout << *iter << std::endl; 
		std::cout << "openset index: "<< *iter << ", AstarNode: " << openSet[*iter] << std::endl; 
	}
	std::cout << "----" << std::endl;
}
#endif

#if 0
int openset_top(std::vector<AstarNode>& openSet) {

#ifdef TRACE
	dump_openset(openSet);
#endif
	AstarNode current = openSet[0];
	size_t current_idx = 0;
	auto iter = openSet.begin();
	auto iterEnd = openSet.end();
	// set initial AstarNode
	for (; iter != iterEnd; ++iter) {
		if (iter->stat == 1) // CLOSED
			continue;
		else {
			current = *iter;
			current_idx = std::distance(openSet.begin(), iter); 
			break;
		}
	}
	// search best AstarNode
	for (; iter != iterEnd; ++iter) {
		if (iter->stat == 1) // CLOSED
			continue;
		if (current.fScore != iter->fScore) {
			if (current.fScore > iter->fScore) {
				current = *iter;
				current_idx = std::distance(openSet.begin(), iter); 
			}
		}
		else {
			if (current.gScore < iter->gScore) {
				current = *iter;
				current_idx = std::distance(openSet.begin(), iter); 
			}
		}
	}

	return current_idx;
}
#endif

#if 0
int focalset_top(std::vector<AstarNode>& openSet, std::vector<int>& focalSet) {
	int coi = focalSet[0]; // coi = current openset index
	AstarNode current = openSet[coi];
	int current_idx = 0;
	auto iter = focalSet.begin();
	auto iterEnd = focalSet.end();
	for (; iter != iterEnd; ++iter) {
		int n_coi = *iter;
		AstarNode n = openSet[n_coi];
		if (current.focalHeuristic != n.focalHeuristic) {
			if (current.focalHeuristic > n.focalHeuristic) {
				coi = n_coi;
				current = n;
				current_idx = std::distance(focalSet.begin(), iter); 
			}
		}
		else if (current.fScore != n.fScore) {
			if (current.fScore > n.fScore) {
				coi = n_coi;
				current = n;
				current_idx = std::distance(focalSet.begin(), iter); 
			}
		}
		else {
			if (current.gScore < n.gScore) {
				coi = n_coi;
				current = n;
				current_idx = std::distance(focalSet.begin(), iter); 
			}
		}
	}

	return current_idx;
}
#endif

#if 0
bool openset_is_allclosed(std::vector<AstarNode>& openSet) {
	auto iter = openSet.begin();
	auto iterEnd = openSet.end();
	for (; iter != iterEnd; ++iter) {
		if(iter->stat == 0)
			return false;
	}
	return true;
}
#endif

// private:
//  Environment& m_env;
//  float m_w;
//};

//}  // namespace libMultiRobotPlanning
