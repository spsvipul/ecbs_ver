#pragma once


#include <unordered_set>

#include "macro.hpp"
#include "swarm_headers.hpp"
#include "common_types.hpp"
#include "neighbor.hpp"

using libMultiRobotPlanning::Neighbor;
#include "planresult_fixed_size.hpp"

static inline State getState(const size_t agentIdx,
		const PlanResult* solution, const size_t t) {
	//const size_t t) const {
	assert(agentIdx < ROBOT_NUMS); // [TEST-removed]
	if (t < solution[agentIdx].states_size) {
		return solution[agentIdx].states[t].state;
	}
	//assert(!solution[agentIdx].states.empty());
	return solution[agentIdx].states[solution[agentIdx].states_size - 1].state;
}

static inline int LL_focalStateHeuristic(
		const State& s, int gScore,
		const PlanResult* solution,
		const size_t m_agentIdx) {
	int numConflicts = 0;
	for (size_t i = 0; i < ROBOT_NUMS; ++i) {
		if (i != m_agentIdx && solution[i].states_size > 0) {
			State state2 = getState(i, solution, s.time);
			if (s.equalExceptTime(state2)) {
				++numConflicts;
			}
		}
	}
	return numConflicts;
}

static inline int focalHeuristic(const PlanResult* solution) {
	int numConflicts = 0;

	int max_t = 0;
	for (int i = 0; i < ROBOT_NUMS; ++i) {
		max_t = std::max<int>(max_t, solution[i].states_size - 1);
	}

	for (int t = 0; t < max_t; ++t) {
		// check drive-drive vertex collisions
		for (size_t i = 0; i < ROBOT_NUMS; ++i) {
			State state1 = getState(i, solution, t);
			for (size_t j = i + 1; j < ROBOT_NUMS;++j) {
				State state2 = getState(j, solution, t);
				if (state1.equalExceptTime(state2)) {
					++numConflicts;
				}
			}
		}
		// drive-drive edge (swap)
		for (size_t i = 0; i < ROBOT_NUMS; ++i) {
			State state1a = getState(i, solution, t);
			State state1b = getState(i, solution, t + 1);
			for (size_t j = i + 1; j < ROBOT_NUMS; ++j) {
				State state2a = getState(j, solution, t);
				State state2b = getState(j, solution, t + 1);
				if (state1a.equalExceptTime(state2b) &&
						state1b.equalExceptTime(state2a)) {
					++numConflicts;
				}
			}
		}
	}
	return numConflicts;
}

class Environment {
 public:
  std::vector<Location> m_goals;

  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)),
        //m_agentIdx(0),
        //m_constraints(nullptr),
        //m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {}

	Environment(void) 
      : //m_agentIdx(0),
        //m_constraints(nullptr),
        //m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {}


  void init(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals) {
		m_dimx = dimx;
		m_dimy = dimy;
		m_obstacles = obstacles;
		m_goals = goals;
	}

  // low-level
  int focalTransitionHeuristic(
      const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const PlanResult solution[], 
			const size_t m_agentIdx) const {
    int numConflicts = 0;
    for (size_t i = 0; i < ROBOT_NUMS; ++i) {
      if (i != m_agentIdx && solution[i].states_size > 0) {
        State s2a = getState(i, solution, s1a.time);
        State s2b = getState(i, solution, s1b.time);
        if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  bool isSolution(const State& s, const size_t m_agentIdx, 
			const int m_lastGoalConstraint) const {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors,
										const Constraints* m_constraints) const {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n, m_constraints) && transitionValid(s, n, m_constraints)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n, m_constraints) && transitionValid(s, n, m_constraints)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      if (stateValid(n, m_constraints) && transitionValid(s, n, m_constraints)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      if (stateValid(n, m_constraints) && transitionValid(s, n, m_constraints)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      if (stateValid(n, m_constraints) && transitionValid(s, n, m_constraints)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1));
      }
    }
  }

  bool getFirstConflict(
			const PlanResult* solution,
      Conflict& result) {

		//pls::info("Environment::getFirstConflict() : at (1)");

    int max_t = 0;
		for (int i = 0; i < ROBOT_NUMS; ++i) {
			max_t = std::max<int>(max_t, solution[i].states_size - 1);
    }

		//pls::info("Environment::getFirstConflict() : at (2)");

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < ROBOT_NUMS; ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < ROBOT_NUMS; ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < ROBOT_NUMS; ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < ROBOT_NUMS; ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
			c1.add(VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.add(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.add(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

 //private:
  bool stateValid(const State& s, const Constraints* m_constraints) const{
    //assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           !m_constraints->isIncluded(VertexConstraint(s.time, s.x, s.y));
  }

  bool transitionValid(const State& s1, const State& s2, const Constraints* m_constraints) const {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return !m_constraints->isIncluded(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y));
  }

 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
};

