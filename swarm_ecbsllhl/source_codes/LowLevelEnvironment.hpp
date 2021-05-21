#pragma once

#include "macro.hpp"
#include "Environment.hpp"

struct LowLevelEnvironment {
	LowLevelEnvironment(
			Environment& env, size_t agentIdx, const Constraints& constraints,
      const PlanResult solution[])
		: m_env(env),
			m_solution(solution) {
				setLowLevelContext(agentIdx, &constraints);
			}

	void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
		//assert(constraints); // [TEST-removed]
		m_agentIdx = agentIdx;
		m_constraints = constraints;
		m_lastGoalConstraint = -1;
		for (const auto& vc : constraints->vertexConstraints) {
			if (vc.x == m_env.m_goals[m_agentIdx].x && vc.y == m_env.m_goals[m_agentIdx].y) {
				m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
			}
		}
	}

	Cost admissibleHeuristic(const State& s) {
		//return m_env.admissibleHeuristic(s);
		return std::abs(s.x - m_env.m_goals[m_agentIdx].x) +
			std::abs(s.y - m_env.m_goals[m_agentIdx].y);
	}

	Cost focalStateHeuristic(const State& s, Cost gScore) {
		return LL_focalStateHeuristic(s, gScore, m_solution, m_agentIdx);
	}

	Cost focalTransitionHeuristic(const State& s1, const State& s2,
			Cost gScoreS1, Cost gScoreS2) {
		return m_env.focalTransitionHeuristic(s1, s2, gScoreS1, gScoreS2,
				m_solution, m_agentIdx);
	}

	bool isSolution(const State& s) { return m_env.isSolution(s, m_agentIdx, m_lastGoalConstraint); }

	void getNeighbors(const State& s,
			std::vector<Neighbor<State, Action, Cost> >& neighbors) {
		m_env.getNeighbors(s, neighbors, m_constraints);
	}

	private:
	Environment& m_env;
	size_t m_agentIdx;
	const Constraints* m_constraints;
  int m_lastGoalConstraint;
	const PlanResult *m_solution;
};

