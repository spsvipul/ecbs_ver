#pragma once

#include "swarm_headers.hpp"

struct StateCost {
	StateCost(State state, Cost cost) : state(state), cost(cost) {}
	StateCost(){}

	State state;
	Cost cost;
};

struct ActionCost {
	ActionCost(Action action, Cost cost) : action(action), cost(cost) {}
	ActionCost(){}

	Action action;
	Cost cost;
};

struct PlanResult {
	PlanResult() {
		states_size = 0;
		actions_size = 0;
		cost = 0;
		fmin = 0;
		success = false;
	}
//public:
	//! states and their gScore
	//std::vector<std::pair<State, Cost> > states;
	//std::vector<StateCost> states;
	StateCost states[STATE_SIZE];
	size_t states_size;

	//! actions and their cost
	//std::vector<std::pair<Action, Cost> > actions;
	//std::vector<ActionCost> actions;
	ActionCost actions[ACTION_SIZE];
	size_t actions_size;

	//! actual cost of the result
	Cost cost;
	//! lower bound of the cost (for suboptimal solvers)
	Cost fmin;

	bool success;

	char pad[48];

	void clear() {
		states_size = 0;
		actions_size = 0;
		cost = 0;
		fmin = 0;
	}

	void add(StateCost s) {
		if (states_size < STATE_SIZE) {
			states[states_size] = s;
			states_size++;
		}
		else {
			swarm::info("[ERROR] PlanResult add StateCost : states_size: %d", states_size);
			}
	}

	void add(ActionCost a) {
		if (actions_size < ACTION_SIZE) {
			actions[actions_size] = a;
			actions_size++;
		}
		else {
			swarm::info("[ERROR] PlanResult add ActionCost : actions_size: %d", actions_size);
			}
	}

	void reverse_states() {
		StateCost tmp_s[STATE_SIZE];
		size_t j = states_size - 1;
		for(int i = 0; i < states_size; ++i) {
			tmp_s[i] = states[j];
			j--;
		}
		for(int i = 0; i < states_size; ++i)
			states[i] = tmp_s[i];
	}

	void reverse_actions() {
		ActionCost tmp_a[STATE_SIZE];
		size_t j = actions_size - 1;
		for(int i = 0; i < actions_size; ++i) {
			tmp_a[i] = actions[j];
			j--;
		}
		for(int i = 0; i < actions_size; ++i)
			actions[i] = tmp_a[i];
	}
};

