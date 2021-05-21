#pragma once

#include "macro.hpp"
#include "swarm_headers.hpp"
#include "Constraint.hpp"

struct Constraints {
	Constraints() {
		vertexConstraints_size = 0;
		edgeConstraints_size = 0;
	}
  //std::unordered_set<VertexConstraint> vertexConstraints;
  //std::unordered_set<EdgeConstraint> edgeConstraints;
	//VertexConstraint vertexConstraints[MAX_CONSTRAINT_SIZE];
	VertexConstraint vertexConstraints[50];
	EdgeConstraint edgeConstraints[50];
	int vertexConstraints_size;
	int edgeConstraints_size;
	char pad[56];


	void add(const VertexConstraint& c) {
		if (vertexConstraints_size > MAX_CONSTRAINT_SIZE - 1) {
#ifdef SWARM
			swarm::info("[ERROR] %s vertexConstraints_size > %d\n", __func__, MAX_CONSTRAINT_SIZE);
#endif
			return;
		}

		vertexConstraints[vertexConstraints_size] = c;
		vertexConstraints_size++;
	}

	void add(const EdgeConstraint& c) {
		if (edgeConstraints_size > MAX_CONSTRAINT_SIZE - 1) {
#ifdef SWARM
			swarm::info("[ERROR] %s edgeConstraints_size > %d\n", __func__, MAX_CONSTRAINT_SIZE);
#endif
			return;
		}

		edgeConstraints[edgeConstraints_size] = c;
		edgeConstraints_size++;
	}

  void add(const Constraints& other) {
    //vertexConstraints.insert(other.vertexConstraints.begin(),
    //                         other.vertexConstraints.end());
    //edgeConstraints.insert(other.edgeConstraints.begin(),
    //                       other.edgeConstraints.end());
		if (vertexConstraints_size >= MAX_CONSTRAINT_SIZE || 
				edgeConstraints_size >= MAX_CONSTRAINT_SIZE) {
#ifdef SWARM
			swarm::info("[ERROR] constraints size is over %d", MAX_CONSTRAINT_SIZE );
#endif
			return;
		}
		for (int i = 0; i < other.vertexConstraints_size; ++i) {
			bool found_same_key = false;
			for (int j =0; j < vertexConstraints_size; ++j) {
				//if (vertexConstraints[j].key == other.vertexConstraints[i].key) {
				if (vertexConstraints[j] == other.vertexConstraints[i]) {
					found_same_key = true;
					break;
				}
			}
			if (!found_same_key) { // Not found
				vertexConstraints[vertexConstraints_size] = other.vertexConstraints[i];
				vertexConstraints_size++;
			}
		}
		for (int i = 0; i < other.edgeConstraints_size; ++i) {
			bool found_same_key = false;
			for (int j =0; j < edgeConstraints_size; ++j) {
				//if (edgeConstraints[j].key == other.edgeConstraints[i].key) {
				if (edgeConstraints[j] == other.edgeConstraints[i]) {
					found_same_key = true;
					break;
				}
			}
			if (!found_same_key) { // Not found
				edgeConstraints[edgeConstraints_size] = other.edgeConstraints[i];
				edgeConstraints_size++;
			}
		}
  }

  bool overlap(const Constraints& other) const {
		for (int i = 0; i < other.vertexConstraints_size; ++i) {
			for (int j =0; j < vertexConstraints_size; ++j) {
				if (vertexConstraints[j] == other.vertexConstraints[i])
					return true;
			}
		}
		for (int i = 0; i < other.edgeConstraints_size; ++i) {
			for (int j =0; j < edgeConstraints_size; ++j) {
				if (edgeConstraints[j] == other.edgeConstraints[i])
					return true;
			}
		}

		return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (int i = 0; i < c.vertexConstraints_size; ++i) {
      os << c.vertexConstraints[i] << std::endl;
    }
    for (int i = 0; i < c.edgeConstraints_size; ++i) {
      os << c.edgeConstraints[i] << std::endl;
    }
    return os;
  }

	bool isIncluded(VertexConstraint c) const {
		for (int i = 0; i < vertexConstraints_size; ++i) {
			if (vertexConstraints[i] == c)
				return true;
		}
		return false;
	}

	bool isIncluded(EdgeConstraint c) const {
		for (int i = 0; i < edgeConstraints_size; ++i) {
			if (edgeConstraints[i] == c)
				return true;
		}
		return false;
	}
};
