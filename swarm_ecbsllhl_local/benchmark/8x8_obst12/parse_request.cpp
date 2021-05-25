#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <string>
#include <string.h>

typedef struct Position {
	std::string x;
	std::string y;
} Position;

struct Robot {
	Position goal;
	Position start;
	std::string name;
} Robot;

void parse_request(std::string path) {
	YAML::Node yaml = YAML::LoadFile(path);
	const YAML::Node& robots = yaml["agents"];
	printf("%d\n", robots.size());
	for(int i = 0; i < robots.size(); ++i) {
		//YAML::Node name = robots[i]["name"];
		std::string name = robots[i]["name"].as<std::string>();
		printf("%s\n", name.c_str());
		const YAML::Node& start = robots[i]["start"];
		const YAML::Node& goal = robots[i]["goal"];
		int start_x = start[0].as<int>();
		int start_y = start[1].as<int>();
		int goal_x = goal[0].as<int>();
		int goal_y = goal[1].as<int>();
		printf("start: %d, %d\n", start_x, start_y);
		printf("goal: %d, %d\n", goal_x, goal_y);
	}
}

bool isYamlFile(const char *filename) {
	const char *ext = strrchr(filename, '.');
	return strcmp(".yaml", ext) == 0;
}

int main(void) {
	const char *filepath = "/home/ubuntu/github/libMultiRobotPlanning/benchmark/8x8_obst12/map_8by8_obst12_agents15_ex0.yaml";
	//const char *filepath2 = "/home/ubuntu/github/libMultiRobotPlanning/benchmark/8x8_obst12/map_8by8_obst12_agents15_ex0.txt";
	//const char *filename = "map_8by8_obst12_agents15_ex0.yaml";
	if (isYamlFile(filepath)) {
		printf("Yes!\n");
		parse_request(std::string(filepath));
		return 0;
	}
	printf("No!\n");
	//parse_request("map_8by8_obst12_agents15_ex0.yaml");
	return 0;
}
