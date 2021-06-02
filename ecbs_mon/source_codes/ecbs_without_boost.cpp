#include <fstream>
#include <iostream>

#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include "timer.hpp"

#include "macro.hpp"
#include "ecbs_without_boost.hpp"


int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  float w;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")(
      "suboptimality,w", po::value<float>(&w)->default_value(1.0),
      "suboptimality bound");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

   std::unordered_set<Location> obstacles;
  std::vector<Location> goals;
  std::vector<State> startStates;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  g_env.init(dimx, dimy, obstacles, goals);
	g_w = w;
  //std::vector<PlanResult<State, Action, int> > solution;

  Timer timer;
	for (int i = 0; i < startStates.size(); ++i)
		g_initialStates.push_back(startStates[i]);
	g_isNoConflict = false;
#ifdef SWARM
	printf("Swarm\n");
	search(0);
	swarm::run();
#else //NO_SWARM
	printf("No Swarm\n");
	search();
#endif
	bool success = g_isNoConflict;
	//for (int i = 0; i < g_solution.size(); ++i)
//		solution.push_back(g_solution[i]);
  timer.stop();

  if (success) {
    std::cout << "Planning successful! " << std::endl;
    int cost = 0;
    int makespan = 0;
    //for (const auto& s : solution) {
    for (int i = 0; i < ROBOT_NUMS; ++i) {
      //cost += s.cost;
			cost += g_solution[i].cost;
      //makespan = std::max<int>(makespan, s.cost);
      makespan = std::max<int>(makespan, g_solution[i].cost);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    //out << "  highLevelExpanded: " << g_env.highLevelExpanded() << std::endl;
    //out << "  lowLevelExpanded: " << g_env.lowLevelExpanded() << std::endl;
    out << "schedule:" << std::endl;
    //for (size_t a = 0; a < solution.size(); ++a) {
    for (size_t a = 0; a < ROBOT_NUMS; ++a) {
      // std::cout << "Solution for: " << a << std::endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   std::cout << solution[a].states[i].second << ": " <<
      //   solution[a].states[i].first << "->" << solution[a].actions[i].first
      //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
      // }
      // std::cout << solution[a].states.back().second << ": " <<
      // solution[a].states.back().first << std::endl;

      out << "  agent" << a << ":" << std::endl;
      //for (const auto& state : solution[a].states) {
      for (int b = 0; b < g_solution[a].states_size; ++b) {
        //out << "    - x: " << state.first.x << std::endl
        //    << "      y: " << state.first.y << std::endl
        //    << "      t: " << state.second << std::endl;
        out << "    - x: " << g_solution[a].states[b].state.x << std::endl
            << "      y: " << g_solution[a].states[b].state.y << std::endl
            << "      t: " << g_solution[a].states[b].cost << std::endl;
      }
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
