CONFIG_DIR=/home/ubuntu/github/swarm_simvp/configs
#EXAMPLES=/home/tyajima/libMultiRobotPlanning_without_boost/benchmark/32x32_obst204
#EXAMPLES=/home/ubuntu/github/libMultiRobotPlanning/benchmark/8x8_obst12
EXAMPLES=/home/ubuntu/github/libMultiRobotPlanning/benchmark/32x32_obst204
#SIM=/home/tyajima/workspace/swarm_sim/build/opt/sim/sim
SIM=/home/ubuntu/github/swarm_simvp/build/opt/trace_sim/sim

#${SIM} -config ${CONFIG_DIR}/2c.cfg -- ./ecbs_simple_swarm -i ./map_32by32_obst204_agents20_ex10.yaml -o output.yaml -w 1.1
#${SIM} -config ${CONFIG_DIR}/1c.cfg -- ./ecbs_simple_swarm -i ./map_32by32_obst204_agents20_ex10.yaml -o output.yaml -w 1.1

#${SIM} -config ${CONFIG_DIR}/4c.cfg -- ./ecbs_simple_swarm -i ./map_32by32_obst204_agents20_ex10.yaml -o output.yaml -w 1.1
#${SIM} -config ${CONFIG_DIR}/1c.cfg -- ./ecbs_simple_swarm -i ./map_32by32_obst204_agents20_ex10.yaml -o output.yaml -w 1.1
#${SIM} -config ${CONFIG_DIR}/2c.cfg -- ./ecbs_simple_swarm -i ./map_32by32_obst204_agents20_ex10.yaml -o output.yaml -w 1.1

#${SIM} -config ${CONFIG_DIR}/1c.cfg -- ./ecbs_simple_swarm -i ${EXAMPLES}/map_32by32_obst204_agents20_ex72.yaml -o output.yaml -w 1.1
#${SIM} -config ${CONFIG_DIR}/2c.cfg -- ./ecbs_simple_swarm -i ${EXAMPLES}/map_32by32_obst204_agents20_ex72.yaml -o output.yaml -w 1.1

#${SIM} -config ${CONFIG_DIR}/8c.cfg -- ./ecbs_simple_swarm -i ${EXAMPLES}/map_32by32_obst204_agents20_ex16.yaml -o output.yaml -w 1.1

#${SIM} -config ${CONFIG_DIR}/2c.cfg -- /home/ubuntu/github/swarm_ecbsllhl/source_codes/ecbs_simple_swarm -i ${EXAMPLES}/map_8by8_obst12_agents10_ex13.yaml -o output.yaml -w 1.1
${SIM} -config ${CONFIG_DIR}/64c.cfg -- /home/ubuntu/github/swarm_ecbsllhl/source_codes/ecbs_simple_swarm -i ${EXAMPLES}/map_32by32_obst204_agents20_ex1.yaml -o output.yaml -w 1.5
#${SIM} -config ${CONFIG_DIR}/4c.cfg -- ./ecbs_simple_swarm -i ${EXAMPLES}/map_8by8_obst12_agents10_ex13.yaml -o output.yaml -w 1.1
#${SIM} -config ${CONFIG_DIR}/64c.cfg -- ./ecbs_simple_swarm -i ${EXAMPLES}/map_8by8_obst12_agents10_ex2.yaml -o output.yaml -w 1.1

#${SIM} -config ${CONFIG_DIR}/256c.cfg -- ./ecbs_simple_swarm -i ${EXAMPLES}/map_8by8_obst12_agents10_ex2.yaml -o output.yaml -w 1.1

#${SIM} -config ${CONFIG_DIR}/8c.cfg -- ./ecbs_simple_swarm -i ${EXAMPLES}/map_32by32_obst204_agents20_ex16.yaml -o output.yaml -w 1.1
#${SIM} -config ${CONFIG_DIR}/16c.cfg -- ./ecbs_simple_swarm -i ${EXAMPLES}/map_32by32_obst204_agents20_ex16.yaml -o output.yaml -w 1.1


# don't exist ?
#${SIM} -config ${CONFIG_DIR}/8c.cfg -- ./ecbs_simple_swarm -i ./map_32by32_obst204_agents20_ex10.yaml -o output.yaml -w 1.1

#${SIM} -config ${CONFIG_DIR}/16c.cfg -- ./ecbs_simple_swarm -i ./map_32by32_obst204_agents20_ex10.yaml -o output.yaml -w 1.1
