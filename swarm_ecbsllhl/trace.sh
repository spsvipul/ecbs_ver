TRACE_PATH=/home/ubuntu/github/swarm_sim/scripts

python ${TRACE_PATH}/trace.py -t $1 > trace.txt
python ${TRACE_PATH}/taskTimeline.py -t $1 -o timeline.png
