#pragma once

#include <map>

#include "macro.hpp" 
#include "Environment.hpp"
#include "LowLevelEnvironment.hpp"
#include "HighLevelNode_fixed_size.hpp"
#include "common_types.hpp"
#include "planresult_fixed_size.hpp"
#include "a_star_epsilon_without_boost_decomposed.hpp"




//#define MAX_NODE_NUMS 1000000000 //upto 1e9 possible because 1e10 = 10gb  
#define MAX_NODE_NUMS 500
std::vector<State> g_initialStates __attribute__((aligned(SWARM_ALIGNMENT)));
PlanResult g_solution[ROBOT_NUMS] __attribute__((aligned(SWARM_ALIGNMENT)));
bool g_isNoConflict __attribute__((aligned(SWARM_ALIGNMENT)));
Environment g_env __attribute__((aligned(SWARM_ALIGNMENT)));
float g_w __attribute__((aligned(SWARM_ALIGNMENT)));


HighLevelNode start;
//long unsigned nodeid; 
struct HLSNode_addr {
	HighLevelNode* addr; // 8 bytes
	int pad[14]; 
};

struct LLNodee {
	AstarNode par; // 8 bytes
  Neighbor<State, Action, Cost> neighbor;
  Cost gscore; 
  int newnodeid;
  size_t c_idx;
  HighLevelNode *hls_node;
   
	//int pad[20]; 
};

#ifdef HLS_NO_POINTER
HighLevelNode global_nodelist[10] __attribute__((aligned(SWARM_ALIGNMENT)));
#else
HLSNode_addr global_nodelist[10] __attribute__((aligned(SWARM_ALIGNMENT)));
#endif

#ifdef NO_SWARM
openSet_t g_open;
focalSet_t g_focal;
Cost bestCost;
#endif
//######################low level setup
//std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,StateHasher> cameFrom[MAX_NODE_NUMS];
//std::vector<std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,StateHasher>> cameFrom(MAX_NODE_NUMS);

struct came_from {
	State curr_state;
  Neighbor<State, Action, Cost> neighbor; 
  Cost gscore;
  bool set;
  
	int pad[6];  
};

//came_from cameFrom[32][32][50][MAX_NODE_NUMS]__attribute__((aligned(SWARM_ALIGNMENT))); 
//bool have_prec[32][32][MAX_NODE_NUMS]__attribute__((aligned(SWARM_ALIGNMENT)));

//bool done[MAX_NODE_NUMS] __attribute__((aligned(SWARM_ALIGNMENT)));

//#####################################

int index(const HighLevelNode& n) {
	return pow(2, n.generation) - 1 + n.parent_child_id * 2 + n.own_child_id;
}
 
int index(int parent_nodeid, int child_id) {
	return parent_nodeid * 2 + 1 + child_id;
}

#ifdef SWARM
	void main_loop_task_para(swarm::Timestamp score, int nodeid,HighLevelNode* node );
#else
	void main_loop_task(int n);
#endif
void enq_upper(swarm::Timestamp ts, HighLevelNode* node,  size_t i , int hlsnum);
void LL_pll(swarm::Timestamp ts,AstarNode current,const LLNodee  prev, LowLevelEnvironment  m_env,came_from *cameFrom, bool *done  );

void filler(swarm::Timestamp ts, HighLevelNode* node,  size_t i , int hlsnum){
  //swarm::info("filler reach ts:%x, superts:%x", ts, swarm::superTimestamp());
  //HLSQueueNode( node,  i , ts, hlsnum);
  node->cost += node->solution[i].cost;
  node->LB += node->solution[i].fmin;
	node->focalHeuristic = focalHeuristic(node->solution);  
  
  unsigned int child_prio = node->focalHeuristic * 1000 + node->cost ;
  node->timestamp =  child_prio;
  if((2000+child_prio)>= swarm::superTimestamp()){ //(2000+child_prio)
   //swarm::info("inside filler enqup %i ::::%x", hlsnum,swarm::superTimestamp());
    swarm::enqueue(enq_upper, 2000+child_prio , EnqFlags::PARENTDOMAIN , node,i,hlsnum);//+2000
  }
  else{
    //swarm::info("inside filler deepen %i", hlsnum);
    swarm::deepen();
    swarm::enqueue(main_loop_task_para,  2000+child_prio,EnqFlags::NOHINT,node->id, node);//2000+
  }
  return;

}


void enq_upper(swarm::Timestamp ts, HighLevelNode* node,  size_t i ,  int hlsnum) {
 			

			int parent_prio = node->parentFH * 1000 + node->parentCost;
			int child_prio = node->focalHeuristic * 1000 + node->cost;  
      
      //swarm::info("fractal doain ts = %i,perentts: %i,  nodets: %i, hlsnum: %i , superts: %x", ts,node->parentTimestamp, child_prio , hlsnum,swarm::superTimestamp() );
      
      if(ts>= swarm::superTimestamp()){ 
       
        //swarm::info("inside enqup enqup %i ::::%x", hlsnum,swarm::superTimestamp());
        swarm::enqueue(enq_upper, ts, EnqFlags(PARENTDOMAIN | SAMETASK) , node,i,hlsnum);
      }
      else{
        //swarm::info("inside enqup deepen %i", hlsnum);
        swarm::deepen();
        swarm::enqueue(main_loop_task_para, ts,EnqFlags::NOHINT,node->id, node);  
      }
}

void enq_upper_ll(swarm::Timestamp ts, AstarNode tmp,const LLNodee  prev, LowLevelEnvironment  m_env,came_from *cameFrom, bool *done) {
 					
      //swarm::info("fractal doain ts = %i,perentts: %i,  nodets: %i, hlsnum: %i , superts: %x", ts,node->parentTimestamp, child_prio , hlsnum,swarm::superTimestamp() );
      
      if(ts>= swarm::superTimestamp()){ 
       
        //swarm::info("inside enqup enqup %i ::::%x", hlsnum,swarm::superTimestamp());
        swarm::enqueue(enq_upper_ll, ts, EnqFlags(PARENTDOMAIN | SAMETASK) , tmp,prev,m_env, cameFrom, done);
       }
       else{
         swarm::deepen();
         swarm::enqueue(LL_pll, ts ,EnqFlags::NOHINT, tmp,prev,m_env, cameFrom, done);  
       }
}

//, std::tuple<State, Action, Cost, Cost>,StateHasher> &cameFrom
/*
#ifdef HLS_NO_POINTER
void LL_pll(swarm::Timestamp ts,AstarNode current,HighLevelNode *hls_node, size_t c_idx,const LLNodee &prev) {
      PlanResult *solution = &hls_node->solution[c_idx];
      //solution->clear();
#else 
*/
void LL_pll(swarm::Timestamp ts,AstarNode current,const LLNodee  prev, LowLevelEnvironment  m_env,came_from *cameFrom, bool *done  ) {//came_from (*cameFrom)[32][32][50]  came_from *cameFrom, HighLevelNode *hls_node
      int newnodeid = prev.newnodeid;
      size_t c_idx= prev.c_idx;
      //HighLevelNode *hls_node = global_nodelist[newnodeid].addr;
      PlanResult *solution = &(prev.hls_node)->solution[c_idx];
      //solution->clear();
//#endif 
      //swarm::info("ll_pll start: %i", newnodeid);
      
      if(*done == true) {
        //swarm::info("return from 1");
        return;
      }
      
      if (m_env.isSolution(current.state)) { 
        /*
        cameFrom[newnodeid].insert(std::make_pair<>(current.state,std::make_tuple<>(prev.neighbor.state, prev.neighbor.action, prev.neighbor.cost,prev.gscore)));
        */
        int i;
        //i=cameFrom[current.state.x][current.state.y][current.state.time][newnodeid].path_len;
        i=prev.gscore;
        swarm::info("is soln map size : %i, HLS nodenum: %i....x: %i, y: %i", i,newnodeid,current.state.x,current.state.y );
        
        cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].curr_state=current.state;
        cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].neighbor=prev.neighbor;
        cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].gscore=prev.gscore;
        cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].set=true;    
        *done=true;
             
        solution->clear();
				solution->states_size = 0;
				solution->actions_size = 0;
        State curr_state;
        //curr_state=cameFrom[current.state.x][current.state.y][current.state.time][newnodeid].curr_state;
        curr_state=cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].curr_state;
        //auto iter = cameFrom[newnodeid].find(current.state);
        while (i>0) { 
          //swarm::info("3");
					//solution->add(StateCost(iter->first, std::get<3>(iter->second))); 
          solution->add(StateCost(cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].curr_state,
          cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].gscore));
          
                   
					//solution->add(ActionCost(std::get<1>(iter->second), std::get<2>(iter->second)));
          solution->add(ActionCost(cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].neighbor.action,
          cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].neighbor.cost));
                                                                                                                   
          //curr_state=  (*cameFrom)[current.state.x][current.state.y][current.state.time].neighbor.state; //store current State because we need to use it below for start state
          curr_state=  cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].neighbor.state;
          //iter = cameFrom[newnodeid].find(std::get<0>(iter->second));
          i--;
        }
        solution->add(StateCost(curr_state, 0)); //find this using came from structure
				solution->reverse_states();
				solution->reverse_actions();
        solution->cost = current.gScore;
        solution->fmin = 0; //what is this value +  how can we set this value

        //swarm::info("enq after slln ts: %i",hls_node->parentTimestamp);
				//HLSQueueNode(hls_node, c_idx,ts, newnodeid);
        delete[]cameFrom; 
        delete done;
        //unsigned int ts_tem=  100 + prev.hls_node->parentTimestamp;
        //unsigned int ts_tem= prev.hls_node->parentTimestamp;
        
        //swarm::enqueue(filler,ts_tem,EnqFlags (NOHINT | PARENTDOMAIN), prev.hls_node,c_idx,newnodeid); 
        swarm::enqueue(filler,ts,EnqFlags (NOHINT), prev.hls_node,c_idx,newnodeid); 
        //swarm::enqueue(enq_upper_llsol,ts_tem,EnqFlags::NOHINT, prev.hls_node,c_idx,newnodeid); 
				return;
      }
      
    if (cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].set==false){ 
      //swarm::info("noprev parent x: %i y: %i",current.state.x,current.state.y );
      //cameFrom.erase(neighbor.state);
      //cameFrom[newnodeid].insert(std::make_pair<>(current.state,std::make_tuple<>(prev.neighbor.state, prev.neighbor.action, prev.neighbor.cost,prev.gscore)));
      
      cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].curr_state=current.state;
      cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].neighbor=prev.neighbor;
      cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].gscore=prev.gscore;
      cameFrom[current.state.x + 32*current.state.y + 1024*current.state.time].set=true;    

      // traverse neighbors
      //neighbors.clear();
      std::vector<Neighbor<State, Action, Cost> > neighbors;
      neighbors.reserve(10);
      m_env.getNeighbors(current.state, neighbors); 
			//std::cout << "current: " << current << std::endl;
      for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {
          Cost tentative_gScore = current.gScore + neighbor.cost;
            Cost fScore =
                tentative_gScore + m_env.admissibleHeuristic(neighbor.state); 
            Cost focalHeuristic =
                current.focalHeuristic +
                m_env.focalStateHeuristic(neighbor.state, tentative_gScore) +
                m_env.focalTransitionHeuristic(current.state, neighbor.state,
                                               current.gScore,
                                               tentative_gScore);
						
            AstarNode tmp = AstarNode(neighbor.state, fScore, tentative_gScore, focalHeuristic);
            LLNodee prevv;
            prevv.par=tmp;
            prevv.neighbor=neighbor;
            prevv.gscore=tentative_gScore; 
            prevv.newnodeid=newnodeid ;
            prevv.c_idx= c_idx;
            prevv.hls_node= prev.hls_node;
            
            int curr= ts;
            //swarm::info("parent ts: %i",curr );
            //swarm::info("child ts: %i",1000*focalHeuristic + 100* fScore + tentative_gScore );
            //int time =std::max(1000*focalHeuristic + 100* fScore + tentative_gScore, curr);
            
            int tt= 100*focalHeuristic + fScore;
            
            if(tt>= swarm::superTimestamp()){ 
       
              //swarm::info("inside enqup enqup %i ::::%x", hlsnum,swarm::superTimestamp());
              swarm::enqueue(enq_upper_ll, tt, EnqFlags(PARENTDOMAIN | SAMETASK) , tmp,prevv,m_env, cameFrom, done);
            }
            else{
              swarm::deepen();
              swarm::enqueue(LL_pll, tt ,EnqFlags::NOHINT, tmp,prevv,m_env, cameFrom, done);  
            }
            
            //swarm::enqueue(LL_pll, fScore ,EnqFlags::NOHINT, tmp,prevv,m_env, cameFrom, done);
            

          // Best path for this AstarNode so far
          // TODO: this is not the best way to update "cameFrom", but otherwise
          // default c'tors of State and Action are required
          
        }
    }
      
    return;
}


#ifdef HLS_NO_POINTER
  void lls_search(swarm::Timestamp score, float m_w, const State* startState, 
			PlanResult* solution, HighLevelNode *hls_node, size_t c_idx) {
#else
  void lls_search(swarm::Timestamp score, float m_w, const State* startState, 
		  HighLevelNode *hls_node, size_t c_idx, int newnodeid){	//int newnodeid, size_t c_idx) {

		////HighLevelNode *hls_node = global_nodelist[newnodeid];
		//HighLevelNode *hls_node = global_nodelist[newnodeid].addr;
		PlanResult *solution = &hls_node->solution[c_idx];
#endif
    //swarm::info("ll deepen next : %i", score);
    LowLevelEnvironment m_env(g_env, c_idx, hls_node->constraints[c_idx], hls_node->solution);
    //came_from *** nextptr= new came_from [32][32][50];
    came_from * arryPtr = new came_from[60000]; //32x32x50
    bool *done = (bool*)malloc(sizeof(bool));
    *done = false;
    //came_from cameFrom[32][32][50];
    
    //std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,StateHasher> cameFrom;
		//LowLevelEnvironment m_env(g_env, c_idx, hls_node->constraints[c_idx], hls_node->solution);
		//solution->clear();
		//solution->add(StateCost(*startState, 0));

		
    //std::vector<Neighbor<State, Action, Cost> > neighbors;
    //neighbors.reserve(10);

    //Cost bestFScore = openSet[focalSet[0]].fScore;

    // std::cout << "new search" << std::endl;

    //while (!focalSet.empty()) { // [TODO] this is right ??? => NG , cameFrom
    AstarNode tmp = AstarNode(*startState, m_env.admissibleHeuristic(*startState), 0, 0);
    LLNodee prev;
    prev.par=tmp;
    prev.neighbor=Neighbor<State, Action, int>(*startState,Action::Wait, 0);
    prev.gscore=0;
    prev.newnodeid=newnodeid ;
    prev.c_idx= c_idx; 
    prev.hls_node = hls_node;
    //cameFrom[tmp.state.x][tmp.state.y][tmp.state.time][newnodeid].path_len=0;
    //prev.m_env = LowLevelEnvironment(g_env, c_idx, hls_node->constraints[c_idx], hls_node->solution);
		swarm::deepen(); 
    
		swarm::enqueue(LL_pll,0,EnqFlags::NOHINT, tmp,prev ,m_env, arryPtr,done);//&cameFrom
			// most good
			//node->timestamp = ROBOT_NUMS * node->generation + node->focalHeuristic;
	}
void star_t(swarm::Timestamp tt, int si,HighLevelNode *newNode){

  swarm::deepen();
  swarm::enqueue(main_loop_task_para,0 , EnqFlags::NOHINT, si, newNode);
}


#ifdef SWARM
	void search(swarm::Timestamp score) {
#else
	void search(void) {
#endif

		printf("[DEBUG] sizeof(HighLevelNode) = %lu\n", sizeof(start));
		printf("[DEBUG] sizeof(PlanResult) = %lu\n", sizeof(start.solution[0]));
		printf("[DEBUG] sizeof(StateCost) = %lu\n", sizeof(start.solution[0].states[0]));
		printf("[DEBUG] sizeof(ActionCost) = %lu\n", sizeof(start.solution[0].actions[0]));
		printf("[DEBUG] sizeof(HighLevelNode.solutions) = %lu\n", sizeof(start.solution));
		printf("[DEBUG] sizeof(Constraint) = %lu\n", sizeof(start.constraints[0]));
		printf("[DEBUG] sizeof(HighLevelNode.constraints) = %lu\n", sizeof(start.constraints));
    printf("[DEBUG] sizeof(camefrom) = %lu\n", sizeof(came_from)); 
    printf("[DEBUG] sizeof(LLNodee) = %lu\n", sizeof(LLNodee)); 
		//printf("please enter key...");
		//while(getchar() != '\n');

    start.cost = 0;
    start.LB = 0;
    start.id = 0;

    for (size_t i = 0; i < g_initialStates.size(); ++i) {
      
      if (i < ROBOT_NUMS && g_solution[i].states_size > 1) {
        //std::cout << g_initialStates[i] << " " << g_solution[i].states[0].state
        //          << std::endl;
        //assert(g_initialStates[i] == g_solution[i].states[0].state);
        start.solution[i] = g_solution[i];
        //std::cout << "use existing solution for agent: " << i << std::endl;
      } else {
        
        LowLevelEnvironment llenv(g_env, i, start.constraints[i],
                                  start.solution);
        //LowLevelSearch_t lowLevel(llenv, g_w);
        //bool success = lowLevel.search(g_initialStates[i], start.solution[i]);
        
        bool success = astar_search(llenv, g_w, g_initialStates[i], start.solution[i]);
        
        if (!success) {
					g_isNoConflict = false;
					return;
        }
      }
      
      
      start.cost += start.solution[i].cost;
      start.LB += start.solution[i].fmin;
      
    } // for
    swarm::info("start intiial soln after start");
    swarm::info("lls focl simm"); 
    start.focalHeuristic = focalHeuristic(start.solution);

#ifdef SWARM
		int si = index(start);
#ifdef HLS_NO_POINTER
		global_nodelist[si] = start;
#else
		//global_nodelist[si].addr = (struct HighLevelNode*)malloc(sizeof(struct HighLevelNode));
		//*global_nodelist[si].addr = start;
     HighLevelNode *newNode = (struct HighLevelNode*)malloc(sizeof(struct HighLevelNode));
     *newNode = start;
#endif
    //global_nodelist[si].addr->timestamp= 50;
		//swarm::enqueue(main_loop_task_para,0 , EnqFlags::NOHINT, si);
    swarm::enqueue(star_t,0 , EnqFlags::NOHINT, si, newNode);
#else
    auto handle = g_open.push(start);
    (*handle).handle = handle;
    g_focal.push(handle);
    bestCost = (*handle).cost;
		main_loop_task(0);
#endif
	}

#ifdef SWARM
	void main_loop_task_para(swarm::Timestamp score, int nodeid, HighLevelNode *newNodee ) {

		// Check done flag.
		 if(g_isNoConflict) {
			 swarm::info("After found solution, terminate all tasks : nodeid = %d", nodeid);
			 return;
		 }

#ifdef HLS_NO_PONTER
		 const HighLevelNode *P = &global_nodelist[nodeid];
#else
		 //const HighLevelNode *P = global_nodelist[nodeid];
		 //const HighLevelNode *P = global_nodelist[nodeid].addr;
     const HighLevelNode *P = newNodee;
     
#endif

		 // Check conflict
		 Conflict conflict;
		 if (!g_env.getFirstConflict(P->solution, conflict)) {
			 swarm::info("Found solution: nodeid: %d, cost: %d", nodeid, P->cost);
			 for(int i = 0; i < ROBOT_NUMS; ++i) {
				 g_solution[i] = P->solution[i];
			 }
			 g_isNoConflict = true;
			 return;
		 }

		 // create constraints
		 std::map<size_t, Constraints> constraints;
		 g_env.createConstraintsFromConflict(conflict, constraints);

		 // NOde expantion
		 int generation = P->generation + 1;
		 int child_id = 0;
     //swarm::info("hls node genrn");
     //int j=0;
		 for (const auto& c : constraints) {
			 size_t i = c.first;
			 int new_nodeid = index(nodeid, child_id) % 480 ;
       //int new_nodeid = 0;

			 if(new_nodeid > MAX_NODE_NUMS) {
				 swarm::info("WARNING : over MAX_NODE_NUMS. over %d", new_nodeid);
				 while(1);
			 }
#ifdef HLS_NO_POINTER
      swarm::info("no pointer");
			HighLevelNode *newNode = &global_nodelist[new_nodeid];
#else
			//global_nodelist[new_nodeid] = (struct HighLevelNode*)malloc(sizeof(struct HighLevelNode));
			//HighLevelNode *newNode = global_nodelist[new_nodeid];
			//global_nodelist[new_nodeid].addr = (struct HighLevelNode*)malloc(sizeof(struct HighLevelNode));
			//HighLevelNode *newNode = global_nodelist[new_nodeid].addr;
      HighLevelNode *newNode = (struct HighLevelNode*)malloc(sizeof(struct HighLevelNode));
#endif

			 //HighLevelNode newNode = P;
        //swarm::info("sol cost: %i nodenum: %i",newNode->cost, new_nodeid);
			 *newNode = *P;
			 newNode->id = new_nodeid;
			 newNode->generation = generation;
			 newNode->parent_child_id = P->own_child_id;
			 newNode->own_child_id = child_id;
			 newNode->parentTimestamp = P->timestamp;

			newNode->constraints[i].add(c.second);
			newNode->cost -= newNode->solution[i].cost;
			newNode->LB -= newNode->solution[i].fmin;

			// LowLevelSearch
			State *s = &g_initialStates[i];
      //swarm::info("lls serch inii: %i , i_con: %i", new_nodeid, i);
      //if (j==1) continue;
      //swarm::info("swarm enw %i" ,i); P->timestamp
			//swarm::enqueue(lls_search,P->timestamp ,EnqFlags::NOHINT, g_w, s, new_nodeid, i);/* &newNode->solution[i], */
      swarm::enqueue(lls_search,(2000+ P->timestamp) ,EnqFlags::NOHINT, g_w, s, newNode, i, new_nodeid); //2000
      
#ifdef HLS_NO_PONTER
          
					&global_nodelist[new_nodeid], i);
          swarm::info("no ptr");
#else
					//global_nodelist[new_nodeid], i);
#endif
      //swarm::info("lls serch after: parent ts : %i , ts: %i", score,P->timestamp);
			child_id++;
      //j++;
			if(child_id > 2) {
				swarm::info("ERROR");
				while(1);
			}

		 } // expand node

		 //free(global_nodelist[nodeid].addr);
		 //global_nodelist[nodeid].addr = NULL;
     free(newNodee);
     newNodee=NULL;
		 return;
	}
#endif

#ifdef NO_SWARM
	void main_loop_task(int n) {

		// loop
    int id = 1;
		int which_iter = 0;
		int generation = 0;
    while (!g_open.empty()) {
			generation++;
#ifdef HLS_INCREMENTAL_DEBUG
			while(getchar() != '\n');
#endif

#ifdef TRACE
			std::cout << "HLS iter : [" << which_iter << "]" << std::endl;
#endif
			which_iter++;
// check focal list/open list consistency

      auto h = g_focal.top();
      HighLevelNode P = *h;
			P.generation = generation;
      g_env.onExpandHighLevelNode(P.cost);
      // std::cout << "expand: " << P << std::endl;

      g_focal.pop();
      g_open.erase(h);

      Conflict conflict;
      if (!g_env.getFirstConflict(P.solution, conflict)) {
        std::cout << "done; cost: " << P.cost << std::endl;
				for (int i = 0; i < ROBOT_NUMS; ++i)
					g_solution[i] = P.solution[i];
				g_isNoConflict = true;
				return;
      }

      // create additional nodes to resolve conflict
#ifndef SWARM
      std::cout << "Found conflict: " << conflict << std::endl;
#endif
      // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
      // conflict.type << std::endl;

      std::map<size_t, Constraints> constraints;
      g_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
        // std::cout << "Add HL node for " << c.first << std::endl;
        size_t i = c.first;
#ifndef SWARM
        std::cout << "create child with id " << id << std::endl;
#endif
        HighLevelNode newNode = P;
        newNode.id = id;
        // (optional) check that this constraint was not included already
        // std::cout << newNode.constraints[i] << std::endl;
        // std::cout << c.second << std::endl;
        assert(!newNode.constraints[i].overlap(c.second));

        newNode.constraints[i].add(c.second);

        newNode.cost -= newNode.solution[i].cost;
        newNode.LB -= newNode.solution[i].fmin; 

        LowLevelEnvironment llenv(g_env, i, newNode.constraints[i],
                                  newNode.solution);
        LowLevelSearch_t lowLevel(llenv, g_w);
        bool success = lowLevel.search(g_initialStates[i], newNode.solution[i]);

        newNode.cost += newNode.solution[i].cost;
        newNode.LB += newNode.solution[i].fmin;
        newNode.focalHeuristic = g_env.focalHeuristic(newNode.solution);

        if (success) {
#ifndef SWARM
          std::cout << "  success. cost: " << newNode.cost << std::endl;
#endif
          auto handle = g_open.push(newNode);
            g_focal.push(handle);
        }

        ++id;
      }
    }

		return;
  }
#endif
