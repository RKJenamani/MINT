#include "Dijkstra.h"


Vertex Dijkstra::minDistance(Graph &g)
{
	double min_dist= INF_VAL;
	Vertex min_vertex;
	VertexIter vi,viend;
	for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
  	{
  		if(!sptSet.count(*vi) && distanceMap[*vi]<min_dist)
  		{
  			min_dist = distanceMap[*vi];
  			min_vertex = *vi;
  		}
  	}

  	// VPIndexMap index_map = get(&VProp::vertex_index, g);
  	// std::cout<<std::endl<<"Distance Map: ";
  	// for(auto it = distanceMap.begin(); it != distanceMap.end(); ++it)
   //  	std::cout << index_map[it->first] << "-" << it->second <<"   ";
   //  std::cout<<std::endl<<"sptSet: ";
   //  for(auto it = sptSet.begin(); it != sptSet.end(); ++it)
   //  	std::cout << index_map[it->first] << "-" << it->second <<"   ";

  	return min_vertex;	
}

std::vector <Vertex> Dijkstra::getSuccessors(Graph &g, Vertex &v)
{
	std::vector<Vertex> successors;
	OutEdgeIter ei, ei_end;

	for (tie(ei, ei_end) = out_edges(v, g); ei != ei_end; ++ei) 
	{
    	Vertex curPred = target(*ei, g);
    	successors.push_back(curPred);
	}

  return successors;
}

double Dijkstra::getDistance(Graph &g, Vertex & u, Vertex &v)
{
	VPStateMap stateMap = get(&VProp::state,g);
	return (stateMap[u]-stateMap[v]).norm();
}

std::vector<Vertex> Dijkstra::computeShortestPath(Graph &g, Vertex & _start, Vertex & _goal, double & cost)
{ 
	Vertex start=_start;
	Vertex goal=_goal;
	VertexIter vi, viend;
	int numVertices=0;
  	for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
  	{
  		numVertices++;
  		distanceMap[*vi]=INF_VAL;
  	}
  	parentMap.clear();
  	sptSet.clear();
  	distanceMap[start]=0;
  	int totalVertices=numVertices+1;
  	while(numVertices>1)
  	{
  		std::cout<< "[INFO]: Search "<<totalVertices-numVertices<<std::endl;
  		Vertex node = minDistance(g);


  		sptSet[node]=1;

  		if(node==goal)
  			break;

  		std::vector <Vertex> successors = getSuccessors(g,node);

  		for(Vertex successor : successors )
  		{
  			if( !sptSet.count(successor) && (distanceMap[successor] > distanceMap[node] + getDistance(g,node,successor)) )
  			{
  				distanceMap[successor] = distanceMap[node] + getDistance(g,node,successor);
  				parentMap[successor]=node;
  			}
  		}
  		numVertices--;
  	}

  	return findPath(g,start,goal,cost);

}

std::vector<Vertex> Dijkstra::findPath(Graph &g,Vertex & start, Vertex & goal, double &cost)
{
	cost=0;
	if(sptSet.count(goal)==0)
	{
		return std::vector<Vertex>();
	}
	
	std::vector<Vertex> path;
	path.push_back(goal);
	Vertex node=goal;
	Vertex new_node;
	while(node!=start)
	{
		new_node=parentMap[node];
		cost+=getDistance(g,new_node,node);
		node=new_node;
		path.push_back(node);
	}
	std::reverse(path.begin(),path.end());

	return path;
}


int main(int argc, char* argv[])
{
	po::options_description desc("Select File and Dimensions");
	desc.add_options()
		("help", "produce help message")
		("dimensions,d", po::value<int>()->required(), "set number of Dimensions")
		("graph,g",po::value<std::string>()->required(), " path to graphml file");
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc,argv,desc), vm);

	if(vm.count("help")) {
		std::cout<<desc<<std::endl;
		return 1;
	}

	std::string filepath=vm["graph"].as<std::string>();
	int dim = vm["dimensions"].as<int>();

	Graph herb_map;
	Vertex start,end;

	int start_index=8;
	int end_index=25;
	std::cout<<"Input start node index: ";
	std::cin>>start_index;
	std::cout<<"Input goal node index: ";
	std::cin>>end_index;
	std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n');

	create_vertices(herb_map,get(&VProp::state,herb_map),filepath,dim,get(&EProp::prior,herb_map));
	create_edges(herb_map,get(&EProp::weight,herb_map));
	VertexIter vi, vend;
	int i=0;
  	for (boost::tie(vi, vend) = vertices(herb_map); vi != vend; ++vi,++i) {
  		put(&VProp::vertex_index,herb_map,*vi,i);
  		if (i==8)
  			start=*vi;
  		if (i==25)
  			end=*vi;
  	}


	// display_graph(herb_map);
	std::cout<<" Graph loaded!"<<std::endl;


	Dijkstra a;
	double pathcost;
	std::vector<Vertex> path;

	std::cout<<"Input [ENTER] to start search: ";
  	std::cin.get();

	path = a.computeShortestPath(herb_map,start,end,pathcost);
	std::cout<<"PathCost: "<<pathcost<<std::endl;
	VPIndexMap index_map = get(&VProp::vertex_index, herb_map);
	std::cout<<"path: ";
	for(auto nodes: path)
	{
		std::cout<<index_map[nodes]<<" ";
	}
}