#include "LPAStarCPG.h"
#include "PRLStackUtilCPG.h"
#include "priority_queue.h"
#include <fstream>
#include <chrono>
#include <boost/lexical_cast.hpp>


std::istream & operator >>
(std::istream & s,
 Eigen::VectorXd & m)
{
    for (int i = 0; i < m.size(); ++i)
      s >> m(i);
  return s;
}


int main(int argc, char* argv[])
{
	po::options_description desc("Select File and Dimensions");
	desc.add_options()
		("help", "produce help message")
		("dimensions,d", po::value<int>()->required(), "set number of Dimensions")
		("input_graph,i",po::value<std::string>()->required(), " path to graphml file")
		("output_graph,o",po::value<std::string>()->required(), " path to graphml file")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc,argv,desc), vm);

	if(vm.count("help")) {
		std::cout<<desc<<std::endl;
		return 1;
	}

	std::string input_filepath=vm["input_graph"].as<std::string>();
	std::string output_filepath=vm["output_graph"].as<std::string>();
	int dim = vm["dimensions"].as<int>();

	Graph input_map;

	create_vertices(input_map,get(&VProp::state,input_map),input_filepath,dim,get(&EProp::prior,input_map));
	create_edges(input_map,get(&EProp::weight,input_map));
	VertexIter vi, vend;
	int i=0;
	for (boost::tie(vi, vend) = vertices(input_map); vi != vend; ++vi,++i)
	{
		put(&VProp::vertex_index,input_map,*vi,i);
	}

	std::cout<<"GRAPH LOADED! "<<std::endl;  


	VertexIter ui, uend;
	std::unordered_map<int,Vertex> morphNodeMap;  

	Graph herb_map=input_map;

	i=0;
	for (boost::tie(ui, uend) = vertices(input_map), boost::tie(vi, vend) = vertices(herb_map) ; vi != vend; ++vi,++ui,++i)
	{
		put(&VProp::vertex_index,herb_map,*vi,i);
		put(&VProp::state,herb_map,*vi,input_map[*ui].state);
	}

	std::cout<<" NEW GRAPH LOADED! "<<std::endl;

	size_t k = 20; // add k*2 number of edges 

	ROS_INFO("Starting ROS node.");
  	ros::init(argc, argv, "create_graph");
  	ros::NodeHandle nh("~");

  	// Create AIKIDO World
  	aikido::planner::WorldPtr env(new aikido::planner::World("world"));

  	// Load HERB either in simulation or real based on arguments
  	ROS_INFO("Loading HERB.");
  	herb::Herb robot(env, true);

  	MetaSkeletonStateSpacePtr rightArmSpace = robot.getRightArm()->getStateSpace();
	MetaSkeletonPtr rightArm = std::const_pointer_cast<MetaSkeleton>(robot.getRightArm()->getMetaSkeleton());

  	// MetaSkeletonStateSpacePtr rightArmSpace = robot.getLeftArm()->getStateSpace();
	// MetaSkeletonPtr rightArm = std::const_pointer_cast<MetaSkeleton>(robot.getLeftArm()->getMetaSkeleton());
	TestablePtr mSelfTestable = robot.getFullCollisionConstraint( rightArmSpace, rightArm, NULL);
	
	std::shared_ptr<GeodesicInterpolator> mInterpolator = std::make_shared<GeodesicInterpolator>(rightArmSpace);

	VPIndexMap indexMap = get(&VProp::vertex_index, herb_map);
	VPStateMap stateMap = get(&VProp::state, herb_map);
	EPWeightMap weightMap = get(&EProp::weight,herb_map);

	std::unordered_map<int,Vertex> nodeMap;

	VertexIter VI, VIEND, UI, UIEND;

	for (boost::tie(VI, VIEND) = vertices(herb_map); VI != VIEND ; ++VI)
	{
		nodeMap[indexMap[*VI]]=*VI;
	}
	std::cout<<"Press [ENTER] to create graph:";
	std::cin.get();
	priorityQueue pq;
	size_t num=0;

	for (boost::tie(VI, VIEND) = vertices(herb_map); VI != VIEND ; ++VI)
	{
		num++;
		if(num%100==0)
			std::cout<<num<<std::endl;
		pq.reset();
		for (boost::tie(UI, UIEND) = vertices(herb_map); UI != UIEND; ++UI)
		{
			if(indexMap[*VI]!=indexMap[*UI])
			{
				pq.insert(indexMap[*UI],(stateMap[*VI]-stateMap[*UI]).norm(),0.0);
			}
		}
		
		
		size_t j=0;
		while(j<k && pq.PQsize()!=0)
		{
			int index = pq.pop();
			// std::cout<<index<<" "<<std::endl;
			bool edge_exists = boost::edge(nodeMap[index], *VI, herb_map).second;


			if(!edge_exists)
			{
				Eigen::VectorXd source_config(7);
				Eigen::VectorXd target_config(7);

				source_config << herb_map[nodeMap[index]].state;
				target_config << herb_map[*VI].state;

				bool col =false;
		
				auto source_state = rightArmSpace->createState();
				rightArmSpace->convertPositionsToState(source_config, source_state);
				auto target_state = rightArmSpace->createState();
				rightArmSpace->convertPositionsToState(target_config, target_state);

				int qCount = 10;
				double step = 1.0/qCount;
				auto test_state = rightArmSpace->createState();

				for (double alpha = 0.0; alpha <= 1.0; alpha += step)
				{
					mInterpolator->interpolate(source_state, target_state, alpha, test_state);
					if (!mSelfTestable->isSatisfied(test_state))
					{
						col=true;
						// std::cout<<"faces coll!";
						break;
					}
				}
				if(!col)
				{
					j++;
					Edge curEdge;
					curEdge = (boost::add_edge(nodeMap[index], *VI, herb_map)).first;
					weightMap[curEdge]=(stateMap[*VI]-stateMap[nodeMap[index]]).norm();
				}
			}
			else
				j++;
			// if(pq.PQsize()==0)
			// 	std::cout<<j<<" empty!! "<<std::endl;
		}
	}

	std::cout<<"Press [ENTER] to display graph:";
	std::cin.get();

	std::cout<<"Average degree: "<<(boost::num_edges(herb_map))/(boost::num_vertices(herb_map))<<std::endl;

	display_graph(herb_map);

	write_graph(herb_map,output_filepath,dim);
	
	return 0;
}