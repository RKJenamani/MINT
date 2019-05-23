#include "PRLStackUtil.h"
#include "LPAStar.h"

void LPAStar::insertPQ(Graph &g, Vertex &node, pair<double,double> priority)
{
	VPIndexMap index_map = get(&VProp::vertex_index, g);
	pq.insert(index_map[node],priority.first,priority.second);
	nodeMap[index_map[node]] = node;
}
void LPAStar::removePQ(Graph &g, Vertex &node)
{
	VPIndexMap index_map = get(&VProp::vertex_index, g);
	pq.remove(index_map[node]);
}
bool LPAStar::isemptyPQ()
{
	return pq.PQsize()==0;
}
bool LPAStar::containsPQ(Graph &g, Vertex &node)
{
	VPIndexMap index_map = get(&VProp::vertex_index, g);
	return pq.contains(index_map[node]);
}
pair <double,double> LPAStar::topKeyPQ()
{
	return pq.topKey();
}
Vertex LPAStar::popPQ(Graph &g)
{
	int top_index = pq.pop();
	return nodeMap[top_index];
}

void LPAStar::initialize(Graph & g, Vertex & _start, Vertex & _goal)
{
	pq.reset();
	nodeMap.clear();
	
	start=_start;
	goal=_goal;
	VertexIter vi,viend;
	for( boost::tie(vi,viend) = vertices(g); vi!=viend;++vi)
	{
		put(&VProp::gValue,g,*vi,INF_VAL);
		put(&VProp::rhsValue,g,*vi,INF_VAL);
	}
	put(&VProp::rhsValue,g,start,0);
	insertPQ(g,start,calculateKey(g,start));
}

std::vector<Vertex> LPAStar::computeShortestPath(Graph &g, double &cost)
{
	VPrhsMap rhsMap = get(&VProp::rhsValue,g);
	VPgMap gMap = get(&VProp::gValue,g);
	VPIndexMap index_map = get(&VProp::vertex_index, g);
	// std::cout<<(topKeyPQ()<calculateKey(g,goal))<<" "<< (rhsMap[goal] != gMap[goal])<<std::endl;
	while(topKeyPQ()<calculateKey(g,goal) || get(&VProp::rhsValue,g,goal) != get(&VProp::gValue,g,goal) )
	{
		Vertex node = popPQ(g);

		// pq.printPQ();
		// std::cout<<"Node:"<<index_map[node]<<std::endl;
		if( get(&VProp::rhsValue,g,node) < get(&VProp::gValue,g,node))
		{
			put(&VProp::gValue,g,node,rhsMap[node]);
			std::vector<Vertex> successors = getSuccessors(g,node);
			for (auto successor : successors)
			{
				// std::cout<<index_map[successor]<<"<-if ";
				updateNode(g,successor);
			}
		}
		else
		{
			put(&VProp::gValue,g,node,INF_VAL);
			updateNode(g,node);
			std::vector<Vertex> successors = getSuccessors(g,node);
			for (auto successor : successors )
			{
				// std::cout<<index_map[successor]<<"<-else ";
				updateNode(g,successor);
			}
		}
		// pq.printPQ();
		// std::cout<<std::endl;;
	}

	return findPath(g,cost);

	// return std::vector< Vertex>();
}

void LPAStar::updateNode(Graph &g, Vertex & node)
{
	VPrhsMap rhsMap = get(&VProp::rhsValue,g);
	VPgMap gMap = get(&VProp::gValue,g);
	EPWeightMap weightMap = get(&EProp::weight,g); 		
	if(node!=start)
	{
		std::vector<Vertex> predecessors = getPredecessors(g,node);
		double min_rhs = INF_VAL;
		for (auto predecessor : predecessors )
		{
			Edge currEdge = boost::edge(predecessor,node,g).first;
			min_rhs=min(min_rhs,gMap[predecessor]+weightMap[currEdge]);
		}
		// std::cout<<std::endl<<min_rhs<<std::endl;
		put(&VProp::rhsValue,g,node,min_rhs);
		if(containsPQ(g,node))
			removePQ(g,node);
		if(get(&VProp::rhsValue,g,node) != get(&VProp::gValue,g,node))
		{
			// std::cout<<"inserted"<<std::endl;
			insertPQ(g,node,calculateKey(g,node));
		}
	}
}

double LPAStar::getHeuristic(Graph &g, Vertex & u, Vertex &v)
{
	VPStateMap stateMap = get(&VProp::state,g);
	return (stateMap[u]-stateMap[v]).norm();
}

std::vector <Vertex> LPAStar::getPredecessors(Graph &g, Vertex &v)
{
	std::vector<Vertex> predecessors;
	InEdgeIter ei, ei_end;

	for (tie(ei, ei_end) = in_edges(v, g); ei != ei_end; ++ei) 
	{
    	Vertex curPred = source(*ei, g);
    	predecessors.push_back(curPred);
	}

  return predecessors;
}
std::vector <Vertex> LPAStar::getSuccessors(Graph &g, Vertex &v)
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



pair<double,double> LPAStar::calculateKey(Graph &g, Vertex &node)
{
	VPrhsMap rhsMap = get(&VProp::rhsValue,g);
	VPgMap gMap = get(&VProp::gValue,g);

	pair <double,double> p;
	p.first = min(rhsMap[node],gMap[node]) + getHeuristic(g,node,goal); //add L2 norm here!!!
	p.second = min(rhsMap[node],gMap[node]);
	return p;
}

std::vector <Vertex> LPAStar::findPath(Graph &g, double &cost)
{	
	VPgMap gMap = get(&VProp::gValue,g);
	VPrhsMap rhsMap = get(&VProp::rhsValue,g);
	EPWeightMap weightMap = get(&EProp::weight,g);
	if (rhsMap[goal]==INF_VAL)
		return std::vector< Vertex>();

	std::vector<Vertex> finalPath;

	finalPath.push_back(goal);

	Vertex node = goal;
	Vertex new_node;
	cost=0;
	while(node!=start)
	{	
		std::vector<Vertex> predecessors = getPredecessors(g,node);
		double min_val = INF_VAL;
		double min_weight;
		for (auto predecessor : predecessors )
		{
			Edge currEdge = boost::edge(predecessor,node,g).first;
			if(min_val>gMap[predecessor]+weightMap[currEdge])
			{
				min_val=gMap[predecessor]+weightMap[currEdge];
				min_weight=weightMap[currEdge];
				new_node = predecessor;
			}
		}
		cost+=min_weight;
		finalPath.push_back(new_node);
		node=new_node;
	}
	std::reverse(finalPath.begin(), finalPath.end());

	return finalPath;
}

//////////////////////////LPAStar done

void PRLStackUtil::waitForUser(const std::string& msg)
{
  std::cout << msg;
  std::cin.get();
}

const SkeletonPtr PRLStackUtil::makeBodyFromURDF(const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
	const std::string& uri, const Eigen::Isometry3d& transform)
{
	dart::utils::DartLoader urdfLoader;
	const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

	if (!skeleton)
		throw std::runtime_error("unable to load '" + uri + "'");

	dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
			->setTransform(transform);
	return skeleton;
}


bool PRLStackUtil::perceiveObject(ros::NodeHandle nh, const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
	herb::Herb& robot, dart::dynamics::SkeletonPtr& object, Eigen::Isometry3d& objectPose, std::string name)
{
	if(name=="table")
	{
		const std::string objectURDFUri("package://pr_assets/data/furniture/uw_demo_table.urdf");
		// Poses for table
		objectPose = Eigen::Isometry3d::Identity();
		// objectPose.translation() = Eigen::Vector3d(1.0, 0.4, 0);
		objectPose.translation() = Eigen::Vector3d(1.0, 0.0, 0);
		Eigen::Matrix3d rot;
		rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
				* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
				* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
		objectPose.linear() = rot;
		// Load table
		object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
		// Add all objects to World
		robot.getWorld()->addSkeleton(object);
		return true;
	}
	else if(name=="shelf")
	{
		const std::string objectURDFUri("package://pr_assets/data/furniture/bookcase.urdf");
		objectPose = Eigen::Isometry3d::Identity();
		objectPose.translation() = Eigen::Vector3d(-0.0, 0.75, 0);
		Eigen::Matrix3d rot;
		rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
					* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
		objectPose.linear() = rot;
		// Load table
		object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
		// Add all objects to World
		robot.getWorld()->addSkeleton(object);
		return true;
	}
	else if(name=="can")
	{
		const std::string objectURDFUri("package://pr_assets/data/objects/can.urdf");
		objectPose = Eigen::Isometry3d::Identity();
		objectPose.translation() = Eigen::Vector3d(1.0, -0.3, 0.7);
		Eigen::Matrix3d rot;
		rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
					* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
		objectPose.linear() = rot;
		// Load can
		object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
		// Add all objects to World
		robot.getWorld()->addSkeleton(object);
		return true;
	}
	else
		return false;
}


PRLStackUtil::PRLStackUtil(const std::string& arm_name)
{
	waitForUser("Press [ENTER] to exit: ");
	std::cout<<"K"<<std::endl;
	char *argv[] = { "long", "live", "HERB" };
	int argc = 3;
	std::cout << "Starting ROS node." << std::endl;
	ros::init(argc, argv, "PRLStackUtil");
	ros::NodeHandle nh("~");

	const std::string baseFrameName("map");
	const std::string topicName("dart_markers");
  
	env = aikido::planner::WorldPtr(new aikido::planner::World("world"));

	const auto resourceRetriever
	 = std::make_shared<aikido::io::CatkinResourceRetriever>();
	ROS_INFO("Loading HERB.");
	herb::Herb robot(env,true);
	
	//adding obstacles in world env

	SkeletonPtr table;
	Eigen::Isometry3d tablePose;
	perceiveObject(nh,resourceRetriever,robot,table,tablePose,"table");

	// SkeletonPtr shelf;
	// Eigen::Isometry3d shelfPose;
	// perceiveObject(nh,resourceRetriever,robot,shelf,shelfPose,"shelf");

	SkeletonPtr can;
	Eigen::Isometry3d canPose;
	perceiveObject(nh,resourceRetriever,robot,can,canPose,"can");

	obstacles.push_back(table);
	// obstacles.push_back(shelf);
	obstacles.push_back(can);

	// ROS_INFO_STREAM("Starting viewer. Please subscribe to the '"<<dartTopicName<<"'InteractiveMarker topic in rviz.");
	// aikido::rviz::WorldInteractiveMarkerViewer viewer(env, dartTopicName, baseFrameName);
	// viewer.setAutoUpdate(true);
	// waitForUser("Press [ENTER] to exit: ");
	// // auto leftArmSpace = robot.getLeftArm()->getStateSpace();

	if(arm_name=="right")
	{
		armSpace = robot.getRightArm()->getStateSpace();

		// Eigen::VectorXd lRelaxedHome(7);
		// lRelaxedHome << 0.64, -1.50, 0.26, 1.96, 1.16, 0.87, 1.43;
		staticArm = std::const_pointer_cast<MetaSkeleton>(robot.getLeftArm()->getMetaSkeleton());
		// leftArm->setPositions(lRelaxedHome);
		dynamicArm = std::const_pointer_cast<MetaSkeleton>(robot.getRightArm()->getMetaSkeleton());
		// Eigen::VectorXd mStartConfig(7);
		// mStartConfig << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;
		
		mSelfTestable = robot.getFullCollisionConstraint(armSpace, dynamicArm, NULL);
		CollisionDetectorPtr envDetector = dart::collision::FCLCollisionDetector::create();
		mEnvTestable = std::make_shared<CollisionFree>(armSpace, dynamicArm, envDetector);
		std::shared_ptr<CollisionGroup> dynamicArmGroup = envDetector->createCollisionGroup( robot.getRightArm()->getMetaSkeleton().get(),
			robot.getRightHand()->getMetaSkeleton().get());

		std::shared_ptr<CollisionGroup> staticArmGroup = envDetector->createCollisionGroup( robot.getLeftArm()->getMetaSkeleton().get(),
			robot.getLeftHand()->getMetaSkeleton().get());
		
		mEnvTestable->addPairwiseCheck(dynamicArmGroup,staticArmGroup);

		for (SkeletonPtr curObstacle : obstacles)
		{
			std::shared_ptr<CollisionGroup> curObstacleGroup = envDetector->createCollisionGroup(curObstacle.get());
			mEnvTestable->addPairwiseCheck(dynamicArmGroup, curObstacleGroup);
		}

		mFullTestable = robot.getFullCollisionConstraint( armSpace, dynamicArm, mEnvTestable);

		mInterpolator = std::make_shared<GeodesicInterpolator>(armSpace);
	}
	else if(arm_name=="left")
	{
		armSpace = robot.getLeftArm()->getStateSpace();

		// Eigen::VectorXd lRelaxedHome(7);
		// lRelaxedHome << 0.64, -1.50, 0.26, 1.96, 1.16, 0.87, 1.43;
		dynamicArm = std::const_pointer_cast<MetaSkeleton>(robot.getLeftArm()->getMetaSkeleton());
		// leftArm->setPositions(lRelaxedHome);
		staticArm = std::const_pointer_cast<MetaSkeleton>(robot.getRightArm()->getMetaSkeleton());
		// Eigen::VectorXd mStartConfig(7);
		// mStartConfig << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;
		
		mSelfTestable = robot.getFullCollisionConstraint(armSpace, dynamicArm, NULL);
		CollisionDetectorPtr envDetector = dart::collision::FCLCollisionDetector::create();
		mEnvTestable = std::make_shared<CollisionFree>(armSpace, dynamicArm, envDetector);
		std::shared_ptr<CollisionGroup> dynamicArmGroup = envDetector->createCollisionGroup( robot.getLeftArm()->getMetaSkeleton().get(),
			robot.getLeftHand()->getMetaSkeleton().get());
		std::shared_ptr<CollisionGroup> staticArmGroup = envDetector->createCollisionGroup( robot.getRightArm()->getMetaSkeleton().get(),
			robot.getRightHand()->getMetaSkeleton().get());


		mEnvTestable->addPairwiseCheck(dynamicArmGroup,staticArmGroup );

		for (SkeletonPtr curObstacle : obstacles)
		{
			std::shared_ptr<CollisionGroup> curObstacleGroup = envDetector->createCollisionGroup(curObstacle.get());
			mEnvTestable->addPairwiseCheck(dynamicArmGroup, curObstacleGroup);
		}


		mFullTestable = robot.getFullCollisionConstraint( armSpace, dynamicArm, mEnvTestable);

		mInterpolator = std::make_shared<GeodesicInterpolator>(armSpace);
	}
	else
		std::cout<<" [INFO:] ERROR! NO ARM (left/right) SPECIFIED IN CONSTRUCTOR "<<std::endl;

	ros::shutdown();
}

bool PRLStackUtil::getCollisionStatus(Eigen::VectorXd &start, Eigen::VectorXd &end)
{
	// waitForUser("13Press [ENTER] to exit: ");
	bool col =false;
	auto startState = armSpace->createState();
	armSpace->convertPositionsToState(start, startState);
	// waitForUser("13Press [ENTER] to exit: ");
	auto endState = armSpace->createState();
	armSpace->convertPositionsToState(end, endState);

	int qCount = 10;
	double step = 1.0/qCount;
	auto testState = armSpace->createState();
	// waitForUser("13Press [ENTER] to exit: ");
	for (double alpha = 0.0; alpha <= 1.0; alpha += step)
	{
		mInterpolator->interpolate(startState, endState, alpha, testState);
		if (!mFullTestable->isSatisfied(testState))
		{
			col=true;
		}
	}
	// waitForUser("13Press [ENTER] to exit: ");

	return col;

}

///////////////////

void PRLStackUtil::executeSim(RobotPtr robot, MetaSkeletonPtr arm, aikido::planner::WorldPtr world,
	const InterpolatedPtr& untimedTraj, std::vector<SkeletonPtr> obstacles) 
{
	// Make sure obstacles show up.
	for (auto obj : obstacles)
	{
		world->addSkeleton(obj);
	}

	auto timedTraj = robot->retimePath(arm, untimedTraj.get());

	// ROS/RViz set-up.
	char *argv[] = { "long", "live", "HERB" };
	int argc = 3;
	std::cout << "Starting ROS node." << std::endl;
	ros::init(argc, argv, "setup_opt_viz");
	ros::NodeHandle nh("~");

	const std::string baseFrameName("map");
	const std::string topicName("dart_markers");

	// Start the RViz viewer.
	std::cout << "Starting viewer. Please subscribe to the '" << topicName
						<< "' InteractiveMarker topic in RViz." << std::endl;
	aikido::rviz::WorldInteractiveMarkerViewer viewer(world, topicName, baseFrameName);

	viewer.setAutoUpdate(true);

	Eigen::VectorXd rRelaxedHome(7);
  	rRelaxedHome << 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;
	arm->setPositions(rRelaxedHome);

	// Let's go!
	int userIn;
	std::cout << "[soda_handoff] Enter any number to execute: ";
	std::cin >> userIn;

	robot->executeTrajectory(std::move(timedTraj)).wait();

	std::cout << "[soda_handoff] Enter any number to exit: ";
	std::cin >> userIn;
}

InterpolatedPtr PRLStackUtil::genTrajFromConfigs( std::vector<Eigen::VectorXd>& trajConfigs,
	MetaSkeletonStateSpacePtr armStateSpace, std::shared_ptr<GeodesicInterpolator> interpolator) 
{

	if (trajConfigs.size() == 0)
		return nullptr;

	InterpolatedPtr traj = std::make_shared<Interpolated>(
		armStateSpace,
		interpolator);

	for (int i = 0; i < trajConfigs.size(); i++)
	{
		auto& config = trajConfigs.at(i);
		auto state = armStateSpace->createState();
		armStateSpace->convertPositionsToState(config, state);

		traj->addWaypoint(i, state);
	}

	return traj;
}

void PRLStackUtil::executePath(std::vector<Eigen::VectorXd> & configs)
{
		// Convert the C-Space path to an AIKIDO trajectory and execute it in sim.

	InterpolatedPtr traj = genTrajFromConfigs(configs, armSpace, mInterpolator);

  // Doing this more than one is frowny face because AIKIDO worlds.
	RobotPtr k = std::make_shared<herb::Herb>(env,true);
	executeSim(k,dynamicArm,env,traj,obstacles);
	std::cout << "[INFO]: Done Executing!" << std::endl;
}


/////////////////

Vertex connect_to_graph(Graph &g, Eigen::VectorXd &config, int &k)
{
	VPIndexMap indexMap = get(&VProp::vertex_index, g);
	VPStateMap stateMap = get(&VProp::state, g);
	EPWeightMap weightMap = get(&EProp::weight,g);

	int i=0;
	VertexIter vi,viend;
	for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
		i++;

	Vertex node;
	node = add_vertex(g);
	stateMap[node] = config;
	indexMap[node] = i;

	std::unordered_map<int,Vertex> nodeMap;

	nodeMap[i]=node;

	priorityQueue pq;

	for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi)
	{
		if(indexMap[*vi]!=i)
		{
			// std::cout<<"Inserting "<<indexMap[node]<<std::endl;
			pq.insert(indexMap[*vi],(stateMap[node]-stateMap[*vi]).norm(),0.0);
			nodeMap[indexMap[*vi]]=*vi;
		}
	}
	// std::cout<<std::endl<<"Priority Queue: ";
	// pq1.printPQ();
	std::cout<<"Nodes connected to config: ";
	for(int j=0;j<k;j++)
	{
		int index = pq.pop();
		std::cout<<index<<" ";
		Edge curEdge;
		curEdge = (add_edge(nodeMap[index], node, g)).first;
		weightMap[curEdge]=(stateMap[node]-stateMap[nodeMap[index]]).norm();
	}
	std::cout<<std::endl;

	return node;
}

///////////////////

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
	Vertex temp1,temp2;
	// int start_index=8;
	// int end_index=25;
	// std::cout<<"Input start node index: ";
	// std::cin>>start_index;
	// std::cout<<"Input goal node index: ";
	// std::cin>>end_index;
	// std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n');

	create_vertices(herb_map,get(&VProp::state,herb_map),filepath,dim,get(&EProp::prior,herb_map));
	create_edges(herb_map,get(&EProp::weight,herb_map));
	VertexIter vi, vend;
	int i=0;
  	for (boost::tie(vi, vend) = vertices(herb_map); vi != vend; ++vi,++i)
  	{
  		put(&VProp::vertex_index,herb_map,*vi,i);
  		if(i==8)
  			temp1=*vi;
  		if(i==25)
  			temp2=*vi;
  	}

  	std::cout<<"GRAPH LOADED! "<<std::endl;
	// display_graph(herb_map);

	Eigen::VectorXd start_config(7);
	start_config << 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;

	Eigen::VectorXd goal_config(7);
	goal_config << 4.42475, -0.900109, -0.672778, 2.29866, -0.792, -1.25398, 0.823374;

	int k=20;

	std::cout<<"Start config :"<<start_config<<std::endl;
	start = connect_to_graph(herb_map ,start_config, k);

	std::cout<<"Goal config :"<<goal_config<<std::endl;
	end = connect_to_graph(herb_map ,goal_config, k);



	VPIndexMap indexMap = get(&VProp::vertex_index, herb_map);

	std::cout<<"Index: "<<indexMap[start]<<" "<<indexMap[end]<<std::endl;

	EPWeightMap weightMap = get(&EProp::weight,herb_map);

	LPAStar a;

	double pathcost;
	std::vector<Vertex> path;

	PRLStackUtil checker(std::string("right"));

	a.initialize(herb_map,start,end);
	int numSearches = 0;

	std::cout<<"Input [ENTER] to start search: ";
  	std::cin.get();

	while(true)
	{
		numSearches++;
		EPEvaluatedMap edgeEvalMap = get(&EProp::evaluated,herb_map);

		VPIndexMap index_map = get(&VProp::vertex_index, herb_map);

		std::cout<< "[INFO]: Search "<<numSearches <<std::endl;
		std::vector<Vertex> shortestPath = a.computeShortestPath(herb_map,pathcost);
		// for(Vertex nodes: shortestPath )
		// 	std::cout<<index_map[nodes]<<" ";
		if( shortestPath.size()==0)
		{
			std::cout << "" <<std::endl;
			std::cout<< "[INFO]: ALL COLL" <<std::endl;
			path=std::vector<Vertex>();
			break;
		}

		bool collisionFree = true;
		for( int i=0;i<shortestPath.size()-1;i++)
		{
			Vertex curU = shortestPath.at(i);
			Vertex curV = shortestPath.at(i+1);

			Edge curEdge = boost::edge(curU,curV,herb_map).first;

			bool curEval = edgeEvalMap[curEdge];

			// std::cout<<std::endl<<"Eval: "<<curEval;

			if(!curEval)
			{
				edgeEvalMap[curEdge] = true;

				bool col= checker.getCollisionStatus(herb_map[curU].state ,herb_map[curV].state);
				// std::cout<<" Col: "<<col;
				if(col)
				{
					put(&EProp::weight,herb_map,curEdge,INF_VAL);
					a.updateNode(herb_map,curU);
					a.updateNode(herb_map,curV);

					collisionFree=false;
					break;
				}
			}
		}
		if(collisionFree)
		{
			std::cout<<"Input [ENTER] to display path: ";
  			std::cin.get();
			std::cout<<std::endl<<"Shortest Path is of size:"<<shortestPath.size()<< " nodes"<<std::endl;

			std::cout<<std::endl<<"Shortest Path is of length:"<<pathcost<<std::endl;
			
			std::cout<<"Path: ";
			for(Vertex nodes: shortestPath )
			{
				std::cout<<index_map[nodes]<<" ";
			}
			std::cout<<std::endl<<"Input [ENTER] to execute path: ";
  			std::cin.get();

  			std::vector<Eigen::VectorXd> configs;

			for(auto vi=shortestPath.begin(); vi!=shortestPath.end();vi++)
				configs.push_back(herb_map[*vi].state);

			checker.executePath(configs);

			std::cout<<std::endl<<"Input [ENTER] to exit: ";
  			std::cin.get();
			break;
		}
	}

	return 0;
}