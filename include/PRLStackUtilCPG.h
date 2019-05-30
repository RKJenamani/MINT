#include <iostream>
#include <Eigen/Dense>

#include <aikido/perception/AssetDatabase.hpp>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <magi/action/Action.hpp>
#include <magi/action/SequenceAction.hpp>
#include <magi/action/WaitForInputAction.hpp>
#include <pr_tsr/can.hpp>
#include <libherb/Herb.hpp>

namespace po = boost::program_options;

using RobotPtr = std::shared_ptr<herb::Herb>;


using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using dart::dynamics::SkeletonPtr;

using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;

using aikido::constraint::TestablePtr;


using aikido::constraint::dart::CollisionFree;
using aikido::constraint::dart::CollisionFreePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;

using aikido::statespace::GeodesicInterpolator;
using aikido::trajectory::Interpolated;
using aikido::trajectory::InterpolatedPtr;

using magi::action::ActionPtr;

class PRLStackUtilCPG
{
public:

	MetaSkeletonPtr mRightArm;
	MetaSkeletonPtr mLeftArm;

	MetaSkeletonStateSpacePtr mRightArmSpace;
	MetaSkeletonStateSpacePtr mLeftArmSpace;

	aikido::planner::WorldPtr mEnv;
	RobotPtr mRobot;

	CollisionFreePtr mLeftEnvTestable;
	TestablePtr mLeftFullTestable;

	CollisionFreePtr mRightEnvTestable;
	TestablePtr mRightFullTestable;

	std::shared_ptr<GeodesicInterpolator> mLeftInterpolator;
	std::shared_ptr<GeodesicInterpolator> mRightInterpolator;

	std::vector<SkeletonPtr> mObstacles;

	PRLStackUtilCPG()
	{
		waitForUser("Press [ENTER] to start constructor: ");
		char *argv[] = { "long", "live", "HERB" };
		int argc = 3;
		std::cout << "Starting ROS node." << std::endl;
		ros::init(argc, argv, "PRLStackUtilCPG");
		ros::NodeHandle nh("~");

		const std::string baseFrameName("map");
		const std::string topicName("dart_markers");
	  
		mEnv = aikido::planner::WorldPtr(new aikido::planner::World("world"));

		const auto resourceRetriever = std::make_shared<aikido::io::CatkinResourceRetriever>();
		
		ROS_INFO("Loading HERB in Simulation.");
		mRobot = std::move(std::make_shared<herb::Herb>(mEnv, true));
		
		//adding obstacles in world env

		SkeletonPtr table;
		Eigen::Isometry3d tablePose;
		perceiveObject(nh,resourceRetriever,table,tablePose,"table");

		SkeletonPtr shelf;
		Eigen::Isometry3d shelfPose;
		perceiveObject(nh,resourceRetriever,shelf,shelfPose,"shelf");

		SkeletonPtr can;
		Eigen::Isometry3d canPose;
		perceiveObject(nh,resourceRetriever,can,canPose,"can");

		// mObstacles.push_back(table);
		// mObstacles.push_back(shelf);
		mObstacles.push_back(can);

		// ROS_INFO_STREAM("Starting viewer. Please subscribe to the '"<<dartTopicName<<"'InteractiveMarker topic in rviz.");
		// aikido::rviz::WorldInteractiveMarkerViewer viewer(env, dartTopicName, baseFrameName);
		// viewer.setAutoUpdate(true);
		// waitForUser("Press [ENTER] to exit: ");

		mRightArmSpace = mRobot->getRightArm()->getStateSpace();
		mRightArm = std::const_pointer_cast<MetaSkeleton>(mRobot->getRightArm()->getMetaSkeleton());

		mLeftArmSpace = mRobot->getLeftArm()->getStateSpace();
		mLeftArm = std::const_pointer_cast<MetaSkeleton>(mRobot->getLeftArm()->getMetaSkeleton());


		CollisionDetectorPtr envDetector = dart::collision::FCLCollisionDetector::create();
		
		mLeftEnvTestable = std::make_shared<CollisionFree>(mLeftArmSpace, mLeftArm, envDetector);
		mRightEnvTestable = std::make_shared<CollisionFree>(mRightArmSpace, mRightArm, envDetector);


		//Collision group for left arm
		std::shared_ptr<CollisionGroup> rightArmGroup = envDetector->createCollisionGroup( mRobot->getLeftArm()->getMetaSkeleton().get(),
				mRobot->getLeftHand()->getMetaSkeleton().get());

		//Collision group for right arm
		std::shared_ptr<CollisionGroup> leftArmGroup = envDetector->createCollisionGroup( mRobot->getRightArm()->getMetaSkeleton().get(),
				mRobot->getRightHand()->getMetaSkeleton().get());

		//adding arms collision checking
		mLeftEnvTestable->addPairwiseCheck(rightArmGroup,leftArmGroup);

		//adding arms collision checking
		mRightEnvTestable->addPairwiseCheck(leftArmGroup,rightArmGroup);

		for (SkeletonPtr curObstacle : mObstacles)
		{
			std::shared_ptr<CollisionGroup> curObstacleGroup = envDetector->createCollisionGroup(curObstacle.get());
				
			mLeftEnvTestable->addPairwiseCheck(leftArmGroup, curObstacleGroup);
			mRightEnvTestable->addPairwiseCheck(rightArmGroup, curObstacleGroup);
		}

		mLeftFullTestable = mRobot->getFullCollisionConstraint( mLeftArmSpace, mLeftArm, mLeftEnvTestable);
		mLeftInterpolator = std::make_shared<GeodesicInterpolator>(mLeftArmSpace);

		mRightFullTestable = mRobot->getFullCollisionConstraint( mRightArmSpace, mRightArm, mRightEnvTestable);
		mRightInterpolator = std::make_shared<GeodesicInterpolator>(mRightArmSpace);
		ros::shutdown();
	}

	void waitForUser(const std::string& msg)
	{
	  std::cout << msg;
	  std::cin.get();
	}

	const SkeletonPtr makeBodyFromURDF(const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
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


	bool perceiveObject(ros::NodeHandle nh, const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
		dart::dynamics::SkeletonPtr& object, Eigen::Isometry3d& objectPose, std::string name)
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
			mRobot->getWorld()->addSkeleton(object);
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
			mRobot->getWorld()->addSkeleton(object);
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
			mRobot->getWorld()->addSkeleton(object);
			return true;
		}
		else
			return false;
	}

	void executeEdgeRight(ros::NodeHandle nh, Eigen::VectorXd & source_config, Eigen::VectorXd & target_config)
	{
		// waitForUser("Press [ENTER] to execute right hand: ");
		InterpolatedPtr traj = std::make_shared<Interpolated>(mRightArmSpace,mRightInterpolator);

		auto source_state = mRightArmSpace->createState();
		mRightArmSpace->convertPositionsToState(source_config, source_state);
		
		traj->addWaypoint(0, source_state);

		auto target_state = mRightArmSpace->createState();
		mRightArmSpace->convertPositionsToState(target_config, target_state);
		
		traj->addWaypoint(1, target_state);

		auto timedTraj = mRobot->retimePath(mRightArm, traj.get());

		mRobot->executeTrajectory(std::move(timedTraj)).wait();

		// std::cout << "[INFO]: Done Executing!" << std::endl;
	}

	void executeEdgeLeft(ros::NodeHandle nh, Eigen::VectorXd & source_config, Eigen::VectorXd & target_config)
	{
		// waitForUser("Press [ENTER] to execute left hand: ");
		InterpolatedPtr traj = std::make_shared<Interpolated>(mLeftArmSpace,mLeftInterpolator);

		auto source_state = mLeftArmSpace->createState();
		mLeftArmSpace->convertPositionsToState(source_config, source_state);
		
		traj->addWaypoint(0, source_state);

		auto target_state = mLeftArmSpace->createState();
		mLeftArmSpace->convertPositionsToState(target_config, target_state);
		
		traj->addWaypoint(1, target_state);

		auto timedTraj = mRobot->retimePath(mLeftArm, traj.get());

		mRobot->executeTrajectory(std::move(timedTraj)).wait();

		// std::cout << "[INFO]: Done Executing!" << std::endl;
	}

	void executePathCPG( std::vector<Eigen::VectorXd> & configs) 
	{
		
		char *argv[] = { "long", "live", "HERB" };
		int argc = 3;
		std::cout << "Starting ROS node." << std::endl;
		ros::init(argc, argv, "execute_viz");
		ros::NodeHandle nh("execute");

		const std::string baseFrameName("map");
		const std::string topicName("dart_markers");

		// Start the RViz viewer.
		std::cout << "Starting viewer. Please subscribe to the '" << topicName
							<< "' InteractiveMarker topic in RViz." << std::endl;
		aikido::rviz::WorldInteractiveMarkerViewer viewer(mEnv, topicName, baseFrameName);

		viewer.setAutoUpdate(true);

		waitForUser("Press [ENTER] to start Execution: ");

		size_t dim = (configs.at(0).size())/2;

		Eigen::VectorXd lStart(dim);
		lStart << configs.at(0).segment(0,dim);
		mLeftArm->setPositions(lStart);

		Eigen::VectorXd rStart(dim);
	  	rStart << configs.at(0).segment(dim,dim);
		mRightArm->setPositions(rStart);


		for(size_t i=0;i<configs.size()-1;i++)
		{
			Eigen::VectorXd source_config(dim+dim);
			Eigen::VectorXd target_config(dim+dim);

			source_config = configs.at(i);
			target_config = configs.at(i+1);

			Eigen::VectorXd left_source(dim);
			Eigen::VectorXd left_target(dim);
					
			Eigen::VectorXd right_source(dim);
			Eigen::VectorXd right_target(dim);

			left_source << source_config.segment(0,dim);
			right_source << source_config.segment(dim,dim);

			left_target << target_config.segment(0,dim);
			right_target << target_config.segment(dim,dim);

			if(left_source==left_target)
			{
				std::cout<<"Right Arm movement! "<<std::endl;
				executeEdgeRight(nh,right_source,right_target);
			}
			else
			{
				std::cout<<"Left Arm movement! "<<std::endl;
				executeEdgeLeft(nh,left_source,left_target);
			}
		}

		std::cout << "[INFO]: Done Executing!" << std::endl;
		ros::shutdown();
	}
};