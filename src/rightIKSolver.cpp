#include <iostream>
#include <Eigen/Dense>
#include <libherb/Herb.hpp>

#include <dart/dart.hpp>

#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/distance/NominalConfigurationRanker.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pr_tsr/can.hpp>

//SODA with respect to table
#define POSITIVE_X -0.20
#define POSITIVE_Y 0.6
#define POSITIVE_Z 0.73

using aikido::constraint::dart::createSampleableBounds;
using aikido::constraint::dart::InverseKinematicsSampleable;
using aikido::distance::NominalConfigurationRanker;
using aikido::statespace::dart::MetaSkeletonStateSaver;



using dart::dynamics::BodyNodePtr;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::InverseKinematicsPtr;
using dart::common::make_unique;

using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using dart::dynamics::ChainPtr;

using aikido::statespace::GeodesicInterpolator;
using aikido::constraint::TestablePtr;
using aikido::constraint::dart::CollisionFree;
using aikido::constraint::dart::CollisionFreePtr;
using aikido::statespace::StateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::trajectory::Interpolated;
using aikido::trajectory::InterpolatedPtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using RobotPtr = std::shared_ptr<herb::Herb>;

const SkeletonPtr makeBodyFromURDF(const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
	const std::string& uri, const Eigen::Isometry3d& transform) 
{
	dart::utils::DartLoader urdfLoader;
	const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

	if (!skeleton)
	throw std::runtime_error("unable to load '" + uri + "'");

	dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))->setTransform(transform);
	return skeleton;
}

std::vector<SkeletonPtr> loadHERBExample(Eigen::Isometry3d& tablePoseOut)
{
	std::vector<SkeletonPtr> loadedObstacles;

	const auto resourceRetriever = std::make_shared<aikido::io::CatkinResourceRetriever>();

	const std::string tableURDFUri("package://pr_assets/data/furniture/table.urdf");
	const std::string shelfURDFUri("package://pr_assets/data/furniture/bookcase.urdf");
	const std::string pitcherURDFUri("package://pr_assets/data/objects/rubbermaid_ice_guard_pitcher.urdf");
	const std::string roofURDFUri("package://pr_assets/data/furniture/table.urdf");

	// Initial perception
	SkeletonPtr table;
	Eigen::Isometry3d tablePose;
	SkeletonPtr shelf;
	Eigen::Isometry3d shelfPose;

	SkeletonPtr pitcher;
	Eigen::Isometry3d pitcherPose;
	SkeletonPtr roof;
	Eigen::Isometry3d roofPose;

	// Poses for table
	tablePose = Eigen::Isometry3d::Identity();
	tablePose.translation() = Eigen::Vector3d(1.0, 0.0, 0);
	Eigen::Matrix3d rot;
	rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
	     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
	     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
	tablePose.linear() = rot;

	// Poses for Shelf
	shelfPose = Eigen::Isometry3d::Identity();
	shelfPose.translation() = Eigen::Vector3d(-0.0, -1.25, 0);
	rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
	    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
	    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
	shelfPose.linear() = rot;

	// Poses for Shelf
	pitcherPose = Eigen::Isometry3d::Identity();
	pitcherPose.translation() = tablePose.translation() +
	 Eigen::Vector3d(0.0, 0.0, 0.73);

	// Poses for Oven
	roofPose = Eigen::Isometry3d::Identity();
	roofPose.translation() = tablePose.translation() +
	 Eigen::Vector3d(0.0, 0.0, 2.3);
	rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
	    * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
	    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
	roofPose.linear() = rot;

	// Load objects
	table = makeBodyFromURDF(resourceRetriever, tableURDFUri, tablePose);
	shelf = makeBodyFromURDF(resourceRetriever, shelfURDFUri, shelfPose);
	pitcher = makeBodyFromURDF(resourceRetriever, pitcherURDFUri, pitcherPose);
	roof = makeBodyFromURDF(resourceRetriever, roofURDFUri, roofPose);

	loadedObstacles.push_back(table);
	// loadedObstacles.push_back(shelf);
	// NOTE: Ignore pitcher since it can block cans.
	// loadedObstacles.push_back(pitcher);
	loadedObstacles.push_back(roof);

	tablePoseOut = tablePose;
	return loadedObstacles;
}

Eigen::Isometry3d stateVecToMatrix(Eigen::VectorXd state)
{
  // NOTE 1: We apply the INVERSE transform of the robot to the world.
  // NOTE 2: We assume R^3 for now.
  Eigen::Vector3d transVec = Eigen::Vector3d(-state[0], -state[1], -state[2]);
  Eigen::Isometry3d stateMat = Eigen::Isometry3d::Identity();
  stateMat.translation() = transVec;

  return stateMat;
}


void addSoda(const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever, RobotPtr robot, 
	Eigen::Isometry3d& tablePose, Eigen::Isometry3d& sodaPose, std::vector<SkeletonPtr>& obstaclesOut) 
{
	const std::string sodaName{"can"};
	const std::string sodaURDFUri("package://pr_assets/data/objects/can.urdf");

	// Poses for soda
	sodaPose = Eigen::Isometry3d::Identity();
	sodaPose.translation() = tablePose.translation() + Eigen::Vector3d(POSITIVE_X, POSITIVE_Y, POSITIVE_Z);

	
	dart::dynamics::SkeletonPtr soda = makeBodyFromURDF(resourceRetriever, sodaURDFUri, sodaPose);
	soda->setName(sodaName);
	obstaclesOut.push_back(std::move(soda));
}

void loadHerbScenario(RobotPtr robot, Eigen::Isometry3d& sodaPoseOut, std::vector<SkeletonPtr>& obstaclesOut)
{
	std::vector<SkeletonPtr> loaded;
	Eigen::Isometry3d loadedTablePose;
	loaded = loadHERBExample(loadedTablePose);

	for (auto loadedSkel : loaded)
		obstaclesOut.push_back(loadedSkel);

	// Now, place soda on the table that HERB must try to grasp.
	const auto resourceRetriever
	   = std::make_shared<aikido::io::CatkinResourceRetriever>();
	addSoda(resourceRetriever, robot, loadedTablePose, sodaPoseOut, obstaclesOut);
}

void enableColChecks(RobotPtr robot, CollisionFreePtr constraint, CollisionDetectorPtr detector, std::vector<SkeletonPtr>& obstacles) 
{
	std::shared_ptr<CollisionGroup> rightArmGroup =
	detector->createCollisionGroup(robot->getRightArm()->getMetaSkeleton().get(),robot->getRightHand()->getMetaSkeleton().get());
	for (SkeletonPtr curObstacle : obstacles)
	{
		std::shared_ptr<CollisionGroup> curObstacleGroup = detector->createCollisionGroup(curObstacle.get());
		constraint->addPairwiseCheck(rightArmGroup, curObstacleGroup);
	}
}

std::unique_ptr<aikido::common::RNG> initAikidoRng(int randomSeed) 
{
	aikido::common::RNGWrapper<std::mt19937> rng = aikido::common::RNGWrapper<std::mt19937>(randomSeed);
	return std::move(cloneRNGFrom(rng)[0]);
}


class sampleTSR
{
public:
	aikido::planner::WorldPtr mWorld;
    std::vector<SkeletonPtr> mObstacles;
    Eigen::Isometry3d mSodaPose;
    std::shared_ptr<aikido::constraint::dart::TSR> mTSR;
    // NOTE: The IK samples taken for a *single* call to generator->sample().
    int mMaxTSRSamples = 5;
    // Robot.
    
    RobotPtr mRobot;
    int mDOF = 7;
    BodyNodePtr mRightHand;
    MetaSkeletonPtr mRightArm;
    MetaSkeletonStateSpacePtr mRightArmStateSpace;
    std::shared_ptr<GeodesicInterpolator> mInterpolator;
    Eigen::VectorXd mStartConfig;
    InverseKinematicsPtr mIK;

    TestablePtr mSelfTestable;
    CollisionFreePtr mEnvTestable;
    TestablePtr mFullTestable;

    // For sampling IK from the TSR.
	int mRandomSeed;

    sampleTSR()
    {
    	mRandomSeed=1;
		std::cout << "[INFO]: Loading Herb in Simulation." << std::endl;
		// Create an AIKIDO world,.

		mWorld = aikido::planner::WorldPtr(new aikido::planner::World("world"));
		mRobot = std::move(std::make_shared<herb::Herb>(mWorld, true));

		std::cout << "[INFO]: Using right arm as default." << std::endl;

		// Also set the left arm in a configuration that is "out of the way"
		// to avoid collisions.
		Eigen::VectorXd lRelaxedHome(7);
		lRelaxedHome << 1.58326, 0.223173, 0.307813, 1.33726, 0.600263, -0.476479, 2.40935;
		MetaSkeletonPtr leftArm = std::const_pointer_cast<MetaSkeleton>(mRobot->getLeftArm()->getMetaSkeleton());
		leftArm->setPositions(lRelaxedHome);

		// Close the right hand slightly to minimize chance of collision.
		mRobot->getRightHand()->executePreshape("partial_open");

		mRightHand = mRobot->getRightHand()->getEndEffectorBodyNode();
		mRightArm = std::const_pointer_cast<MetaSkeleton>(mRobot->getRightArm()->getMetaSkeleton());
		mRightArmStateSpace = std::make_shared<MetaSkeletonStateSpace>(mRightArm.get());
		mInterpolator = std::make_shared<GeodesicInterpolator>(mRightArmStateSpace);

		mStartConfig = Eigen::VectorXd(7);
		mStartConfig << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;
		mRightArm->setPositions(mStartConfig);

		// Create an IK solver.
		mIK = InverseKinematics::create(mRightHand);
		mIK->setDofs(mRightArm->getDofs());

		loadHerbScenario(mRobot, mSodaPose, mObstacles);

		// Create the TSR with the soda can's pose.
		mTSR = std::make_shared<aikido::constraint::dart::TSR>(
		pr_tsr::getDefaultCanTSR());
		mTSR->mTw_e.matrix()
		*= mRobot->getRightHand()->getEndEffectorTransform("cylinder")->matrix();
		mTSR->mT0_w = mSodaPose;

		mSelfTestable = mRobot->getFullCollisionConstraint(
		mRightArmStateSpace, mRightArm, NULL);

		auto envDetector = dart::collision::FCLCollisionDetector::create();
		mEnvTestable = std::make_shared<CollisionFree>(
		mRightArmStateSpace, mRightArm, envDetector);
		// Account for obstacles!
		enableColChecks(mRobot, mEnvTestable, envDetector, mObstacles);

		mFullTestable = mRobot->getFullCollisionConstraint(
		mRightArmStateSpace, mRightArm, mEnvTestable);

	}

	Eigen::VectorXd sampleTSRConfigs(Eigen::VectorXd state)
	{
		 // Create an IK solver.
		mIK = InverseKinematics::create(mRightHand);
		mIK->setDofs(mRightArm->getDofs());

		mTSR = std::make_shared<aikido::constraint::dart::TSR>(
		  pr_tsr::getDefaultCanTSR());
		mTSR->mTw_e.matrix()
		  *= mRobot->getRightHand()->getEndEffectorTransform("cylinder")->matrix();
		mTSR->mT0_w = mSodaPose;
		// Shift the TSR to account for robot state.
		Eigen::Isometry3d stateTrans = stateVecToMatrix(state);
		Eigen::Isometry3d newTSRPose =  stateTrans * mSodaPose;
		mTSR->mT0_w = newTSRPose;

		// Sample the TSR for an IK.
		InverseKinematicsSampleable ikSampleable(
			mRightArmStateSpace,
			mRightArm,
			std::const_pointer_cast<aikido::constraint::dart::TSR>(mTSR),
			createSampleableBounds(mRightArmStateSpace, initAikidoRng(mRandomSeed)),
			mIK,
			mMaxTSRSamples);
		auto generator = ikSampleable.createSampleGenerator();

		auto saver = MetaSkeletonStateSaver(mRightArm);
		DART_UNUSED(saver);

		auto robotSkeleton = mRightArm->getBodyNode(0)->getSkeleton();

		std::vector<MetaSkeletonStateSpace::ScopedState> configurations;

		// Use ranker to pick the goal config closest to the start config.
		auto startState = mRightArmStateSpace->createState();
		mRightArmStateSpace->getState(mRightArm.get(), startState);
		auto nominalState = mRightArmStateSpace->createState();
		mRightArmStateSpace->copyState(startState, nominalState);
		auto configurationRanker = std::make_shared<const NominalConfigurationRanker>(
			mRightArmStateSpace,
			mRightArm,
			nominalState,
			std::vector<double>());

		auto goalState = mRightArmStateSpace->createState();

		// Sampling loop.
		static const std::size_t maxSamples{100};
		std::size_t samples = 0;
		while (samples < maxSamples && generator->canSample())
		{
			// Sample from TSR
			std::lock_guard<std::mutex> lock(robotSkeleton->getMutex());
			bool sampled = generator->sample(goalState);

			// Increment even if it's not a valid sample since this loop
			// has to terminate even if none are valid.
			++samples;

			if (!sampled)
			  continue;

			configurations.emplace_back(goalState.clone());
		}

		// NOTE: Zero config represents failure here!
		if (configurations.empty())
		{
		std::cout << "" << std::endl;
		std::cout << "Couldn't sample TSR IK!" << std::endl;

		return Eigen::VectorXd::Zero(mDOF);
		}

		configurationRanker->rankConfigurations(configurations);

		for (std::size_t i = 0; i < configurations.size(); ++i)
		{
			// Now that configs are sorted, return the first free goal state.
			if (mFullTestable->isSatisfied(configurations[i]))
			{
			  Eigen::VectorXd goalConfigOut(mDOF);
			  mRightArmStateSpace->convertStateToPositions(configurations[i], goalConfigOut);
			  return goalConfigOut;
			}
		}

		// Failure to find a close, collision-free goal config.
		std::cout << "" << std::endl;
		std::cout << "All TSR IK in collision!" << std::endl;
		return Eigen::VectorXd::Zero(mDOF);
	}
	void getGoalConfigs(Eigen::VectorXd state) 
	{
		// TODO: SHIFT around the robot/env.
		// Reset the start config.
		mRightArm->setPositions(mStartConfig);

		double xTrans = state[0];
		double yTrans = state[1];

		std::cout<<"Press [ENTER] to start solving:";
		std::cin.get();

		// Sample a goal config from the IK of the TSR.
		Eigen::VectorXd sampledGoal = sampleTSRConfigs(state);

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
		aikido::rviz::WorldInteractiveMarkerViewer viewer(mWorld, topicName, baseFrameName);

		viewer.setAutoUpdate(true);
		for (std::size_t i = 0; i < mObstacles.size(); ++i)
		{
			mWorld->addSkeleton(mObstacles.at(i));
		}
		std::cout<<"Press [ENTER] to send arm to can: ";
		std::cin.get();

		if (sampledGoal.isZero())
		{
			std::cout << "" << std::endl;
			std::cout << "Goal state sampling failed!" << std::endl;
		}
		else
		{
			mRightArm->setPositions(sampledGoal);

			std::cout<<"GOAL STATE: "<<sampledGoal<<std::endl;

			std::cout<<"PRESS [ENTER] TO EXIT";
			std::cin.get();
		}
	}
};

int main(int argc, char* argv[])
{
	sampleTSR solver;
	Eigen::VectorXd defaultState(3);
	defaultState << 0 , 0, 0;
    solver.getGoalConfigs(defaultState);

}