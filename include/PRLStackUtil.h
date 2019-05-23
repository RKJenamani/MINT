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

static const std::string dartTopicName("dart_markers");
static const std::string baseFrameName("map");

class PRLStackUtil
{
public:

	MetaSkeletonStateSpacePtr armSpace;

	aikido::planner::WorldPtr env;

	MetaSkeletonPtr dynamicArm;
	MetaSkeletonPtr staticArm;

	TestablePtr mSelfTestable;
	CollisionFreePtr mEnvTestable;
	TestablePtr mFullTestable;

	std::shared_ptr<GeodesicInterpolator> mInterpolator;

	std::vector<SkeletonPtr> obstacles;

	PRLStackUtil(const std::string& arm_name);

	void waitForUser(const std::string& msg);

	bool getCollisionStatus(Eigen::VectorXd &start, Eigen::VectorXd &end);

	void executePath(std::vector<Eigen::VectorXd> & configs);


	const SkeletonPtr makeBodyFromURDF(const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
		const std::string& uri,const Eigen::Isometry3d& transform);

	bool perceiveObject(ros::NodeHandle nh, const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
		herb::Herb& robot, dart::dynamics::SkeletonPtr& object, Eigen::Isometry3d& objectPose, std::string name);

	InterpolatedPtr genTrajFromConfigs(std::vector<Eigen::VectorXd>& trajConfigs,
		MetaSkeletonStateSpacePtr armStateSpace, std::shared_ptr<GeodesicInterpolator> interpolator);

	void executeSim(RobotPtr robot, MetaSkeletonPtr arm, aikido::planner::WorldPtr world, 
		const InterpolatedPtr& untimedTraj,std::vector<SkeletonPtr> obstacles);

};