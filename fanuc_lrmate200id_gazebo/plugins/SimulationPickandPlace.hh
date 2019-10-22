#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math3/ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <Eigen/Core>

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fanuc_arm_for_chicony/pickplace/PickandPlacer.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <fanuc_arm_for_chicony/utils.h>
namespace gazebo
{

class SimulationPickandPlace : public WorldPlugin
{

    // Pointer to the model
private:
    physics::WorldPtr world;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    PickandPlacer *pickplacer_;
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_;
    bool flag = true;

    ros::Publisher objects_tobe_picked_pub_;
    ros::NodeHandle *nh_;

public:
    SimulationPickandPlace();
    ~SimulationPickandPlace();
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
    void OnUpdate();
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SimulationPickandPlace);
} // namespace gazebo