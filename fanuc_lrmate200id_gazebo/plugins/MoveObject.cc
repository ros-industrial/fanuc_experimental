#include <ros/ros.h>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math4/ignition/math/Vector3.hh>

namespace gazebo {
class ModelPush : public ModelPlugin {
   public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
   public:
    void OnUpdate() {
        // Apply a small linear velocity to the modeltNsm=
        this->model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0));
        this->model->SetGravityMode(false);
        ignition::math::Pose3d current_pose = this->model->WorldPose();
        ignition::math::Vector3d current_pos = current_pose.Pos();
        ignition::math::Quaterniond current_rot = current_pose.Rot();
        // current_pos[2] = 1.1;
        // current_pose.Set(current_pos, current_rot);
    }

    // Pointer to the model
   private:
    physics::ModelPtr model;

    // Pointer to the update event connection
   private:
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}  // namespace gazebo
