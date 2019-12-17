#include <ros/ros.h>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math4/ignition/math/Vector3.hh>

double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

namespace gazebo {
class RandomModelMove : public ModelPlugin {
   public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
        // Store the pointer to the model
        this->model = _parent;
        world_ = this->model->GetWorld();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&RandomModelMove::OnUpdate, this));

        previous_time = world_->RealTime();
    }

    // Called by the world update start event
   public:
    void OnUpdate() {
        // Apply a small linear velocity to the modeltNsm=

        if ((world_->RealTime() - previous_time) > 2.0) {
            this->model->SetGravityMode(false);
            ignition::math::Pose3d current_pose = this->model->WorldPose();
            ignition::math::Vector3d current_pos = current_pose.Pos();
            ignition::math::Quaterniond current_rot(0, 0, 0, 1);
            current_pos = ignition::math::Vector3d(
                fRand(1.075, 1.525), fRand(-2.85, -2.15), fRand(1.1, 1.35));
            ignition::math::Pose3d random_pose(current_pos, current_rot);
            // this->model->SetWorldPose(random_pose);
            // this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0,
            // 0.0));
            previous_time = world_->RealTime();
        }

        // sleep(1.0);

        // current_pos[2] = 1.1;
        // current_pose.Set(current_pos, current_rot);
    }

    // Pointer to the model
   private:
    physics::ModelPtr model;
    gazebo::physics::WorldPtr world_;
    common::Time previous_time;

    // Pointer to the update event connection
   private:
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RandomModelMove)
}  // namespace gazebo
