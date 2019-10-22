#include "SimulationPickandPlace.hh"

using namespace gazebo;
SimulationPickandPlace::SimulationPickandPlace()
{

    nh_ = new ros::NodeHandle();
    objects_tobe_picked_pub_ = nh_->advertise<geometry_msgs::PoseArray>("objects_tobe_picked", 1);
}

SimulationPickandPlace::~SimulationPickandPlace()
{
}

void SimulationPickandPlace::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    this->world = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimulationPickandPlace::OnUpdate, this));
}

void SimulationPickandPlace::OnUpdate()
{
    geometry_msgs::PoseArray objects_tobe_picked_array;
    std::string robot_model_name = "larmate200id7l";
    std::string gripper_link_name = "Gripper_Base";
    std::string base_link_name = "base_link";

    physics::Model_V objects_tobe_picked;
    physics::LinkPtr gripper_link;
    physics::LinkPtr base_link;
    ignition::math::Vector3d gripper_link_pos;
    ignition::math::Vector3d base_link_pos;

    physics::Model_V models_vector = world->Models();
    for (int i = 0; i < models_vector.size(); i++)
    {
        std::string current_model_name = models_vector[i]->GetName();
        physics::ModelPtr current_model = models_vector[i];

        if (current_model_name == robot_model_name)
        {
            gripper_link = current_model->GetLink(gripper_link_name);
            gripper_link_pos = gripper_link->WorldPose().Pos();
            base_link = current_model->GetLink(base_link_name);
            base_link_pos = base_link->WorldPose().Pos();
        }
        if (current_model_name.substr(0, 6) == "pulley")
        {
            objects_tobe_picked.push_back(current_model);
        }
    }

    for (int o = 0; o < objects_tobe_picked.size(); o++)
    {
        physics::ModelPtr current_object_tobe_picked = objects_tobe_picked[o];
        ignition::math::Vector3d current_object_pos = current_object_tobe_picked->WorldPose().Pos();

        ignition::math::Vector3d obj_in_base_link((current_object_pos[0] - base_link_pos[0]),
                                                  (current_object_pos[1] - base_link_pos[1]),
                                                  (current_object_pos[2] - base_link_pos[2]));

        if (std::abs(gripper_link_pos[1] - current_object_pos[1]) < 0.1)
        {

            geometry_msgs::Pose obj_pose;
            obj_pose.position.x = obj_in_base_link[0];
            obj_pose.position.y = obj_in_base_link[1];
            obj_pose.position.z = obj_in_base_link[2];
            obj_pose.orientation.x = 0.0;
            obj_pose.orientation.y = -1.0;
            obj_pose.orientation.z = 0.0;
            obj_pose.orientation.w = 0.0;

            //ROS_INFO("Found object to be picked at x %.2f y %.2f z %.2f", obj_pose.position.x, obj_pose.position.y, obj_pose.position.z);
            objects_tobe_picked_array.poses.push_back(obj_pose);

            if ((gripper_link_pos[2] - current_object_pos[2] < 0.25) && (gripper_link_pos[0] > 0.45))
            {
                ROS_INFO("SHOULD BE ATTACHED NOW");
                ignition::math::Pose3d offset(gripper_link_pos[0], gripper_link_pos[1], (gripper_link_pos[2] - 0.2), 0, -1, 0, 0);
                current_object_tobe_picked->SetWorldPose(offset, true, true);
                current_object_tobe_picked->SetGravityMode(true);
            }
        }
    }
    objects_tobe_picked_pub_.publish(objects_tobe_picked_array);
}
