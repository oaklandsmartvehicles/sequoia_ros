#include <sequoia_gazebo/CameraControlPlugin.h>

namespace gazebo
{

CameraControlPlugin::CameraControlPlugin()
{
}

void CameraControlPlugin::Load(int argc, char** argv)
{
  ros::init(argc, argv, "view_control");
  n_ = new ros::NodeHandle("view_control");

  srv_.reset(new dynamic_reconfigure::Server<sequoia_gazebo::CameraControlPluginConfig>(*n_));
  srv_->setCallback(boost::bind(&CameraControlPlugin::reconfig, this, _1, _2));

  async_ = new ros::AsyncSpinner(3);
  async_->start();
}

void CameraControlPlugin::reconfig(sequoia_gazebo::CameraControlPluginConfig& config, uint32_t level)
{
  cfg_ = config;
}

void CameraControlPlugin::Init()
{
  connections_.push_back(event::Events::ConnectPreRender(boost::bind(&CameraControlPlugin::Update, this)));
}

void CameraControlPlugin::Reset()
{
}

void CameraControlPlugin::Update()
{
  rendering::VisualPtr camera_target = rendering::get_scene()->GetVisual(cfg_.model_name);

  if (!user_cam_){
    user_cam_ = gui::get_active_camera();
  }else if (cfg_.enable){
    if (camera_target){
      lookAtRobot(camera_target);
#if GAZEBO_MAJOR_VERSION > 2
      // SetWorldPosition with math::Vector3 argument is deprecated in Gazebo v7
      ignition::math::Vector3d ignition_pos;
      ignition_pos.Set(camera_pos_.x, camera_pos_.y, camera_pos_.z);
      user_cam_->SetWorldPosition(ignition_pos);
#else
      user_cam_->SetWorldPosition(camera_pos_);
#endif
      user_cam_->SetFocalPoint(camera_target->GetPose().pos);
    }
  }
}

void CameraControlPlugin::lookAtRobot(const rendering::VisualPtr& camera_target)
{
  // Get current orientation of the camera
#if GAZEBO_MAJOR_VERSION > 2
  math::Quaternion cam_rotation = user_cam_->WorldRotation();
#else
  math::Quaternion cam_rotation = user_cam_->GetWorldRotation();
#endif

  // Compute equivalent rotation matrix
  tf::Matrix3x3 rot_mat;
  rot_mat.setRotation(tf::Quaternion(cam_rotation.x, cam_rotation.y, cam_rotation.z, cam_rotation.w));

  // Move camera such that LOS vector points toward robot
  camera_pos_.x = camera_target->GetPose().pos.x - cfg_.view_dist * rot_mat.getColumn(0).x();
  camera_pos_.y = camera_target->GetPose().pos.y - cfg_.view_dist * rot_mat.getColumn(0).y();
  camera_pos_.z = camera_target->GetPose().pos.z - cfg_.view_dist * rot_mat.getColumn(0).z();
}

CameraControlPlugin::~CameraControlPlugin()
{
  this->connections_.clear();
  this->user_cam_.reset();
}

}
