// Switches the scene's "grasp_weld" (wrist_3_link -> board) on while the gripper is commanded
// closed on the board, and off when it opens. MuJoCo's soft friction lets a board held only by
// contacts creep in the grasp; the weld stands in for the static friction of the real pads.
// The rule mirrors maze_sim_description.scene_builder.grasp_weld_engaged, and the tolerances and
// the gripper actuator come from the model (<numeric>/<text> written by the scene builder).

#include <cmath>
#include <string>

#include <mujoco/mujoco.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace maze_sim_nodes
{

class GraspWeldPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * /*data*/) override
  {
    auto log = node->get_logger();
    model_ = model;
    weld_ = mj_name2id(model, mjOBJ_EQUALITY, "grasp_weld");
    const int tol = mj_name2id(model, mjOBJ_NUMERIC, "grasp_weld");
    const int text = mj_name2id(model, mjOBJ_TEXT, "grasp_weld_actuator");
    if (weld_ < 0 || model->eq_type[weld_] != mjEQ_WELD || tol < 0 || model->numeric_size[tol] < 2 || text < 0) {
      RCLCPP_ERROR(log, "GraspWeldPlugin: the model has no grasp_weld weld, numeric or actuator text");
      return false;
    }
    actuator_ = mj_name2id(model, mjOBJ_ACTUATOR, model->text_data + model->text_adr[text]);
    if (actuator_ < 0) {
      RCLCPP_ERROR(log, "GraspWeldPlugin: actuator '%s' not found", model->text_data + model->text_adr[text]);
      return false;
    }
    pos_tol_ = model->numeric_data[model->numeric_adr[tol]];
    cos_tol_ = std::cos(model->numeric_data[model->numeric_adr[tol] + 1] * M_PI / 180.0);
    const mjtNum * range = model->actuator_ctrlrange + 2 * actuator_;
    closed_above_ = (range[0] + range[1]) / 2;
    RCLCPP_INFO(log, "GraspWeldPlugin: grasp weld follows %s", model->text_data + model->text_adr[text]);
    return true;
  }

  // physics thread, before every step: the weld is on for exactly the steps the rule says
  void pre_step(mjData * data) override
  {
    data->eq_active[weld_] = engaged(data, data->eq_active[weld_]);
  }

  // xpos may still be the pre-reset world here: start released, the next step decides
  void on_reset(mjData * data) override
  {
    data->eq_active[weld_] = 0;
  }

  void cleanup() override
  {
  }

private:
  mjtByte engaged(const mjData * d, mjtByte was_engaged) const
  {
    if (d->ctrl[actuator_] <= closed_above_) {
      return 0;
    }
    if (was_engaged) {
      return 1;
    }
    // the board's pose in the wrist frame against the weld's target (eq_data: anchor, relpose)
    const int b1 = model_->eq_obj1id[weld_], b2 = model_->eq_obj2id[weld_];
    const mjtNum * rel = model_->eq_data + mjNEQDATA * weld_;
    mjtNum dp[3], p_rel[3], R_rel[9], R_cur[9];
    mju_sub3(dp, d->xpos + 3 * b2, d->xpos + 3 * b1);
    mju_mulMatTVec3(p_rel, d->xmat + 9 * b1, dp);
    mju_subFrom3(p_rel, rel + 3);
    mju_quat2Mat(R_rel, rel + 6);
    mju_mulMatTMat(R_cur, d->xmat + 9 * b1, d->xmat + 9 * b2, 3, 3, 3);  // board in wrist
    mjtNum trace = 0;  // trace(R_rel^T R_cur)
    for (int i = 0; i < 9; ++i) {
      trace += R_rel[i] * R_cur[i];
    }
    return mju_norm3(p_rel) < pos_tol_ && (trace - 1) / 2 > cos_tol_;
  }

  const mjModel * model_ = nullptr;
  int weld_ = -1;
  int actuator_ = -1;
  mjtNum pos_tol_ = 0;
  mjtNum cos_tol_ = 1;
  mjtNum closed_above_ = 0;
};

}  // namespace maze_sim_nodes

PLUGINLIB_EXPORT_CLASS(maze_sim_nodes::GraspWeldPlugin, mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
