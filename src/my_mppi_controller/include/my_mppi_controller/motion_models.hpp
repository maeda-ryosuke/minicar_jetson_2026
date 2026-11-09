// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MY_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define MY_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <algorithm>  // [ADD] std::clamp
#include <cmath>  // [ADD] std::exp, std::tan
#include <cstdint>
#include <string>

#include "my_mppi_controller/models/control_sequence.hpp"
#include "my_mppi_controller/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

#include "my_mppi_controller/tools/parameters_handler.hpp"

namespace my_mppi
{

/**
 * @class my_mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for my_mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for my_mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    using namespace xt::placeholders;  // NOLINT
    xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
      xt::view(state.cvx, xt::all(), xt::range(0, -1));

    xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
      xt::view(state.cwz, xt::all(), xt::range(0, -1));

    if (isHolonomic()) {
      xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
        xt::view(state.cvy, xt::all(), xt::range(0, -1));
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}
};

/**
 * @class my_mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for my_mppi::AckermannMotionModel
    */
  explicit AckermannMotionModel(ParametersHandler * param_handler, const std::string & name)
  {
    auto getParam = param_handler->getParamGetter(name + ".AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & wz = control_sequence.wz;

    auto view = xt::masked_view(wz, (xt::fabs(vx) / xt::fabs(wz)) < min_turning_r_);
    view = xt::sign(wz) * xt::fabs(vx) / min_turning_r_;
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};
};

// [ADD BEGIN] 運動学的自転車モデル
/**
 * @class my_mppi::BicycleMotionModel
 * @brief cwz をステア角 δ [rad] として扱う。δ はサーボの1次遅れで指令に追従し、
 * ヨーレートは wz = v·tan(δ)/L で求める
 */
class BicycleMotionModel : public MotionModel
{
public:
  BicycleMotionModel(ParametersHandler * param_handler, const std::string & name, float model_dt)
  : model_dt_(model_dt)
  {
    auto getParam = param_handler->getParamGetter(name);
    getParam(wheelbase_, "wheelbase", 0.257);
    getParam(steer_time_constant_, "steer_time_constant", 0.08);
    getParam(steer_rate_max_, "steer_rate_max", 4.0);
  }

  bool isHolonomic() override
  {
    return false;
  }

  void predict(models::State & state) override
  {
    using namespace xt::placeholders;  // NOLINT
    xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
      xt::view(state.cvx, xt::all(), xt::range(0, -1));

    // δ は前の周期から持ち越した推定値から始め、1ステップずつ指令へ追従させる
    const size_t batch_size = state.cwz.shape(0);
    const size_t time_steps = state.cwz.shape(1);
    for (size_t i = 0; i < batch_size; ++i) {
      float delta = delta_estimate_;
      for (size_t t = 1; t < time_steps; ++t) {
        delta = steerStep(delta, state.cwz(i, t - 1), model_dt_);
        state.wz(i, t) = state.vx(i, t) * std::tan(delta) / wheelbase_;
      }
    }
  }

  /**
   * @brief 出力した δ 指令で、実際の δ の推定値を1制御周期ぶん進める
   */
  void updateSteerEstimate(float delta_cmd, float dt)
  {
    delta_estimate_ = steerStep(delta_estimate_, delta_cmd, dt);
  }

  float getWheelbase() const {return wheelbase_;}

protected:
  // 1次遅れ(1ステップの間 cmd が一定としたときの厳密解)。その後、変化量をレート上限で制限する
  float steerStep(float delta, float cmd, float dt) const
  {
    const float alpha = 1.0f - std::exp(-dt / steer_time_constant_);
    const float max_step = steer_rate_max_ * dt;
    return delta + std::clamp((cmd - delta) * alpha, -max_step, max_step);
  }

  float model_dt_;
  float wheelbase_{0.257};
  float steer_time_constant_{0.08};
  float steer_rate_max_{4.0};
  float delta_estimate_{0.0};
};
// [ADD END]

/**
 * @class my_mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for my_mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class my_mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for my_mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace my_mppi

#endif  // MY_MPPI_CONTROLLER__MOTION_MODELS_HPP_
