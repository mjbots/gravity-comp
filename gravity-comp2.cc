// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <optional>

#include "moteus.h"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/sample-models.hpp"

namespace {
template <typename Scalar, int Options,
  template <typename, int> class JointCollectionTpl>
void BuildModel(pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl>* model) {
  using namespace pinocchio;

  using M = Model;
  using JC = JointCollectionTpl<Scalar, Options>;
  using CV = typename JC::JointModelRX::ConfigVector_t;
  using TV = typename JC::JointModelRX::TangentVector_t;

  M::JointIndex idx = 0;

  constexpr double kFudge = 0.95;

  SE3 Tlink (SE3::Matrix3::Identity(), SE3::Vector3(0, 0, 0.15));
  Inertia Ilink1(kFudge * 0.29, Tlink.translation(),
                 Inertia::Matrix3::Identity() * 0.001);
  Inertia Ilink2(kFudge * 0.28, Tlink.translation(),
                 Inertia::Matrix3::Identity() * 0.001);

  CV qmin = CV::Constant(-4);
  CV qmax = CV::Constant(4);
  TV vmax = CV::Constant(10);
  TV taumax = CV::Constant(10);

  idx = model->addJoint(idx, typename JC::JointModelRY(), Tlink,
                        "link1_joint", taumax, vmax, qmin, qmax);
  model->appendBodyToJoint(idx, Ilink1);
  model->addJointFrame(idx);
  model->addBodyFrame("link1_body", idx);

  idx = model->addJoint(idx, typename JC::JointModelRY(), Tlink,
                        "link2_joint", taumax, vmax, qmin, qmax);
  model->appendBodyToJoint(idx, Ilink2);
  model->addJointFrame(idx);
  model->addBodyFrame("link2_body", idx);
}

std::optional<mjbots::moteus::Query::Result> FindServo(
    const std::vector<mjbots::moteus::CanFdFrame>& frames,
    int id) {
  for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
    if (it->source ==id) {
      return mjbots::moteus::Query::Parse(it->data, it->size);
    }
  }
  return {};
}

double WrapAround0(double v) {
  const auto v1 = std::fmod(v, 1.0);
  const auto v2 = (v1 < 0.0) ? (v1 + 1.0) : v1;
  return v2 > 0.5 ? (v2 - 1.0) : v2;
}
}

int main(int argc, char** argv) {
  using namespace mjbots;
  moteus::Controller::DefaultArgProcess(argc, argv);
  auto transport = moteus::Controller::MakeSingletonTransport({});

  pinocchio::Model model;
  BuildModel(&model);
  pinocchio::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  moteus::Controller::Options options_common;

  auto& pf = options_common.position_format;
  pf.position = moteus::kIgnore;
  pf.velocity = moteus::kIgnore;
  pf.feedforward_torque = moteus::kFloat;
  pf.kp_scale = moteus::kInt8;
  pf.kd_scale = moteus::kInt8;

  std::vector<std::shared_ptr<moteus::Controller>> controllers = {
    std::make_shared<moteus::Controller>([&]() {
      auto options = options_common;
      options.id = 1;
      return options;
    }()),
    std::make_shared<moteus::Controller>([&]() {
      auto options = options_common;
      options.id = 2;
      return options;
    }()),
  };

  for (auto& c : controllers) { c->SetStop(); }

  moteus::PositionMode::Command cmd;
  cmd.kp_scale = 0.0;
  cmd.kd_scale = 0.0;
  cmd.feedforward_torque = 0.0;


  double torque_command[2] = {};
  std::vector<moteus::CanFdFrame> send_frames;
  std::vector<moteus::CanFdFrame> receive_frames;

  int missed_replies = 0;
  int status_count = 0;
  constexpr int kStatusPeriod = 100;

  while (true) {
    ::usleep(10);

    send_frames.clear();
    receive_frames.clear();

    for (size_t i = 0; i < controllers.size(); i++) {
      cmd.feedforward_torque = torque_command[i];
      send_frames.push_back(controllers[i]->MakePosition(cmd));
    }

    transport->BlockingCycle(
        &send_frames[0], send_frames.size(),
        &receive_frames);

    auto maybe_servo1 = FindServo(receive_frames, 1);
    auto maybe_servo2 = FindServo(receive_frames, 2);

    if (!maybe_servo1 || !maybe_servo2) {
      missed_replies++;
      if (missed_replies > 3) {
        printf("\n\nServo not responding 1=%d 2=%d\n",
               maybe_servo1 ? 1 : 0,
               maybe_servo2 ? 1 : 0);
        break;
      }
      continue;
    } else {
      missed_replies = 0;
    }

    const auto& v1 = *maybe_servo1;
    const auto& v2 = *maybe_servo2;

    q(0) = WrapAround0(v1.position + 0.5) * 2 * M_PI;
    q(1) = WrapAround0(v2.position) * 2 * M_PI;

    const Eigen::VectorXd& tau = pinocchio::rnea(model, data, q, v, a);

    torque_command[0] = tau(0);
    torque_command[1] = tau(1);

    status_count++;
    if (status_count > kStatusPeriod) {

      printf("Mode: %2d/%2d  position: %6.3f/%6.3f  torque: %6.3f/%6.3f  temp: %4.1f/%4.1f  \r",
             static_cast<int>(v1.mode), static_cast<int>(v2.mode),
             v1.position, v2.position,
             torque_command[0], torque_command[1],
             v1.temperature, v2.temperature);
      fflush(stdout);

      status_count = 0;
    }

  }

  printf("Entering fault mode!\n");

  while (true) {
    ::usleep(50000);

    for (auto& c : controllers) { c->SetBrake(); }
  }

  return 0;
}
