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

#include "moteus.h"

int main(int argc, char** argv) {
  using namespace mjbots;

  moteus::Controller::DefaultArgProcess(argc, argv);

  moteus::Controller::Options options;

  auto& pf = options.position_format;
  pf.position = moteus::kIgnore;
  pf.velocity = moteus::kIgnore;
  pf.feedforward_torque = moteus::kFloat;
  pf.kp_scale = moteus::kInt8;
  pf.kd_scale = moteus::kInt8;

  moteus::Controller c(options);

  c.SetStop();

  moteus::PositionMode::Command cmd;
  cmd.kp_scale = 0.0;
  cmd.kd_scale = 0.0;
  cmd.feedforward_torque = 0.0;

  constexpr double kMassKg = 0.26;  //  (this is the actual value 0.3;)
  constexpr double kLinkLength = 0.15;

  int missed_replies = 0;
  int status_count = 0;
  constexpr int kStatusPeriod = 100;

  while (true) {
    ::usleep(10);

    const auto maybe_result = c.SetPosition(cmd);
    if (!maybe_result) {
      missed_replies++;
      if (missed_replies > 3) {
        printf("\n\nmotor timeout!\n");
        break;
      }

      continue;
    } else {
      missed_replies = 0;
    }

    const auto& result = *maybe_result;
    const auto& v = result.values;

    const double command_torque =
        std::sin(v.position * 2 * M_PI) * kMassKg * kLinkLength * 9.8;
    cmd.feedforward_torque = command_torque;

    status_count++;
    if (status_count > kStatusPeriod) {
      status_count = 0;

      printf("Mode: %2d  position: %6.3f  cmd_torque: %6.3f  temp: %4.1f  \r",
             static_cast<int>(v.mode),
             v.position,
             command_torque,
             v.temperature);
      fflush(stdout);
    }
  }

  printf("Entering fault mode!\n");

  while (true) {
    ::usleep(50000);
    c.SetBrake();
  }

  return 0;
}
