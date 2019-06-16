//
// Created by Ao Shen on 19-6-15
//

#ifndef PUBLICCOURSE_SAO_CONSTANTS_H
#define PUBLICCOURSE_SAO_CONSTANTS_H

namespace sao_agent {
constexpr int history_length = 10;  // save recent 0.1s' status
constexpr double kmh_to_ms(double kmh) { return kmh / 3.6; }
constexpr double vec_limit = kmh_to_ms(20);
constexpr double vec_hard_limit = kmh_to_ms(50);
constexpr double throttle_limit = 0.2999;
constexpr double checkpoint_threshold = 5.0;
constexpr double iteration_time = 0.01;
constexpr double goal_near_threshold = 11.5;
constexpr int greenlight_pass_time = 1000;

namespace V {
constexpr double P = 0.1;
constexpr double I = 0.08;
constexpr double D = 0.02;
}  // namespace V
namespace W {
constexpr double P = 5.9;
constexpr double I = 0.0;
constexpr double D = 0.1;
}  // namespace W
}  // namespace sao_agent

#endif  // PUBLICCOURSE_SAO_CONSTANTS_H
