#include "apex/estimation/lateral_model.hpp"

#include <cmath>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"

namespace apex::estimation {
namespace {

// The magic formula of core/vehicle.hpp, evaluated forward.
double magic_formula(const TireParams& tire, double fz, double alpha) {
    const double b = tire.B * alpha;
    return -tire.mu * fz * std::sin(tire.C * std::atan(b - tire.E * (b - std::atan(b))));
}

TEST(LateralModel, SlipAngleInvertsTheMagicFormula) {
    for (const double e : {0.0, -0.5, 0.3}) {
        TireParams tire;
        tire.E = e;
        for (const double alpha : {-0.08, -0.02, 0.0005, 0.01, 0.05, 0.09}) {
            const double fy = magic_formula(tire, 1500.0, alpha);
            const SlipAngle slip = slip_angle_for_force(tire, 1500.0, fy);
            EXPECT_NEAR(slip.alpha, alpha, 1e-9) << e << " " << alpha;
            EXPECT_NEAR(slip.utilization, std::abs(fy) / (tire.mu * 1500.0), 1e-12);
        }
    }
}

TEST(LateralModel, ForcesBeyondThePeakSaturate) {
    const TireParams tire;
    const SlipAngle at_peak = slip_angle_for_force(tire, 1000.0, tire.mu * 1000.0);
    const SlipAngle beyond = slip_angle_for_force(tire, 1000.0, 2.0 * tire.mu * 1000.0);
    EXPECT_GE(beyond.utilization, 1.0);
    EXPECT_NEAR(beyond.alpha, at_peak.alpha, 1e-9);
    EXPECT_LT(beyond.alpha, 0.0);  // a positive force needs a negative slip angle
    const SlipAngle none = slip_angle_for_force(tire, 1000.0, 0.0);
    EXPECT_EQ(none.alpha, 0.0);
    EXPECT_EQ(none.utilization, 0.0);
}

TEST(LateralModel, AxleLoadsBalanceWeightDownforceAndTransfer) {
    const VehicleParams car;
    const double weight = car.mass * kGravity;
    const AxleLoads still = axle_normal_loads(car, 0.0, 0.0);
    EXPECT_NEAR(still.front + still.rear, weight, 1e-9);
    EXPECT_NEAR(still.front / weight, car.lr / car.wheelbase(), 1e-12);
    const AxleLoads fast = axle_normal_loads(car, 20.0, 0.0);
    EXPECT_NEAR(fast.front + fast.rear, weight + car.downforce(20.0), 1e-9);
    // Braking moves load forward.
    const AxleLoads braking = axle_normal_loads(car, 20.0, -10.0);
    EXPECT_NEAR(braking.front - fast.front, car.mass * 10.0 * car.cg_height / car.wheelbase(),
                1e-9);
    EXPECT_NEAR(braking.front + braking.rear, fast.front + fast.rear, 1e-9);
}

TEST(LateralModel, SteadyCorneringSplitsTheForceByTheCogPosition) {
    const VehicleParams car;
    const double ay = 8.0;
    const AxleSlip slip = predict_axle_slip(car, 15.0, 0.0, ay, 0.0);
    const AxleLoads loads = axle_normal_loads(car, 15.0, 0.0);
    // Turning left (ay > 0) takes positive lateral forces: negative slip angles.
    EXPECT_LT(slip.front.alpha, 0.0);
    EXPECT_LT(slip.rear.alpha, 0.0);
    EXPECT_NEAR(magic_formula(car.front_tire, loads.front, slip.front.alpha),
                car.mass * ay * car.lr / car.wheelbase(), 1e-6);
    EXPECT_NEAR(magic_formula(car.rear_tire, loads.rear, slip.rear.alpha),
                car.mass * ay * car.lf / car.wheelbase(), 1e-6);
    // A yaw acceleration loads the front and unloads the rear.
    const AxleSlip turning_in = predict_axle_slip(car, 15.0, 0.0, ay, 5.0);
    EXPECT_GT(turning_in.front.utilization, slip.front.utilization);
    EXPECT_LT(turning_in.rear.utilization, slip.rear.utilization);
}

}  // namespace
}  // namespace apex::estimation
