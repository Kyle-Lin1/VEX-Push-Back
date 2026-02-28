#include "subsystemHeaders/poseControl.hpp"
#include "subsystemHeaders/globals.hpp"
#include "subsystemHeaders/ekfConfig.hpp"
#include "lemlib/pose.hpp"
#include <cstdio>

/**
 * @brief Set the robot's pose and synchronize EKF, chassis, and IMU
 *
 * When you need to reset the robot's position (e.g., at the start of autonomous),
 * use this function instead of chassis.setPose() directly. This ensures that:
 * 1. The IMU heading is reset to match the desired heading
 * 2. The EKF is reset to the new pose
 * 3. The chassis is set to the new pose
 * 4. All three remain synchronized
 *
 * This prevents the EKF from immediately overwriting the pose with stale IMU data.
 *
 * @param x X position in inches
 * @param y Y position in inches
 * @param theta Heading in degrees
 */
void setRobotPose(double x, double y, double theta) {
    printf("\n[POSE] setRobotPose called: X=%.2f Y=%.2f Theta=%.2f (EKF: %s)\n",
           x, y, theta, ENABLE_EKF ? "ON" : "OFF");

#if ENABLE_EKF
    // CRITICAL: Reset IMU heading first to prevent it from overwriting our pose
    // The IMU uses 0-360 range, so convert negative angles
    double imu_heading = theta;
    if (imu_heading < 0) {
        imu_heading += 360.0;
    }

    printf("[POSE] Setting IMU heading to: %.2f\n", imu_heading);
    imu.set_heading(imu_heading);

    // Verify IMU was set correctly
    double actual_imu = imu.get_heading();
    printf("[POSE] IMU heading after set: %.2f\n", actual_imu);

    // Update the EKF to the new pose
    printf("[POSE] Setting EKF pose\n");
    ekf.setPose(x, y, theta);

    // Update the chassis
    lemlib::Pose pose(x, y, theta);
    chassis.setPose(pose);
    printf("[POSE] Chassis pose set\n");

    // Small delay to ensure IMU heading is set before EKF task reads it
    pros::delay(20);

    // Verify final pose
    lemlib::Pose final_pose = chassis.getPose();
    printf("[POSE] Final pose: X=%.2f Y=%.2f Theta=%.2f\n\n",
           final_pose.x, final_pose.y, final_pose.theta);
#else
    // EKF disabled - just set chassis pose directly (raw LemLib)
    lemlib::Pose pose(x, y, theta);
    chassis.setPose(pose);
    printf("[POSE] Chassis pose set (raw LemLib, no EKF)\n");

    // Verify
    lemlib::Pose final_pose = chassis.getPose();
    printf("[POSE] Final pose: X=%.2f Y=%.2f Theta=%.2f\n\n",
           final_pose.x, final_pose.y, final_pose.theta);
#endif
}

/**
 * @brief Get the current robot pose
 * 
 * This returns the chassis pose, which is continuously updated by the EKF.
 * 
 * @return lemlib::Pose The current pose (EKF-filtered)
 */
lemlib::Pose getRobotPose() {
    return chassis.getPose();
}