#include "subsystemHeaders/poseControl.hpp"
#include "subsystemHeaders/globals.hpp"
#include "lemlib/pose.hpp"

/**
 * @brief Set the robot's pose and synchronize both EKF and chassis
 * 
 * When you need to reset the robot's position (e.g., at the start of autonomous),
 * use this function instead of chassis.setPose() directly. This ensures that:
 * 1. The EKF is reset to the new pose
 * 2. The chassis is set to the new pose
 * 3. Both remain synchronized
 * 
 * @param x X position in inches
 * @param y Y position in inches  
 * @param theta Heading in degrees
 */
void setRobotPose(double x, double y, double theta) {
    // First, update the EKF to the new pose
    ekf.setPose(x, y, theta);
    
    // Then update the chassis
    // Note: The EKF task will continue to update chassis.setPose() every 10ms,
    // but this immediate update ensures the pose is set right away
    lemlib::Pose pose(x, y, theta);
    chassis.setPose(pose);
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

