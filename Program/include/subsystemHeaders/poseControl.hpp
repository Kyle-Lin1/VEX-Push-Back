#pragma once

#include "lemlib/pose.hpp"

/**
 * @file poseControl.hpp
 * @brief Unified pose control that keeps EKF and LemLib chassis synchronized
 *
 * This module provides functions to set the robot's pose in a way that keeps
 * both the Extended Kalman Filter and LemLib's chassis odometry in sync.
 */

/**
 * @brief Set the robot's pose and synchronize both EKF and chassis
 *
 * This function should be used instead of directly calling chassis.setPose()
 * to ensure the EKF and chassis remain synchronized.
 *
 * @param x X position in inches
 * @param y Y position in inches
 * @param theta Heading in degrees
 */
void setRobotPose(double x, double y, double theta);

/**
 * @brief Get the current robot pose (from chassis, which is updated by EKF)
 *
 * @return lemlib::Pose The current pose
 */
lemlib::Pose getRobotPose();

