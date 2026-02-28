#pragma once

/**
 * @file ekfConfig.hpp
 * @brief Configuration for EKF (Extended Kalman Filter)
 * 
 * Set ENABLE_EKF to true to use EKF-filtered odometry
 * Set ENABLE_EKF to false to use raw LemLib odometry (for comparison/debugging)
 */

// ============================================================================
// CONFIGURATION: Change this to enable/disable EKF
// ============================================================================

// Set to true to enable EKF filtering
// Set to false to use raw LemLib odometry (no EKF)
#define ENABLE_EKF false

// ============================================================================
// Logging Configuration
// ============================================================================

// Enable detailed logging to PROS terminal
#define ENABLE_EKF_LOGGING true

// Log interval in milliseconds (how often to print logs)
#define EKF_LOG_INTERVAL_MS 500

// ============================================================================
// DO NOT MODIFY BELOW THIS LINE
// ============================================================================

#if ENABLE_EKF
    #define EKF_STATUS_TEXT "EKF: ENABLED"
#else
    #define EKF_STATUS_TEXT "EKF: DISABLED (Raw LemLib)"
#endif