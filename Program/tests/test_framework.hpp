#pragma once

#include <iostream>
#include <string>
#include <cmath>

// ANSI color codes for terminal output
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_CYAN    "\033[36m"

// Global test counters
static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;
static std::string g_current_test_name = "";

// Test macros
#define TEST_START(name) \
    do { \
        g_current_test_name = name; \
        g_tests_run++; \
        std::cout << COLOR_CYAN << "[TEST] " << COLOR_RESET << name << "..." << std::endl; \
    } while(0)

#define TEST_PASS() \
    do { \
        g_tests_passed++; \
        std::cout << COLOR_GREEN << "  ✓ PASS" << COLOR_RESET << std::endl << std::endl; \
    } while(0)

#define TEST_FAIL(message) \
    do { \
        g_tests_failed++; \
        std::cout << COLOR_RED << "  ✗ FAIL: " << message << COLOR_RESET << std::endl << std::endl; \
        return; \
    } while(0)

#define ASSERT_TRUE(condition, message) \
    do { \
        if (!(condition)) { \
            std::cout << COLOR_RED << "  ✗ Assertion failed: " << message << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Expected: true" << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Got: false" << COLOR_RESET << std::endl; \
            TEST_FAIL(message); \
        } \
    } while(0)

#define ASSERT_FALSE(condition, message) \
    do { \
        if (condition) { \
            std::cout << COLOR_RED << "  ✗ Assertion failed: " << message << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Expected: false" << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Got: true" << COLOR_RESET << std::endl; \
            TEST_FAIL(message); \
        } \
    } while(0)

#define ASSERT_EQUAL(actual, expected, message) \
    do { \
        if ((actual) != (expected)) { \
            std::cout << COLOR_RED << "  ✗ Assertion failed: " << message << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Expected: " << (expected) << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Got: " << (actual) << COLOR_RESET << std::endl; \
            TEST_FAIL(message); \
        } \
    } while(0)

#define ASSERT_APPROX_EQUAL(actual, expected, tolerance, message) \
    do { \
        double _diff = std::abs((actual) - (expected)); \
        if (_diff > (tolerance)) { \
            std::cout << COLOR_RED << "  ✗ Assertion failed: " << message << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Expected: " << (expected) << " (±" << (tolerance) << ")" << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Got: " << (actual) << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Difference: " << _diff << COLOR_RESET << std::endl; \
            TEST_FAIL(message); \
        } \
    } while(0)

#define ASSERT_NOT_EQUAL(actual, expected, message) \
    do { \
        if ((actual) == (expected)) { \
            std::cout << COLOR_RED << "  ✗ Assertion failed: " << message << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Expected: NOT " << (expected) << COLOR_RESET << std::endl; \
            std::cout << COLOR_YELLOW << "    Got: " << (actual) << COLOR_RESET << std::endl; \
            TEST_FAIL(message); \
        } \
    } while(0)

#define PRINT_TEST_SUMMARY() \
    do { \
        std::cout << "========================================\n"; \
        std::cout << "  Test Summary\n"; \
        std::cout << "========================================\n"; \
        std::cout << "Total tests: " << g_tests_run << std::endl; \
        std::cout << COLOR_GREEN << "Passed: " << g_tests_passed << COLOR_RESET << std::endl; \
        if (g_tests_failed > 0) { \
            std::cout << COLOR_RED << "Failed: " << g_tests_failed << COLOR_RESET << std::endl; \
        } else { \
            std::cout << "Failed: " << g_tests_failed << std::endl; \
        } \
        std::cout << "========================================\n"; \
        if (g_tests_failed == 0) { \
            std::cout << COLOR_GREEN << "All tests passed! ✓" << COLOR_RESET << std::endl; \
        } else { \
            std::cout << COLOR_RED << "Some tests failed! ✗" << COLOR_RESET << std::endl; \
        } \
        std::cout << std::endl; \
    } while(0)

#define TEST_RETURN_CODE() (g_tests_failed > 0 ? 1 : 0)

