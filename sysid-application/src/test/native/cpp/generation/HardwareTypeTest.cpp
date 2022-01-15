// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <array>
#include <functional>
#include <stdexcept>
#include <string_view>

#include "gtest/gtest.h"
#include "sysid/generation/HardwareType.h"

/**
 * Tests to see if a HardwareType conversion function works on a given test
 * array. It iterates through the test array and sees if the function returns
 * the right HardwareType when given the string associated with said
 * HardwareType.
 *
 * @tparam S The size of the test array
 *
 * @param testArray The test array of HardwareTypes
 * @param func The function that should return the corresponding HardwareType
 *             based off of a passed string
 */
template <size_t S>
static void TestConversionFunction(
    std::array<sysid::HardwareType, S> testArray,
    std::function<sysid::HardwareType(std::string_view)> func) {
  for (auto& it : testArray) {
    auto hardwareType = func(it.name);
    EXPECT_EQ(hardwareType, it);
  }
}

TEST(HardwareTypeTest, FromMotorControllerName) {
  TestConversionFunction(sysid::motorcontroller::kMotorControllers,
                         sysid::motorcontroller::FromMotorControllerName);
  EXPECT_THROW(sysid::motorcontroller::FromMotorControllerName(""),
               std::runtime_error);
}

TEST(HardwareTypeTest, FromEncoderName) {
  TestConversionFunction(sysid::encoder::kEncoders,
                         sysid::encoder::FromEncoderName);
  EXPECT_THROW(sysid::encoder::FromEncoderName(""), std::runtime_error);
}

TEST(HardwareTypeTest, FromGyroName) {
  TestConversionFunction(sysid::gyro::kGyros, sysid::gyro::FromGyroName);
  EXPECT_THROW(sysid::gyro::FromGyroName(""), std::runtime_error);
}
