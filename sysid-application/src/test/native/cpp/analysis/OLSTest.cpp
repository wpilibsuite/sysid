// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <vector>

#include "gtest/gtest.h"
#include "sysid/analysis/OLS.h"

TEST(OLS, TwoVariablesTwoPoints) {
  // (1, 3) and (2, 5). Should produce y = 2x + 1.
  std::vector<double> data{3.0, 1.0, 1.0, 5.0, 1.0, 2.0};

  auto [coefficients, cod] = sysid::OLS(data, 2);
  EXPECT_EQ(coefficients.size(), 2u);

  EXPECT_NEAR(coefficients[0], 1.0, 0.05);
  EXPECT_NEAR(coefficients[1], 2.0, 0.05);
  EXPECT_NEAR(cod, 1.0, 1E-4);
}

TEST(OLS, TwoVariablesFivePoints) {
  // (2, 4), (3, 5), (5, 7), (7, 10), (9, 15)
  // Should produce 1.518x + 0.305.
  std::vector<double> data{4, 1, 2, 5, 1, 3, 7, 1, 5, 10, 1, 7, 15, 1, 9};

  auto [coefficients, cod] = sysid::OLS(data, 2);
  EXPECT_EQ(coefficients.size(), 2u);

  EXPECT_NEAR(coefficients[0], 0.305, 0.05);
  EXPECT_NEAR(coefficients[1], 1.518, 0.05);
  EXPECT_NEAR(cod, 0.985, 0.05);
}

#ifndef NDEBUG
TEST(OLS, MalformedData) {
  std::vector<double> data{4, 1, 2, 5, 1};
  EXPECT_DEATH(sysid::OLS(data, 2), "");
}
#endif
