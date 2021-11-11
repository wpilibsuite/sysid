// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "sysid/Util.h"

TEST(UtilTest, Split) {
  EXPECT_THAT(sysid::Split("", ','), testing::ElementsAre(""));
  EXPECT_THAT(sysid::Split("a", ','), testing::ElementsAre("a"));
  EXPECT_THAT(sysid::Split("a,", ','), testing::ElementsAre("a", ""));
  EXPECT_THAT(sysid::Split("a,b,c", ','), testing::ElementsAre("a", "b", "c"));
  EXPECT_THAT(sysid::Split("abc,de,f", ','),
              testing::ElementsAre("abc", "de", "f"));
  EXPECT_THAT(sysid::Split("abc,de,", ','),
              testing::ElementsAre("abc", "de", ""));
  EXPECT_THAT(sysid::Split(",de,f", ','), testing::ElementsAre("", "de", "f"));
}
