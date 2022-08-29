// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <imgui.h>

namespace sysid {
/**
 * constexpr shim for ImVec2.
 */
struct Vector2d {
  /**
   * X coordinate.
   */
  float x = 0;

  /**
   * Y coordinate.
   */
  float y = 0;

  constexpr Vector2d(const Vector2d&) = default;
  constexpr Vector2d& operator=(const Vector2d&) = default;

  constexpr Vector2d(Vector2d&&) = default;
  constexpr Vector2d& operator=(Vector2d&&) = default;

  /**
   * Vector2d addition operator.
   *
   * @param rhs Vector to add.
   * @return Sum of two vectors.
   */
  constexpr Vector2d operator+(const Vector2d& rhs) const {
    return Vector2d{x + rhs.x, y + rhs.y};
  }

  /**
   * Vector2d subtraction operator.
   *
   * @param rhs Vector to subtract.
   * @return Difference of two vectors.
   */
  constexpr Vector2d operator-(const Vector2d& rhs) const {
    return Vector2d{x - rhs.x, y - rhs.y};
  }

  /**
   * Conversion operator to ImVec2.
   */
  explicit operator ImVec2() const { return ImVec2{x, y}; }
};

// App window size
static constexpr Vector2d kAppWindowSize{1280, 720};

// Menubar height
static constexpr int kMenubarHeight = 20;

// Gap between window edges
static constexpr int kWindowGap = 5;

// Left column position and size
static constexpr Vector2d kLeftColPos{kWindowGap, kMenubarHeight + kWindowGap};
static constexpr Vector2d kLeftColSize{
    310, kAppWindowSize.y - kLeftColPos.y - kWindowGap};

// Left column contents
static constexpr Vector2d kGeneratorWindowPos = kLeftColPos;
static constexpr Vector2d kGeneratorWindowSize{kLeftColSize.x, 300};
static constexpr Vector2d kLoggerWindowPos =
    kGeneratorWindowPos + Vector2d{0, kGeneratorWindowSize.y + kWindowGap};
static constexpr Vector2d kLoggerWindowSize{
    kLeftColSize.x, kAppWindowSize.y - kWindowGap - kLoggerWindowPos.y};

// Center column position and size
static constexpr Vector2d kCenterColPos =
    kLeftColPos + Vector2d{kLeftColSize.x + kWindowGap, 0};
static constexpr Vector2d kCenterColSize{
    360, kAppWindowSize.y - kLeftColPos.y - kWindowGap};

// Center column contents
static constexpr Vector2d kAnalyzerWindowPos = kCenterColPos;
static constexpr Vector2d kAnalyzerWindowSize{kCenterColSize.x, 550};
static constexpr Vector2d kProgramLogWindowPos =
    kAnalyzerWindowPos + Vector2d{0, kAnalyzerWindowSize.y + kWindowGap};
static constexpr Vector2d kProgramLogWindowSize{
    kCenterColSize.x, kAppWindowSize.y - kWindowGap - kProgramLogWindowPos.y};

// Right column position and size
static constexpr Vector2d kRightColPos =
    kCenterColPos + Vector2d{kCenterColSize.x + kWindowGap, 0};
static constexpr Vector2d kRightColSize =
    kAppWindowSize - kRightColPos - Vector2d{kWindowGap, kWindowGap};

// Right column contents
static constexpr Vector2d kDiagnosticPlotWindowPos = kRightColPos;
static constexpr Vector2d kDiagnosticPlotWindowSize = kRightColSize;

// Text box width as a multiple of the font size
static constexpr int kTextBoxWidthMultiple = 10;
}  // namespace sysid
