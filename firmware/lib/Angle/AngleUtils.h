#pragma once
/**
 * @file AngleUtils.h
 * @brief Angle wrapping, shortest-path error and continuous heading unwrapping.
 */

#include <cmath>

namespace angle {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kTwoPi    = 2.0 * kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

/**
 * Wrap any angle (deg) into [-180, +180).
 */
inline double wrap180(double deg) {
  double x = std::fmod(deg + 180.0, 360.0);
  if (x < 0.0) x += 360.0;
  return x - 180.0;
}

/** Same as wrap180() but in radians: result in [-pi, +pi). */
inline double wrapPi(double rad) {
  double x = std::fmod(rad + kPi, kTwoPi);
  if (x < 0.0) x += kTwoPi;
  return x - kPi;
}

/** Wrap into [0, 360). Handy for display / logging only - NOT for control. */
inline double wrap360(double deg) {
  double x = std::fmod(deg, 360.0);
  if (x < 0.0) x += 360.0;
  return x;
}

/**
 * Shortest signed angular error (deg): choosing the shorter of the two possible directions.
 *
 *   result in [-180, +180);  positive => turn in the positive direction.
 *
 * Example: target=10, current=350  ->  +20 (not -340).
 */
inline double shortestError(double targetDeg, double currentDeg) {
  return wrap180(targetDeg - currentDeg);
}

/**
 * Express `targetDeg` in the SAME continuous "turn count" as `currentDeg`
 */
inline double nearestEquivalent(double targetDeg, double currentDeg) {
  return currentDeg + shortestError(targetDeg, currentDeg);
}

class HeadingUnwrapper {
 public:
  /**
   * @param startAtZero true  -> first sample defines heading 0 (relative frame)
   *                    false -> first sample keeps its absolute value (0..360)
   */
  explicit HeadingUnwrapper(bool startAtZero = true) : startAtZero_(startAtZero) {}

  /** Feed a new raw heading in degrees; returns the unwrapped heading (deg). */
  double update(double rawDeg) {
    if (!initialised_) {
      initialised_ = true;
      unwrapped_ = startAtZero_ ? 0.0 : rawDeg;
    } else {
      unwrapped_ += wrap180(rawDeg - lastRaw_);
    }
    lastRaw_ = rawDeg;
    return unwrapped_;
  }

  /** Redefine the current heading as 0 without losing the last raw sample. */
  void zero() { unwrapped_ = 0.0; }

  /** Forget everything; the next sample re-initialises the unwrapper. */
  void reset() { initialised_ = false; unwrapped_ = 0.0; lastRaw_ = 0.0; }

  bool   initialised()   const { return initialised_; }
  double unwrappedDeg()  const { return unwrapped_; }
  double lastRawDeg()    const { return lastRaw_; }

  /** Signed number of complete turns accumulated */
  double turns()         const { return unwrapped_ / 360.0; }

 private:
  bool   startAtZero_;
  bool   initialised_ = false;
  double unwrapped_   = 0.0;
  double lastRaw_     = 0.0;
};

}