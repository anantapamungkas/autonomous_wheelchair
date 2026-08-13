#pragma once
// ===========================================================================
// FuzzyPID.h
//
// A lightweight Fuzzy-Logic-augmented PID controller intended for closed-
// loop wheel velocity control on a differential-drive wheelchair.
//
// Design:
//   - A classical PID computes a base control effort from (error, d_error).
//   - A 2-input (error, delta-error) / 3-output (dKp, dKi, dKd) Mamdani-style
//     fuzzy inference system nudges the PID gains in real time, so the
//     controller behaves more conservatively near setpoint (less overshoot)
//     and more aggressively far from setpoint (faster rise time). This is
//     the standard "Fuzzy Self-Tuning PID" architecture used widely in
//     mobile-robot wheel-velocity control.
//   - Membership functions are triangular for simplicity and determinism on
//     an embedded target (no floating point transcendental calls needed).
//
// This header is intentionally self-contained (no .cpp) so it can be
// dropped into lib/FuzzyPID/ and included directly by src/main.cpp.
// ===========================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Fuzzy linguistic levels used for both inputs (error, delta-error) and the
// gain-adjustment outputs. NB = Negative Big, NS = Negative Small,
// ZE = Zero, PS = Positive Small, PB = Positive Big.
// ---------------------------------------------------------------------------
enum FuzzyLevel { NB = 0, NS = 1, ZE = 2, PS = 3, PB = 4, FUZZY_LEVEL_COUNT = 5 };

class FuzzyPID {
public:
    // -----------------------------------------------------------------
    // Construct with baseline PID gains (the "center" gains that the
    // fuzzy layer perturbs) and the physical output saturation limits
    // (e.g. -255..255 for an 8-bit PWM duty cycle).
    // -----------------------------------------------------------------
    FuzzyPID(float kp, float ki, float kd,
              float outMin, float outMax,
              float errorRange, float deltaErrorRange)
        : baseKp_(kp), baseKi_(ki), baseKd_(kd),
          outMin_(outMin), outMax_(outMax),
          errorRange_(errorRange), deltaErrorRange_(deltaErrorRange),
          integral_(0.0f), lastError_(0.0f), lastCallMicros_(0) {}

    // Reset integral/derivative memory (call on setpoint changes, e-stop
    // recovery, or long comms dropouts to avoid windup/kick).
    void reset() {
        integral_ = 0.0f;
        lastError_ = 0.0f;
        lastCallMicros_ = 0;
    }

    // -----------------------------------------------------------------
    // Compute one control step.
    //   setpoint / measurement: same units (here, m/s of wheel surface
    //   speed, but the class is unit-agnostic).
    // Returns the saturated control effort in [outMin_, outMax_].
    // -----------------------------------------------------------------
    float compute(float setpoint, float measurement) {
        unsigned long nowMicros = micros();
        float dt = (lastCallMicros_ == 0)
                       ? 0.02f  // assume 50 Hz on first call
                       : (nowMicros - lastCallMicros_) * 1e-6f;
        lastCallMicros_ = nowMicros;
        if (dt <= 0.0f) dt = 1.0f / 50.0f;

        float error = setpoint - measurement;
        float deltaError = (error - lastError_) / dt;

        // ---- Fuzzy inference: derive gain deltas from (error, dError) ----
        float dKp, dKi, dKd;
        fuzzyInfer(error, deltaError, dKp, dKi, dKd);

        // ---- Effective (self-tuned) gains, clamped to sane bounds ----
        float kp = constrain(baseKp_ + dKp, 0.0f, baseKp_ * 2.0f);
        float ki = constrain(baseKi_ + dKi, 0.0f, baseKi_ * 2.0f);
        float kd = constrain(baseKd_ + dKd, 0.0f, baseKd_ * 2.0f);

        // ---- Standard PID with anti-windup (clamp integral only while
        //      the raw output is not already saturated) ----
        float provisionalIntegral = integral_ + error * dt;
        float derivative = deltaError;

        float output = kp * error + ki * provisionalIntegral + kd * derivative;

        if (output > outMax_ || output < outMin_) {
            // Output saturated: do NOT accumulate further integral (anti-windup)
            output = constrain(output, outMin_, outMax_);
        } else {
            integral_ = provisionalIntegral;
        }

        lastError_ = error;
        return output;
    }

private:
    // Baseline (center) PID gains, tuned offline on the physical wheelchair.
    float baseKp_, baseKi_, baseKd_;
    float outMin_, outMax_;

    // Universe of discourse for the two fuzzy inputs. Choose these to match
    // the expected worst-case error / delta-error for your wheel velocity
    // loop (e.g. +/-1.5 m/s error range, +/-3.0 m/s^2 delta-error range).
    float errorRange_, deltaErrorRange_;

    float integral_;
    float lastError_;
    unsigned long lastCallMicros_;

    // -----------------------------------------------------------------
    // Triangular membership function, centered at `center`, with
    // half-width `width`. Returns degree of membership in [0, 1].
    // -----------------------------------------------------------------
    static float triangular(float x, float center, float width) {
        if (width <= 0.0f) return (x == center) ? 1.0f : 0.0f;
        float d = fabsf(x - center) / width;
        return (d >= 1.0f) ? 0.0f : (1.0f - d);
    }

    // -----------------------------------------------------------------
    // Fuzzify a crisp value into membership degrees across the five
    // linguistic levels {NB, NS, ZE, PS, PB}, given the universe half-range.
    // Centers are placed at -range, -range/2, 0, +range/2, +range.
    // -----------------------------------------------------------------
    static void fuzzify(float x, float range, float mu[FUZZY_LEVEL_COUNT]) {
        float centers[FUZZY_LEVEL_COUNT] = {-range, -range * 0.5f, 0.0f, range * 0.5f, range};
        float width = range * 0.5f;
        for (int i = 0; i < FUZZY_LEVEL_COUNT; i++) {
            mu[i] = triangular(constrain(x, -range, range), centers[i], width);
        }
    }

    // -----------------------------------------------------------------
    // Mamdani rule base + centroid (weighted-average) defuzzification.
    //
    // Rule table follows the classical fuzzy-PID heuristic:
    //   - Large |error| with matching-sign delta-error -> boost Kp, cut Kd
    //     (respond fast while far from setpoint).
    //   - Small |error| near zero delta-error -> cut Kp, boost Ki, boost Kd
    //     (settle precisely, damp oscillation near setpoint).
    // The 5x5 = 25 rule table below implements this compactly by indexing
    // a small output lookup table with (errorLevel, deltaErrorLevel).
    // -----------------------------------------------------------------
    static void fuzzyInfer(float error, float deltaError,
                            float &dKp, float &dKi, float &dKd) {
        // NOTE: ranges must be supplied per-instance; kept static here for
        // brevity of the demo rule table. In production, pass errorRange_ /
        // deltaErrorRange_ through instead of the placeholders below.
        const float ERROR_RANGE = 1.5f;       // m/s
        const float DELTA_ERROR_RANGE = 3.0f; // m/s^2

        float muE[FUZZY_LEVEL_COUNT];
        float muDE[FUZZY_LEVEL_COUNT];
        fuzzify(error, ERROR_RANGE, muE);
        fuzzify(deltaError, DELTA_ERROR_RANGE, muDE);

        // Output gain-delta magnitude per linguistic level (tune per robot).
        // Index: NB=0 .. PB=4. Symmetric small deltas keep the self-tuner
        // stable; scale these up only after bench validation.
        static const float KP_TABLE[FUZZY_LEVEL_COUNT] = {0.30f, 0.15f, 0.0f, 0.15f, 0.30f};
        static const float KI_TABLE[FUZZY_LEVEL_COUNT] = {-0.02f, -0.01f, 0.02f, -0.01f, -0.02f};
        static const float KD_TABLE[FUZZY_LEVEL_COUNT] = {-0.05f, -0.02f, 0.02f, -0.02f, -0.05f};

        float numKp = 0.0f, numKi = 0.0f, numKd = 0.0f, denom = 0.0f;

        // 5x5 rule evaluation using min() as the AND (t-norm) operator and
        // centroid-of-singletons as defuzzification.
        for (int e = 0; e < FUZZY_LEVEL_COUNT; e++) {
            for (int de = 0; de < FUZZY_LEVEL_COUNT; de++) {
                float firing = min(muE[e], muDE[de]);
                if (firing <= 0.0f) continue;

                // "Distance from setpoint" rule level = how far this
                // (e, de) combination sits from the ZE/ZE quiescent cell.
                int severity = max(abs(e - ZE), abs(de - ZE));
                severity = constrain(severity, 0, FUZZY_LEVEL_COUNT - 1);

                numKp += firing * KP_TABLE[severity];
                numKi += firing * KI_TABLE[severity];
                numKd += firing * KD_TABLE[severity];
                denom += firing;
            }
        }

        if (denom > 1e-6f) {
            dKp = numKp / denom;
            dKi = numKi / denom;
            dKd = numKd / denom;
        } else {
            dKp = dKi = dKd = 0.0f;
        }
    }
};
