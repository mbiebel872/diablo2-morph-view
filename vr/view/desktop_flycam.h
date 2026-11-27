// Detached camera for Flat third person, with no dependency on the game window.
#pragma once

#include "../../cleanroom/camera/mat4.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace vrc::flycam {

enum MoveKey : uint32_t {
    Forward = 1u << 0, Back = 1u << 1, Left = 1u << 2, Right = 1u << 3,
    Lower = 1u << 4, Raise = 1u << 5, BankLeft = 1u << 6, BankRight = 1u << 7
};

inline uint32_t KeyBit(unsigned key) {
    switch (key) {
    case 'W': case 'w': return Forward;
    case 'S': case 's': return Back;
    case 'A': case 'a': return Left;
    case 'D': case 'd': return Right;
    case 'Q': case 'q': return Lower;
    case 'E': case 'e': return Raise;
    case 'Z': case 'z': return BankLeft;
    case 'C': case 'c': return BankRight;
    default: return 0;
    }
}

using d2rcam::m4::V3;
inline constexpr float kRadiansPerDegree = 3.14159265f / 180.0f;
inline constexpr float kPitchLimit = 1.55f;

struct Pose {
    V3 eye{};
    float heading = 0.0f, down = 0.0f, roll = 0.0f;

    void Reset(V3 position, V3 forward, V3 up) {
        using namespace d2rcam::m4;
        eye = position;
        heading = std::atan2(-forward.x, -forward.z);
        down = std::atan2(-forward.y, std::hypot(forward.x, forward.z));
        const V3 lateral{std::cos(heading), 0.0f, -std::sin(heading)};
        roll = std::atan2(Dot(up, lateral), Dot(up, Cross(lateral, forward)));
    }

    void Step(uint32_t keys, float dt, float yawDegrees, float pitchDegrees,
              V3& forward, V3& ahead, V3& right, V3& up) {
        using namespace d2rcam::m4;
        constexpr float kSpeed = 24.0f;       // world units per second
        constexpr float kBankSpeed = 70.0f;   // degrees per second
        constexpr float kTurn = 360.0f * kRadiansPerDegree;
        const auto axis = [keys](MoveKey positive, MoveKey negative) {
            return float((keys & positive) != 0) - float((keys & negative) != 0);
        };
        heading = std::remainder(heading - yawDegrees * kRadiansPerDegree, kTurn);
        down = std::clamp(down + pitchDegrees * kRadiansPerDegree, -kPitchLimit, kPitchLimit);
        roll = std::remainder(roll + axis(BankRight, BankLeft) * kBankSpeed * kRadiansPerDegree * dt, kTurn);

        ahead = {-std::sin(heading), 0.0f, -std::cos(heading)};
        const V3 lateral{-ahead.z, 0.0f, ahead.x};
        forward = {ahead.x * std::cos(down), -std::sin(down), ahead.z * std::cos(down)};
        const V3 levelUp = Cross(lateral, forward);
        up = levelUp * std::cos(roll) + lateral * std::sin(roll);
        right = lateral * std::cos(roll) - levelUp * std::sin(roll);

        const V3 move = forward * axis(Forward, Back) + lateral * axis(Right, Left)
                      + V3{0.0f, axis(Raise, Lower), 0.0f};
        eye = eye + move * (dt * kSpeed / std::max(1.0f, Length(move)));
    }
};

bool Available();
bool Active();
bool BlocksHeroKeys();
void Toggle();
void Stop(bool releaseKeys = false);
bool TakeKey(unsigned key, bool down, bool repeat);
bool OwnsKey(unsigned key);
void AddLook(float yawDegrees, float pitchDegrees);

}   // namespace vrc::flycam
