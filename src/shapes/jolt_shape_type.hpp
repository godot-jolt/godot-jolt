#pragma once

// NOLINTNEXTLINE(readability-identifier-naming)
namespace JoltShapeType {

constexpr JPH::EShapeType EMPTY = JPH::EShapeType::User1;

} // namespace JoltShapeType

// NOLINTNEXTLINE(readability-identifier-naming)
namespace JoltShapeSubType {

constexpr JPH::EShapeSubType EMPTY = JPH::EShapeSubType::User1;
constexpr JPH::EShapeSubType OVERRIDE_USER_DATA = JPH::EShapeSubType::User2;
constexpr JPH::EShapeSubType RAY = JPH::EShapeSubType::UserConvex1;
constexpr JPH::EShapeSubType MOTION = JPH::EShapeSubType::UserConvex2;

} // namespace JoltShapeSubType
