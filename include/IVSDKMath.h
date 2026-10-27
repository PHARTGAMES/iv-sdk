#pragma once
#include "IVSDKCore.h"

inline float Lerp(float a, float b, float t)
{
	return a + (b - a) * t;
}

float MoveTowardsSmooth(
	float Current,
	float Target,
	float Speed,
	float SlowdownDistance,
	float DeltaTime)
{
	float Delta = Target - Current;
	float Distance = abs(Delta);

	if (Distance < 0.0001f)
		return Target;

	float CurrentSpeed = Speed;

	if (SlowdownDistance > 0.0f && Distance < SlowdownDistance)
	{
		CurrentSpeed *= Distance / SlowdownDistance;
	}

	float Step = CurrentSpeed * DeltaTime;

	return Current + copysign(min(Step, Distance), Delta);
}


bool IntersectLineSegmentsYZ(
	const CVector& a_start1,
	const CVector& a_end1,
	const CVector& a_start2,
	const CVector& a_end2,
	CVector& a_intersection)
{
	const float y1 = a_start1.y;
	const float z1 = a_start1.z;
	const float y2 = a_end1.y;
	const float z2 = a_end1.z;

	const float y3 = a_start2.y;
	const float z3 = a_start2.z;
	const float y4 = a_end2.y;
	const float z4 = a_end2.z;

	const float dy1 = y2 - y1;
	const float dz1 = z2 - z1;
	const float dy2 = y4 - y3;
	const float dz2 = z4 - z3;

	const float denominator = dy1 * dz2 - dz1 * dy2;

	// Parallel or collinear lines
	if (abs(denominator) < 0.000001f)
		return false;

	const float dy = y3 - y1;
	const float dz = z3 - z1;

	const float t = (dy * dz2 - dz * dy2) / denominator;
	const float u = (dy * dz1 - dz * dy1) / denominator;

	// Intersection must lie within both segments
	if (t < 0.0f || t > 1.0f || u < 0.0f || u > 1.0f)
		return false;

	a_intersection = CVector(
		a_start1.x + (a_end1.x - a_start1.x) * t,
		y1 + dy1 * t,
		z1 + dz1 * t);

	return true;
}

CVector CalculateAngularVelocity(const CVector& a_previousAngles, const CVector& a_currentAngles, float a_deltaTime)
{
	if (a_deltaTime <= 0.0f)
		return CVector(0.0f, 0.0f, 0.0f);

	auto AngleDelta = [](float a_from, float a_to)
		{
			float delta = std::remainder(a_to - a_from, 360.0f);
			return delta;
		};

	return CVector(
		AngleDelta(a_previousAngles.x, a_currentAngles.x) / a_deltaTime,
		AngleDelta(a_previousAngles.y, a_currentAngles.y) / a_deltaTime,
		AngleDelta(a_previousAngles.z, a_currentAngles.z) / a_deltaTime);
}



CVector CalculateAngularVelocity(const CMatrix& a_previousMatrix, const CMatrix& a_currentMatrix, float a_deltaTime)
{
	if (a_deltaTime <= 0.0f)
		return CVector(0.0f, 0.0f, 0.0f);

	// Calculate the relative rotation in the previous local coordinate system.
	CVector prevRight = a_previousMatrix.right;
	CVector prevForward = a_previousMatrix.at;
	CVector prevUp = a_previousMatrix.up;

	CVector currRight = a_currentMatrix.right;
	CVector currForward = a_currentMatrix.at;
	CVector currUp = a_currentMatrix.up;

	// Relative rotation matrix R = Previous^T * Current
	float r00 = Dot(prevRight, currRight);
	float r01 = Dot(prevRight, currForward);
	float r02 = Dot(prevRight, currUp);

	float r10 = Dot(prevForward, currRight);
	float r11 = Dot(prevForward, currForward);
	float r12 = Dot(prevForward, currUp);

	float r20 = Dot(prevUp, currRight);
	float r21 = Dot(prevUp, currForward);
	float r22 = Dot(prevUp, currUp);

	float cosAngle = std::clamp((r00 + r11 + r22 - 1.0f) * 0.5f, -1.0f, 1.0f);
	float angle = std::acos(cosAngle);

	if (angle < 0.000001f)
		return CVector(0.0f, 0.0f, 0.0f);

	float sinAngle = std::sin(angle);

	// Rotation vector (axis * angle), with small-angle fallback.
	float scale = (std::abs(sinAngle) > 0.0001f)
		? angle / (2.0f * sinAngle)
		: 0.5f;

	CVector rotation(
		(r21 - r12) * scale,
		(r02 - r20) * scale,
		(r10 - r01) * scale);

	const float RadToDeg = 180.0f / 3.14159265358979323846f;
	const float velocityScale = RadToDeg / a_deltaTime;

	return rotation * velocityScale;
}