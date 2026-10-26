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