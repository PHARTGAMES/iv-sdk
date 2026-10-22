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