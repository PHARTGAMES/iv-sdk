#ifndef IVSDK_BUILDING_UMBRELLA
#include "IVSDK.h"
#endif
#ifndef IVSDK_HEADER_CMATRIX_H
#define IVSDK_HEADER_CMATRIX_H
#include <cmath>

class CMatrix
{
public:
	CVector_pad right; // 00-10
	CVector_pad at; // 10-20
    CVector_pad up; // 20-30
    CVector_pad pos; // 30-40
};

//////////////////////////////////////////////////
// Vector / Position Transform
//////////////////////////////////////////////////

// Transform a direction vector (ignores translation)
inline CVector TransformVector(const CMatrix& m, const CVector& v)
{
    return CVector(
        m.right.x * v.x + m.at.x * v.y + m.up.x * v.z,
        m.right.y * v.x + m.at.y * v.y + m.up.y * v.z,
        m.right.z * v.x + m.at.z * v.y + m.up.z * v.z
    );
}

// Transform a position (applies translation)
inline CVector TransformPosition(const CMatrix& m, const CVector& v)
{
    return CVector(
        m.right.x * v.x + m.at.x * v.y + m.up.x * v.z + m.pos.x,
        m.right.y * v.x + m.at.y * v.y + m.up.y * v.z + m.pos.y,
        m.right.z * v.x + m.at.z * v.y + m.up.z * v.z + m.pos.z
    );
}

//////////////////////////////////////////////////
// Matrix Multiply
//////////////////////////////////////////////////

// mOut = a * b
inline CMatrix MultiplyMatrix(const CMatrix& a, const CMatrix& b)
{
    CMatrix out;

    // Rotation/scale part
    out.right = ToPadded(TransformVector(a, b.right));
    out.up = ToPadded(TransformVector(a, b.up));
    out.at = ToPadded(TransformVector(a, b.at));

    // Position = a * b.pos
    out.pos = ToPadded(TransformPosition(a, b.pos));

    return out;
}

inline CMatrix MultiplyMatrix(CMatrix* a, CMatrix* b)
{
    return MultiplyMatrix(*a, *b);
}


//////////////////////////////////////////////////
// Matrix Inverse (orthonormal basis assumed)
//////////////////////////////////////////////////

inline CMatrix InverseMatrix(const CMatrix& m)
{
    CMatrix out;

    // Transpose the rotation (orthonormal assumption)
    out.right = ToPadded(CVector(m.right.x, m.at.x, m.up.x));
    out.at = ToPadded(CVector(m.right.y, m.at.y, m.up.y));
    out.up = ToPadded(CVector(m.right.z, m.at.z, m.up.z));

    // New position = -(R^T * pos)
    CVector invPos = TransformVector(out, CVector(-m.pos.x, -m.pos.y, -m.pos.z));
    out.pos = ToPadded(invPos);

    return out;
}

inline float DegToRad(float deg) { return deg * 3.14159265358979323846f / 180.0f; }
inline float RadToDeg(float rad) { return rad * 180.0f / 3.14159265358979323846f; }



inline CMatrix BuildCameraOffset(float yawDeg, float pitchDeg)
{
    float yaw   = DegToRad(yawDeg);
    float pitch = DegToRad(pitchDeg);

    float sy = sinf(yaw), cy = cosf(yaw);
    float sp = sinf(pitch), cp = cosf(pitch);

    CMatrix m;

    // Forward (at)
    m.at = ToPadded(CVector(
        sy * cp,
        cy * cp,
        sp
    ));

    CVector worldUp(0,0,1);

    // Right = at × worldUp
    m.right = ToPadded(Normalize(Cross(m.at, worldUp)));

    // Up = right × at
    m.up = ToPadded(Normalize(Cross(m.right, m.at)));

    m.pos = ToPadded(CVector(0,0,0));

    return m;
}

inline CVector CameraMatrixToAngles(const CMatrix& a_matrix)
{
    const float RadToDeg = 180.0f / 3.14159265358979323846f;

    const CVector& forward = a_matrix.at;

    float pitch = atan2f(forward.z, sqrtf(forward.x * forward.x + forward.y * forward.y)) * RadToDeg;
    float yaw = atan2f(forward.x, forward.y) * RadToDeg;

    return CVector(pitch, 0.0f, yaw);
}

inline CMatrix BuildRotationFromAxisAngle(const CVector& axis, float angleDeg)
{
    float angle = DegToRad(angleDeg);
    float c = cosf(angle);
    float s = sinf(angle);
    float t = 1.0f - c;

    // Normalize axis
    CVector u = axis;
    float mag = sqrtf(u.x * u.x + u.y * u.y + u.z * u.z);
    if (mag > 1e-6f)
    {
        u.x /= mag;
        u.y /= mag;
        u.z /= mag;
    }

    float x = u.x, y = u.y, z = u.z;

    CMatrix m;

    // Right (X basis)
    m.right = ToPadded(CVector(
        t * x * x + c,
        t * x * y + s * z,
        t * x * z - s * y
    ));

    // At (Y basis, forward)
    m.at = ToPadded(CVector(
        t * x * y - s * z,
        t * y * y + c,
        t * y * z + s * x
    ));

    // Up (Z basis)
    m.up = ToPadded(CVector(
        t * x * z + s * y,
        t * y * z - s * x,
        t * z * z + c
    ));

    // Position
    m.pos = ToPadded(CVector(0, 0, 0));

    return m;
}

inline CMatrix BuildMatrixFromAngles(const CVector& anglesDeg)
{
    CMatrix pitch = BuildRotationFromAxisAngle(
        CVector(1.0f, 0.0f, 0.0f), anglesDeg.x);

    CMatrix roll = BuildRotationFromAxisAngle(
        CVector(0.0f, 1.0f, 0.0f), anglesDeg.y);

    CMatrix yaw = BuildRotationFromAxisAngle(
        CVector(0.0f, 0.0f, 1.0f), anglesDeg.z);

    return MultiplyMatrix(MultiplyMatrix(yaw, roll), pitch);
}

inline CVector ExtractEulerAngles(const CMatrix& m)
{
    CVector angles; // x=pitch, y=roll, z=yaw

    const CVector& at = m.at;    // forward
    const CVector& right = m.right; // right
    const CVector& up = m.up;    // up

    // Pitch: nose up/down from forward Z
    float pitch = asinf(at.z);

    // Yaw: heading from forward XY projection
    CVector atXY = CVector(at.x, at.y, 0.0f).Normalized();
    float yaw = -atan2f(atXY.x, atXY.y);

    // Roll: bank from right & up tilt
    float roll = atan2f(-right.z, up.z);

    // Convert to degrees
    const float rad2deg = 180.0f / 3.14159265358979323846f;
    angles.x = pitch * rad2deg;
    angles.y = roll * rad2deg;
    angles.z = yaw * rad2deg;

    return angles;
}


inline CQuaternion MatrixToQuaternion(const CMatrix& m)
{
    CQuaternion q;

    // GTA IV convention: right=X, at=Y, up=Z
    float R00 = m.right.x, R01 = m.at.x, R02 = m.up.x;
    float R10 = m.right.y, R11 = m.at.y, R12 = m.up.y;
    float R20 = m.right.z, R21 = m.at.z, R22 = m.up.z;

    float trace = R00 + R11 + R22;

    if (trace > 0.0f)
    {
        float s = 0.5f / sqrtf(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = (R21 - R12) * s;
        q.y = (R02 - R20) * s;
        q.z = (R10 - R01) * s;
    }
    else
    {
        if (R00 > R11 && R00 > R22)
        {
            float s = 2.0f * sqrtf(1.0f + R00 - R11 - R22);
            q.w = (R21 - R12) / s;
            q.x = 0.25f * s;
            q.y = (R01 + R10) / s;
            q.z = (R02 + R20) / s;
        }
        else if (R11 > R22)
        {
            float s = 2.0f * sqrtf(1.0f + R11 - R00 - R22);
            q.w = (R02 - R20) / s;
            q.x = (R01 + R10) / s;
            q.y = 0.25f * s;
            q.z = (R12 + R21) / s;
        }
        else
        {
            float s = 2.0f * sqrtf(1.0f + R22 - R00 - R11);
            q.w = (R10 - R01) / s;
            q.x = (R02 + R20) / s;
            q.y = (R12 + R21) / s;
            q.z = 0.25f * s;
        }
    }

    return q;
}

inline CMatrix LerpMatrix(const CMatrix& a_from, const CMatrix& a_to, float a_alpha)
{
    a_alpha = min(max(a_alpha, 0.0f), 1.0f);

    CMatrix result = a_from;

    // Interpolate position
    result.pos = CVector_pad(
        a_from.pos.x + (a_to.pos.x - a_from.pos.x) * a_alpha,
        a_from.pos.y + (a_to.pos.y - a_from.pos.y) * a_alpha,
        a_from.pos.z + (a_to.pos.z - a_from.pos.z) * a_alpha,
        1.0f);

    // Interpolate forward and up vectors
    CVector forward = Lerp(CVector(a_from.at), CVector(a_to.at), a_alpha);
    CVector up = Lerp(CVector(a_from.up), CVector(a_to.up), a_alpha);

    // Rebuild an orthonormal basis
    forward.Normalize();

    CVector right = Normalize(Cross(forward, up));
    up = Normalize(Cross(right, forward));

    result.right = CVector_pad(right);
    result.at = CVector_pad(forward);
    result.up = CVector_pad(up);

    return result;
}

#endif // IVSDK_HEADER_CMATRIX_H
