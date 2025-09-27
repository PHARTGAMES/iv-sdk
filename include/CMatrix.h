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

inline CMatrix BuildMatrixFromAngles(const CVector& anglesDeg)
{
    // Angles in degrees: angles.x = pitch, angles.y = yaw, angles.z = roll
    float pitch = DegToRad(anglesDeg.x); // around Y (forward)
    float yaw = DegToRad(anglesDeg.y); // around Z (up)
    float roll = DegToRad(anglesDeg.z); // around X (right)

    // Precompute sines/cosines
    float sp = sinf(pitch), cp = cosf(pitch);
    float sy = sinf(yaw), cy = cosf(yaw);
    float sr = sinf(roll), cr = cosf(roll);

    CMatrix m;

    // Rotation matrix (right, at, up)
    m.right = ToPadded(CVector(
        cy * cr + sy * sp * sr,
        sy * cp,
        cy * -sr + sy * sp * cr
    ));

    m.at = ToPadded(CVector(
        -sy * cr + cy * sp * sr,
        cy * cp,
        sy * sr + cy * sp * cr
    ));

    m.up = ToPadded(CVector(
        cp * sr,
        -sp,
        cp * cr
    ));

    // Position = origin
    m.pos = ToPadded(CVector(0.0f, 0.0f, 0.0f));

    return m;
}


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
