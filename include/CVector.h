class CVector
{
public:
	float x, y, z;

    CVector() : x(0), y(0), z(0) {}
    CVector(float x, float y, float z) : x(x), y(y), z(z) {}

	float Heading() { return atan2(-x, y); }
	float MagnitudeSqr() const  { return x * x + y * y + z * z; }
	float MagnitudeSqr2D() const { return x * x + y * y; }
	float Magnitude() const { return sqrt(MagnitudeSqr()); }
	float Magnitude2D() const { return sqrt(MagnitudeSqr2D()); }

    // Operators
    CVector operator+(const CVector& rhs) const
    {
        return CVector(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    CVector operator-(const CVector& rhs) const
    {
        return CVector(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    // Componentwise multiplication
    CVector operator*(const CVector& rhs) const
    {
        return CVector(x * rhs.x, y * rhs.y, z * rhs.z);
    }

    // Scalar multiplication
    CVector operator*(float scalar) const
    {
        return CVector(x * scalar, y * scalar, z * scalar);
    }

    // Compound assignments for convenience
    CVector& operator+=(const CVector& rhs)
    {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }

    CVector& operator-=(const CVector& rhs)
    {
        x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }

    CVector& operator*=(const CVector& rhs)
    {
        x *= rhs.x; y *= rhs.y; z *= rhs.z;
        return *this;
    }

    CVector& operator*=(float scalar)
    {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    // Normalize in place
    void Normalize()
    {
        float mag = Magnitude();
        if (mag > 1e-6f) // avoid divide by zero
        {
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }

    // Return a normalized copy
    CVector Normalized() const
    {
        float mag = Magnitude();
        if (mag > 1e-6f)
            return CVector(x / mag, y / mag, z / mag);
        return CVector(0, 0, 0);
    }

    // Dot product with another vector
    float Dot(const CVector& rhs) const
    {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    // Cross product with another vector
    CVector Cross(const CVector& rhs)
    {
        return CVector(
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        );
    }
};

inline float Dot(const CVector& a, const CVector& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline CVector Cross(const CVector& a, const CVector& b)
{
    return CVector(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

inline CVector Normalize(const CVector& v)
{
    float mag = v.Magnitude();
    if (mag > 1e-6f)
        return CVector(v.x / mag, v.y / mag, v.z / mag);
    return CVector(0, 0, 0);
}

class CVector_pad : public CVector
{
public:
	unsigned int flags; // ???

    CVector_pad() : CVector(), flags(0) {}
    CVector_pad(float x, float y, float z, unsigned int flags = 0)
        : CVector(x, y, z), flags(flags) {
    }

    // Construct from CVector
    explicit CVector_pad(const CVector& vec, unsigned int flags = 0)
        : CVector(vec), flags(flags) {
    }
};

//  Conversion functions 

// Convert CVector to CVector_pad
inline CVector_pad ToPadded(const CVector& vec, unsigned int flags = 0)
{
    return CVector_pad(vec, flags);
}

// Convert CVector_pad to CVector
inline CVector ToVector(const CVector_pad& vecPad)
{
    return CVector(vecPad.x, vecPad.y, vecPad.z);
}

inline CVector Lerp(const CVector& a, const CVector& b, float t)
{
    return CVector(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    );
}

inline float SignedAngle(const CVector& a, const CVector& b, const CVector& axis)
{
    // Normalize inputs
    CVector an = a.Normalized();
    CVector bn = b.Normalized();

    // Dot gives cosine of angle
    float dot = an.Dot(bn);
    dot = std::fmax(-1.0f, std::fmin(1.0f, dot)); // clamp for safety

    float angle = acosf(dot);

    // Cross gives direction
    CVector cross = an.Cross(bn);

    // If cross points opposite to axis, flip the sign
    if (cross.Dot(axis) < 0.0f)
        angle = -angle;

    return angle; // radians
}