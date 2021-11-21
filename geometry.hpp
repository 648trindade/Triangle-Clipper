#pragma once

#include <cmath>

struct float3 {
    float x, y, z;
};

inline float3 operator+(const float3& a, const float3& b)
{
    return float3 { a.x + b.x, a.y + b.y, a.z + b.z };
}

inline float3& operator+=(float3& a, const float3& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline float3 operator-(const float3& a, const float3& b)
{
    return float3 { a.x - b.x, a.y - b.y, a.z - b.z };
}

inline float3& operator-=(float3& a, const float3& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline float3 operator-(const float3& a)
{
    return float3 { -a.x, -a.y, -a.z };
}

inline float3 operator*(const float3& a, const float3& b)
{
    return float3 { a.x * b.x, a.y * b.y, a.z * b.z };
}

inline float3 operator*(const float3& a, float b)
{
    return float3 { a.x * b, a.y * b, a.z * b };
}

inline float3 operator/(const float3& a, const float3& b)
{
    return float3 { a.x / b.x, a.y / b.y, a.z / b.z };
}

inline float3 operator/(const float3& a, float b)
{
    return float3 { a.x / b, a.y / b, a.z / b };
}

inline float dot(const float3& a, const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float dot(const float3& a)
{
    return dot(a, a);
}

inline float3 cross(const float3& a, const float3& b)
{
    return float3 {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

inline float norm(const float3& a)
{
    return std::sqrt(dot(a));
}

inline float3 normal(const float3& a, const float3& b, const float3& c)
{
    return cross(b - a, c - a);
}

inline float3 normalize(const float3& a)
{
    return a / norm(a);
}

inline float3 apply_trans_matrix(const float M[][3], const float3& a)
{
    return float3 {
        dot(float3{ M[0][0], M[0][1], M[0][2] }, a),
        dot(float3{ M[1][0], M[1][1], M[1][2] }, a),
        dot(float3{ M[2][0], M[2][1], M[2][2] }, a)
    };
}

struct Triangle
{
    float3 v0, v1, v2;

    float3 normal() const
    {
        return ::normal(this->v0, this->v1, this->v2);
    }

    float area() const
    {
        return norm(this->normal()) / 2.f;
    }

    void translate(const float3& direction)
    {
        this->v0 += direction;
        this->v1 += direction;
        this->v2 += direction;
    }

    void rotate(const float rotation_matrix[][3])
    {
        this->v0 = apply_trans_matrix(rotation_matrix, this->v0);
        this->v1 = apply_trans_matrix(rotation_matrix, this->v1);
        this->v2 = apply_trans_matrix(rotation_matrix, this->v2);
    }
};