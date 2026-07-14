// 轻量级矢量库
// Lightweight vector library

#pragma once

class Vector : public Printable {
public:
	float x, y, z;

	Vector(): x(0), y(0), z(0) {};

	Vector(float x, float y, float z): x(x), y(y), z(z) {};

	bool zero() const {
		return x == 0 && y == 0 && z == 0;
	}

	bool finite() const {
		return isfinite(x) && isfinite(y) && isfinite(z);
	}

	bool valid() const {
		return finite();
	}

	bool invalid() const {
		return !valid();
	}

	void invalidate() {
		x = NAN;
		y = NAN;
		z = NAN;
	}


	float norm() const {
		return sqrt(x * x + y * y + z * z);
	}

	void normalize() {
		float n = norm();
		x /= n;
		y /= n;
		z /= n;
	}

	Vector operator + (const float b) const {
		return Vector(x + b, y + b, z + b);
	}

	Vector operator * (const float b) const {
		return Vector(x * b, y * b, z * b);
	}

	Vector operator / (const float b) const {
		return Vector(x / b, y / b, z / b);
	}

	Vector operator + (const Vector& b) const {
		return Vector(x + b.x, y + b.y, z + b.z);
	}

	Vector operator - (const Vector& b) const {
		return Vector(x - b.x, y - b.y, z - b.z);
	}

	Vector& operator += (const Vector& b) {
		return *this = *this + b;
	}

	Vector& operator -= (const Vector& b) {
		return *this = *this - b;
	}

	// Element-wise multiplication
	Vector operator * (const Vector& b) const {
		return Vector(x * b.x, y * b.y, z * b.z);
	}

	// Element-wise division
	Vector operator / (const Vector& b) const {
		return Vector(x / b.x, y / b.y, z / b.z);
	}

	bool operator == (const Vector& b) const {
		return x == b.x && y == b.y && z == b.z;
	}

	bool operator != (const Vector& b) const {
		return !(*this == b);
	}

	static float dot(const Vector& a, const Vector& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	static Vector cross(const Vector& a, const Vector& b) {
		return Vector(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		);
	}

	static Vector rotationVectorBetween(const Vector& a, const Vector& b) {
		Vector axis = cross(a, b);
		float angle = acos(constrain(dot(a, b) / (a.norm() * b.norm()), -1, 1));
		return axis.normalize() * angle;
	}

	size_t printTo(Print& p) const {
		return p.print("(") + p.print(x) + p.print(", ") + p.print(y) + p.print(", ") + p.print(z) + p.print(")");
	}

	float& operator [] (int i) {
		return ((float*)this)[i];
	}
};
