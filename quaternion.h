// 轻量级旋转四元数库
// Lightweight rotation quaternion library

#pragma once

#include "vector.h"

class Quaternion : public Printable {
public:
	float w, x, y, z;

	Quaternion(): w(1), x(0), y(0), z(0) {};

	Quaternion(float w, float x, float y, float z): w(w), x(x), y(y), z(z) {};

	static Quaternion fromAxisAngle(const Vector& axis, float angle) {
		float halfAngle = angle * 0.5;
		float sin2 = sin(halfAngle);
		float cos2 = cos(halfAngle);
		float sinNorm = sin2 / axis.norm();
		return Quaternion(cos2, axis.x * sinNorm, axis.y * sinNorm, axis.z * sinNorm);
	}

	static Quaternion fromRotationVector(const Vector& rotation) {
		if (rotation.zero()) {
			return Quaternion();
		}
		return Quaternion::fromAxisAngle(rotation, rotation.norm());
	}

	static Quaternion fromEuler(const Vector& euler) {
		float cx = cos(euler.x / 2);
		float cy = cos(euler.y / 2);
		float cz = cos(euler.z / 2);
		float sx = sin(euler.x / 2);
		float sy = sin(euler.y / 2);
		float sz = sin(euler.z / 2);

		return Quaternion(
			cx * cy * cz + sx * sy * sz,
			sx * cy * cz - cx * sy * sz,
			cx * sy * cz + sx * cy * sz,
			cx * cy * sz - sx * sy * cz);
	}

	static Quaternion fromBetweenVectors(const Vector& u, const Vector& v) {
		float dot = u.x * v.x + u.y * v.y + u.z * v.z;
		float w1 = u.y * v.z - u.z * v.y;
		float w2 = u.z * v.x - u.x * v.z;
		float w3 = u.x * v.y - u.y * v.x;

		Quaternion ret(
			dot + sqrt(dot * dot + w1 * w1 + w2 * w2 + w3 * w3),
			w1,
			w2,
			w3);
		ret.normalize();
		return ret;
	}

	bool finite() const {
		return isfinite(w) && isfinite(x) && isfinite(y) && isfinite(z);
	}

	bool valid() const {
		return finite();
	}

	bool invalid() const {
		return !valid();
	}

	void invalidate() {
		w = NAN;
		x = NAN;
		y = NAN;
		z = NAN;
	}


	float norm() const {
		return sqrt(w * w + x * x + y * y + z * z);
	}

	void normalize() {
		float n = norm();
		w /= n;
		x /= n;
		y /= n;
		z /= n;
	}

	Quaternion operator * (const Quaternion& b) const {
		return Quaternion(
			w * b.w - x * b.x - y * b.y - z * b.z,
			w * b.x + x * b.w + y * b.z - z * b.y,
			w * b.y - x * b.z + y * b.w + z * b.x,
			w * b.z + x * b.y - y * b.x + z * b.w);
	}

	Quaternion inversed() const {
		return Quaternion(w, -x, -y, -z);
	}

	static Quaternion rotate(const Quaternion& a, const Quaternion& b) {
		return b * a * b.inversed();
	}

	static Vector rotateVector(const Vector& v, const Quaternion& q) {
		Quaternion ret = q * Quaternion(0, v.x, v.y, v.z) * q.inversed();
		return Vector(ret.x, ret.y, ret.z);
	}

	Vector toEuler() const {
		Vector ret;
		ret.x = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
		ret.y = asin(constrain(2 * (w * y - z * x), -1, 1));
		ret.z = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
		return ret;
	}

	size_t printTo(Print& p) const {
		return p.print("(") + p.print(w) + p.print(", ") + p.print(x) + p.print(", ") + p.print(y) + p.print(", ") + p.print(z) + p.print(")");
	}
};
