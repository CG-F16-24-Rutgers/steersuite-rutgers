#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

float dotProduct(Util::Vector A, Util::Vector B) {
	return A.x * B.x + A.z * B.z;
}

Util::Vector crossProduct(Util::Vector A, Util::Vector B) {
	Util::Vector cross;
	cross.x = A.y * B.z - A.z * B.y;
	cross.y = A.z * B.x - A.x * B.z;
	cross.z = A.x * B.y - A.y * B.x;
	return cross;
}

Util::Vector tripleProduct(Util::Vector A, Util::Vector B, Util::Vector C) {
	return B * dotProduct(A, C) - C * dotProduct(A, B);
}

Util::Vector getSupport(const std::vector<Util::Vector>& _shape, Util::Vector d) {
	float highest = -FLT_MAX;
	Util::Vector support;
	for (int i = 0; i < _shape.size(); ++i) {
		Util::Vector v = _shape[i];
		float max_dot = dotProduct(v,d);

		if (max_dot > highest) {
			highest = max_dot;
			support = v;
		}
	}
	return support;
}

//check to see if simplex contains the origin
bool nearestSimplex(std::vector<Util::Vector>& s, Util::Vector& d) {
	Util::Vector a = s.back();
	Util::Vector a0 = -1 * a;
	if (s.size() == 3) {
		Util::Vector b = s[1];
		Util::Vector c = s[0];

		Util::Vector ab = b - a;
		Util::Vector ac = c - a;

		Util::Vector abPerp = tripleProduct(ac, ab, ab);
		Util::Vector acPerp = tripleProduct(ab, ac, ac);
		if (dotProduct(abPerp, a0) > 0) {
			s.erase(s.begin());
			d = abPerp;
		} else {
			if (dotProduct(acPerp, a0) > 0) {
				s.erase(s.begin() + 1);
				d = acPerp;
			} else
				return true;
		}
	} else {
		Util::Vector b = s[1];
		Util::Vector ab = b - a;
		Util::Vector abPerp = tripleProduct(ab, a0, ab);
		d = abPerp;
	}
	return false;
}

bool GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& s) {
	Util::Vector initial_axis(1, 0, -1);
	Util::Vector A = getSupport(_shapeA, initial_axis) - getSupport(_shapeB, -1 * initial_axis);
	s.push_back(A);
	Util::Vector D = -1 * A;
	while (true) {
		A = getSupport(_shapeA, D) - getSupport(_shapeB, -1 * D);
		if (dotProduct(A, D) < 0) {
			return false;
		}
		s.push_back(A);
		if (nearestSimplex(s, D))
			return true;
	}

}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	bool is_colliding = GJK(_shapeA, _shapeB, simplex);

	if (is_colliding)
	{
		//EPA(_shapeA, _shapeB, simplex, return_penetration_depth, return_penetration_vector);
	}
	return is_colliding;
	//return false; // There is no collision
}