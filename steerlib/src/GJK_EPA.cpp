#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

Util::Vector tripleProduct(Util::Vector A, Util::Vector B, Util::Vector C) {
	return B * Util::dot(A, C) - C * Util::dot(A, B);
}

Util::Vector getSupport(const std::vector<Util::Vector>& _shape, Util::Vector d) {
	float highest = -FLT_MAX;
	Util::Vector support;
	for (int i = 0; i < _shape.size(); ++i) {
		Util::Vector v = _shape[i];
		float max_dot = Util::dot(v,d);

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

		Util::Vector ab_norm = tripleProduct(ac, ab, ab);
		Util::Vector ac_norm = tripleProduct(ab, ac, ac);
		if (Util::dot(ab_norm, a0) > 0) {
			s.erase(s.begin());
			d = ab_norm;
		} else {
			if (Util::dot(ac_norm, a0) > 0) {
				s.erase(s.begin() + 1);
				d = ac_norm;
			} else
				return true;
		}
	} else {
		Util::Vector b = s[1];
		Util::Vector ab = b - a;
		Util::Vector ab_norm = tripleProduct(ab, a0, ab);
		d = ab_norm;
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
		if (Util::dot(A, D) < 0)
			return false;
		s.push_back(A);
		if (nearestSimplex(s, D))
			return true;
	}

}

Util::Vector findClosestEdge(std::vector<Util::Vector>& s, float& d, int& index) {
	Util::Vector closest_edge_normal;
	d = FLT_MAX;
	for (int i = 0; i < s.size(); i++) {
		int j = 0;
		if (i + 1 == s.size())
			j = 0;
		else
			j = i + 1;

		Util::Vector a = s[i];
		Util::Vector b = s[j];

		Util::Vector e = b - a;
		Util::Vector oa = a;
		Util::Vector n = tripleProduct(e, oa, e);

		n = Util::normalize(n);

		float dist = Util::dot(n, a);

		if (dist < d) {
			d = dist;
			closest_edge_normal = n;
			index = j;
		}
	}

	return closest_edge_normal;
}

void EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector> s) {
	while (true) {
		float edge_distance;
		int index;
		Util::Vector edge_normal = findClosestEdge(s, edge_distance, index);

		Util::Vector p = getSupport(_shapeA, edge_normal) - getSupport(_shapeB, -1 * edge_normal);

		float d = Util::dot(p, edge_normal);
		if (d - edge_distance < 0.0001) {
			return_penetration_vector = edge_normal;
			return_penetration_depth = d;
		}
		else
			s.insert(s.begin() + index, p);
	}
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	bool is_colliding = GJK(_shapeA, _shapeB, simplex);

	if (is_colliding)
	{
		EPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, simplex);
	}
	return is_colliding;
	//return false; // There is no collision
}