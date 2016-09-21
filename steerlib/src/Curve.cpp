//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

bool curvePointCmp(CurvePoint cp1, CurvePoint cp2) {
	return cp1.time < cp2.time;
}

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI
	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	for (int i = 1; i < controlPoints.size(); i++) {
		Util::DrawLib::drawLineAlpha(controlPoints[i - 1].position, controlPoints[i].position, Color(0.0f, 0.0f, 1.0f), 1.0f);
	}
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	sort(controlPoints.begin(), controlPoints.end(), curvePointCmp);
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	return controlPoints.size() >= 2;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
// This function assumes that controlPoints is in ascending order.
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	for (int i = 1; i < controlPoints.size(); i++) {
		if (time < controlPoints[i].time) {
			nextPoint = i;
			return true;
		}
	}
	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	// Calculate position at t = time on Hermite curve
	float normalTime = (time - controlPoints[nextPoint - 1].time) / (controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time);
	CurvePoint p0 = controlPoints[nextPoint - 1];
	CurvePoint p1 = controlPoints[nextPoint];
	float tSquared = normalTime * normalTime;
	float tCubed = tSquared * normalTime;
	newPosition = (2.0f * tCubed - 3.0f * tSquared + 1.0f) * p0.position + 
		(tCubed - 2.0f * tSquared + normalTime) * p0.tangent +
		(-2.0f * tCubed + 3.0f * tSquared) * p1.position + 
		(tCubed - tSquared) * p1.tangent;
	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	// Calculate position at t = time on Catmull-Rom curve
	float normalTime = (time - controlPoints[nextPoint - 1].time) / (controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time);
	Point p0 = controlPoints[nextPoint - 1].position;
	Point p1 = controlPoints[nextPoint].position;
	Vector m0, m1;
	if (nextPoint > 1) {
		m0 = 0.5f * (p1 - controlPoints[nextPoint - 2].position);
	} else {
		m0 = p1 - p0;
	}
	if (nextPoint < controlPoints.size() - 2) {
		m1 = 0.5f * (controlPoints[nextPoint + 1].position - p0);
	} else {
		m1 = p1 - p0;
	}
	float tSquared = normalTime * normalTime;
	float tCubed = tSquared * normalTime;
	newPosition = (2.0f * tCubed - 3.0f * tSquared + 1.0f) * p0 + 
		(tCubed - 2.0f * tSquared + normalTime) * m0 +
		(-2.0f * tCubed + 3.0f * tSquared) * p1 + 
		(tCubed - tSquared) * m1;
	/*if (nextPoint > 1 && nextPoint < controlPoints.size() - 2) {
		Point p0 = controlPoints[nextPoint - 2].position;
		Point p1 = controlPoints[nextPoint - 1].position;
		Point p2 = controlPoints[nextPoint].position;
		Point p3 = controlPoints[nextPoint + 1].position;
		float tSquared = normalTime * normalTime;
		float tCubed = tSquared * normalTime;
		newPosition.x = 0.5f * ((2.0f * p1.x) + 
			((p2.x - p0.x) * normalTime) + 
			((2.0f * p0.x + 4.0f * p2.x - 5.0f * p1.x - p3.x) * tSquared) + 
			((3.0f * p1.x + p3.x - 3.0f * p2.x - p0.x) * tCubed));
		newPosition.y = 0.5f * ((2.0f * p1.y) + 
			((p2.y - p0.y) * normalTime) + 
			((2.0f * p0.y + 4.0f * p2.y - 5.0f * p1.y - p3.y) * tSquared) + 
			((3.0f * p1.y + p3.y - 3.0f * p2.y - p0.y) * tCubed));
		newPosition.z = 0.5f * ((2.0f * p1.z) + 
			((p2.z - p0.z) * normalTime) + 
			((2.0f * p0.z + 4.0f * p2.z - 5.0f * p1.z - p3.z) * tSquared) + 
			((3.0f * p1.z + p3.z - 3.0f * p2.z - p0.z) * tCubed));
	} else {
		// Use the Hermite Curve formula if right after the first control point or right before the last control point.
		newPosition = useHermiteCurve(nextPoint, time);
	}*/
	
	// Return result
	return newPosition;
}