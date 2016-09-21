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

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	
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
	float tSquared = normalTime * normalTime;
	float tCubed = tSquared * normalTime;
	CurvePoint p0 = controlPoints[nextPoint - 1];
	CurvePoint p1 = controlPoints[nextPoint];
	newPosition = (2.0f * tCubed - 3.0f * tSquared + 1.0f) * p0.position + 
		(tCubed - 2.0f * tSquared + normalTime) * p0.tangent +
		(-2.0f * tCubed + 3.0f * tSquared) * p1.position + 
		(tCubed - tSquared) * p1.tangent;
	
	/*float normalTime, intervalTime;

	if (!findTimeInterval(nextPoint, time)) {
		return newPosition;
	}
	normalTime = time;
	intervalTime = findTimeInterval(nextPoint, normalTime);

	float h1 = 2 * normalTime * normalTime * normalTime - 3 * normalTime * normalTime + 1;
	float h2 = -2 * normalTime * normalTime * normalTime + 3 * normalTime * normalTime;
	float h3 = normalTime * normalTime * normalTime - 2 * normalTime * normalTime + normalTime;
	float h4 = normalTime * normalTime * normalTime - normalTime * normalTime;

	//find x
	newPosition.x = h1 * controlPoints[normalTime].position.x +
		h2 * controlPoints[intervalTime].position.x +
		h3 * controlPoints[normalTime].tangent.x +
		h4 * controlPoints[intervalTime].tangent.x;

	//find y
	newPosition.y = h1 * controlPoints[normalTime].position.y +
		h2 * controlPoints[intervalTime].position.y +
		h3 * controlPoints[normalTime].tangent.y +
		h4 * controlPoints[intervalTime].tangent.y;

	//find z
	newPosition.z = h1 * controlPoints[normalTime].position.z +
		h2 * controlPoints[intervalTime].position.z +
		h3 * controlPoints[normalTime].tangent.z +
		h4 * controlPoints[intervalTime].tangent.z;*/
	
	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	// Calculate position at t = time on Catmull-Rom curve
	float normalTime = (time - controlPoints[nextPoint - 1].time) / (controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time);
	if (nextPoint > 1 && nextPoint < controlPoints.size() - 2) {
		Point p0 = controlPoints[nextPoint - 2].position;
		Point p1 = controlPoints[nextPoint - 1].position;
		Point p2 = controlPoints[nextPoint].position;
		Point p3 = controlPoints[nextPoint + 1].position;
		newPosition = 0.5f * ((2.0f * p1) + 
			(-p0 + p2) * normalTime + 
			(2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * pow(normalTime, 2.0f) + 
			(-p0 + 3.0f * p1 - 3.0f * p2 + p3) * pow(normalTime, 3.0f));
	} else {
		// Use the Hermite Curve formula if right after the first control point or right before the last control point.
		newPosition = useHermiteCurve(nextPoint, time);
	}
	
	// Return result
	return newPosition;
}