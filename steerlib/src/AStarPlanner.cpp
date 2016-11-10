//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//

// something was changed

#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include <float.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){
		std::cout << "\ncreated";
	}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	double AStarPlanner::hCostEst(Util::Point start, Util::Point goal) {
		int xDiff = start.x - goal.x;
		int yDiff = start.y - goal.y;
		return xDiff * xDiff + yDiff * yDiff;
	}
	
	std::vector<SteerLib::AStarPlannerNode> AStarPlanner::getNeighbors(SteerLib::AStarPlannerNode n) {
		std::vector<SteerLib::AStarPlannerNode> result;
		Util::Point p(n.point.x - 1, 0.0, n.point.y - 1);
		SteerLib::AStarPlannerNode node(p, DBL_MAX, DBL_MAX, &n);
		result.push_back(node);
		p.y = n.point.y;
		node.point = p;
		result.push_back(node);
		p.y = n.point.y + 1;
		node.point = p;
		result.push_back(node);
		p.x = n.point.x;
		p.y = n.point.y - 1;
		node.point = p;
		result.push_back(node);
		p.y = n.point.y + 1;
		node.point = p;
		result.push_back(node);
		p.x = n.point.x + 1;
		p.y = n.point.y - 1;
		node.point = p;
		result.push_back(node);
		p.y = n.point.y;
		node.point = p;
		result.push_back(node);
		p.y = n.point.y + 1;
		node.point = p;
		result.push_back(node);
		return result;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		std::vector<SteerLib::AStarPlannerNode> closedset;
		std::vector<SteerLib::AStarPlannerNode> openset;
		openset.push_back(SteerLib::AStarPlannerNode(start, DBL_MAX, DBL_MAX, NULL));
		std::vector<SteerLib::AStarPlannerNode> came_from;
		
		while (openset.size() > 0) {
			double lowestF = openset[0].f;
			int lowestIndex = 0;
			for (int i = 1; i < openset.size(); i++) {
				if (openset[i].f < lowestF) {
					lowestF = openset[i].f;
				}
			}
			AStarPlannerNode current = openset[lowestIndex];
			if (current.point == goal) {
				for (int i = 0; i < came_from.size(); i++) {
					agent_path.push_back(came_from[i].point);
				}
				return true;
			}
			
			openset.erase(openset.begin() + lowestIndex);
			closedset.push_back(current);
			std::vector<SteerLib::AStarPlannerNode> neighbors = getNeighbors(current);
			for (int i = 0; i < neighbors.size(); i++) {
				if (std::find(closedset.begin(), closedset.end(), neighbors[i]) != closedset.end()) {
					double tempG = current.g + distanceBetween(current.point, neighbors[i].point);
					if (tempG < neighbors[i].g) {
						SteerLib::AStarPlannerNode node(neighbors[i].point, tempG, tempG + hCostEst(neighbors[i].point, goal), &current);
						came_from.push_back(node);
						if (std::find(openset.begin(), openset.end(), neighbors[i]) == openset.end()) {
							openset.push_back(neighbors[i]);
						}
					}
				}
			}
		}

		return false;
	}
}