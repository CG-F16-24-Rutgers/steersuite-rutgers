//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//

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


#define WEIGHT 2
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

	bool AStarPlanner::canBeTraversed(int id) {
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP) {
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP) {
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
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
		int x_diff = start.x - goal.x;
		int y_diff = start.y - goal.y;
		int z_diff = start.z - goal.z;
		return abs(x_diff) + abs(y_diff) + abs(z_diff);
		/*int xDiff = start.x - goal.x;
		int yDiff = start.y - goal.y;
		return xDiff * xDiff + yDiff * yDiff;*/
	}
	
	std::vector<SteerLib::AStarPlannerNode> AStarPlanner::getNeighbors(SteerLib::AStarPlannerNode n, std::map<SteerLib::AStarPlannerNode, double>& g_score, std::map<SteerLib::AStarPlannerNode, double>& f_score) {
		std::vector<SteerLib::AStarPlannerNode> result; 
		Util::Point p;
		std::set<SpatialDatabaseItemPtr> neighborList;
		SteerLib::AStarPlannerNode node(p, DBL_MAX, DBL_MAX, &n);
		int minX, maxX, minZ, maxZ;
		minX = MAX(n.point.x - 1, gSpatialDatabase->getOriginX());
		maxX = MIN(n.point.x + 1, gSpatialDatabase->getOriginX() + gSpatialDatabase->getNumCellsX());
		minZ = MAX(n.point.z - 1, gSpatialDatabase->getOriginZ());
		maxZ = MIN(n.point.z + 1, gSpatialDatabase->getOriginZ() + gSpatialDatabase->getNumCellsZ());

		for (int x = minX; x <= maxX; x++) {
			for (int z = minZ; z <= maxZ; z++) {
				if ((x != (int) n.point.x || z != (int) n.point.z)) {
					if (canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(x, z))) {
						p.x = x;
						p.z = z;
						node.point = p;
						g_score.emplace(node, node.g);
						f_score.emplace(node, node.f);
						result.push_back(node);
					}
				}
			}
		}

		return result;
	}

	bool AStarPlanner::reconstructPath(std::vector<Util::Point>& agent_path, SteerLib::AStarPlannerNode current, std::map<SteerLib::AStarPlannerNode, AStarPlannerNode> came_from, Util::Point start) {
		agent_path.push_back(current.point);
		while (came_from.find(current) != came_from.end()) {
			current = came_from.at(current);
			agent_path.push_back(current.point);
		}
		return true;
		//std::reverse(agent_path.begin(), agent_path.end());*/		
	}

	bool AStarPlanner::computeAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface* gSpatialDatabase, bool append_to_path) {
		agent_path.clear();

		std::vector<SteerLib::AStarPlannerNode> closedset;
		std::vector<SteerLib::AStarPlannerNode> openset;
		std::map<SteerLib::AStarPlannerNode, SteerLib::AStarPlannerNode> came_from;
		std::map<SteerLib::AStarPlannerNode, double> g_score;
		std::map<SteerLib::AStarPlannerNode, double> f_score;

		SteerLib::AStarPlannerNode start_node(start, 0, hCostEst(start, goal), NULL);

		g_score.emplace(start_node, start_node.g);
		f_score.emplace(start_node, start_node.f);

		openset.push_back(start_node);

		double lowest_f_value = 0;
		int lowest_f_index = 0;
		double tentative_g_score = 0;

		while (!openset.empty()) {
			for (int i = 0; i < openset.size(); i++) {
				if (openset[i].f < lowest_f_value) {	
					lowest_f_index = i;
					lowest_f_value = openset[lowest_f_index].f;
					
				}
			}

			SteerLib::AStarPlannerNode current = openset[lowest_f_index];

			//std::cout << "(" << current.point.x << ", " << current.point.z << ")\n";

			if (current.point == goal) {
				return reconstructPath(agent_path, current, came_from, start);
			}

			openset.erase(openset.begin() + lowest_f_index);

			closedset.push_back(current);

			std::vector<SteerLib::AStarPlannerNode> neighbors = getNeighbors(current, g_score, f_score);

			for each(SteerLib::AStarPlannerNode neighbor in neighbors) {
				if (std::find(closedset.begin(), closedset.end(), neighbor) != closedset.end()) {
					continue;
				}

				tentative_g_score = g_score.at(current) + distanceBetween(current.point, neighbor.point);

				//if (tentative_g_score < g_score.at(neighbor)) {
				if(tentative_g_score < neighbor.g) {
					//came_from.at(neighbor) = current;
					came_from.emplace(neighbor, current);

					g_score.at(neighbor) = tentative_g_score;
					//g_score.emplace(neighbor, tentative_g_score);

					f_score.at(neighbor) = g_score.at(neighbor) + WEIGHT * hCostEst(neighbor.point, goal);
					//f_score.emplace(neighbor, g_score.at(neighbor) + hCostEst(neighbor.point, goal));

					if (std::find(openset.begin(), openset.end(), neighbor) == openset.end()) {
						openset.push_back(neighbor);
					}
				}
			}
		}

		return false;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		return computeAStar(agent_path, start, goal, gSpatialDatabase, append_to_path);
	}
}