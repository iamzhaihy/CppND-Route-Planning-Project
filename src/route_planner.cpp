#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find closet nodes to start and end position
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate current_node.neighbors
    current_node->FindNeighbors();
    
    for (RouteModel::Node *neighbor: current_node->neighbors) {
        // Add visited neighbor nodes
        if (neighbor->visited == false) {
            neighbor->visited = true;
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + 
                                neighbor->distance(*current_node);
            open_list.push_back(neighbor);
        }
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    // Compare function for nodes
    auto compare = [](RouteModel::Node * a, RouteModel::Node * b) {
        return (a->g_value + a->h_value) > (b->g_value + b->h_value);
    };

    // Sort in descending order
    std::sort(open_list.begin(), open_list.end(), compare);

    // Take the node with smallest g + h value
    RouteModel::Node *result = open_list.back();
    open_list.pop_back();

    return result;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Trace back from end_node
    while (current_node != nullptr) {
        if (current_node->parent)
            distance += current_node->distance(*(current_node->parent));
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale(); 

    // Reverse the vector to get correct order
    std::reverse(path_found.begin(), path_found.end());  

    return path_found;
}


void RoutePlanner::AStarSearch() {
    start_node->visited = true;
    start_node->parent = nullptr;
    start_node->g_value = 0.0f;
    start_node->h_value = CalculateHValue(start_node);
    open_list.push_back(start_node);

    RouteModel::Node *current_node = nullptr;

    while (!open_list.empty()) {
        // Find closest node
        current_node = NextNode();

        // Construct the path if reached the goal
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // Expand neighbors otherwise
        AddNeighbors(current_node);
    }
}