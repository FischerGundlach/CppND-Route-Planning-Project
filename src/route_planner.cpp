#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    start_node->g_value = 0.0;
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (const auto& neighbor : current_node->neighbors){
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->parent = current_node;
        this->open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const auto& a, const auto& b){
        return a->totalCost() < b->totalCost();
    });

    auto next_node = open_list[0];
    open_list.erase(open_list.begin());

    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = current_node->g_value;

    std::vector<RouteModel::Node> path_found;
    path_found.push_back(*current_node);
    auto parent_node = current_node->parent;
    while(parent_node){
        current_node = parent_node;
        path_found.push_back(*current_node);
        parent_node = parent_node->parent;
    }
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale();
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    open_list.push_back(start_node);
    start_node->visited = true;

    while (!open_list.empty()){
        current_node = NextNode();

        if (current_node == end_node){
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        else
            AddNeighbors(current_node);
    }
}