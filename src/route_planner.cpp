#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // - use the distance to the end_node for the h value.
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    // - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();
    for(RouteModel::Node* n: current_node->neighbors)
    {
      n->g_value = current_node->g_value + n->distance(*current_node);

      // - Use CalculateHValue below to implement the h-Value calculation.
      n->h_value = this->CalculateHValue(n);

      n->parent = current_node;

      // - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
      n->visited = true;
      this->open_list.push_back(n);
    }
}

bool Compare(RouteModel::Node *n1, RouteModel::Node *n2){
    return (n1->g_value + n1->h_value) > (n2->g_value + n2->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    // - Sort the open_list according to the sum of the h value and g value.
    std::sort(this->open_list.begin(),this->open_list.end(), Compare);
    // - Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node *next = open_list.back();
    // - Remove that node from the open_list.
    open_list.pop_back();
    return next;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while(current_node != this->start_node)
    {
      // - For each node in the chain, add the distance from the node to its parent to the distance variable.
      this->distance += current_node->distance(*(current_node->parent));
      path_found.insert(path_found.begin(),*current_node);
	  current_node = current_node->parent;
    }
	path_found.insert(path_found.begin(),*current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    //start_node->g_value = 0;
    //start_node->h_value = this->CalculateHValue(start_node);
    start_node->visited = true;
    this->open_list.push_back(start_node);
    current_node = this->start_node;
	while(this->open_list.size()>0)
    {
      // - Use the NextNode() method to sort the open_list and return the next node.
      current_node = this->NextNode();
      if(current_node==this->end_node)
      {
        std::cout<<"Path Found \n";
	// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
        this->m_Model.path = this->ConstructFinalPath(current_node);
		return;
      }
      // - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list
      this->AddNeighbors(current_node);
    }
}
