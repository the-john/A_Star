#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage so we can use them in m_Model.FindClosestNode:
    start_x *= 0.01;  //we multiply the start_x value by 0.01 to convert it to a percentage
    start_y *= 0.01;  //we multiply the start_y value by 0.01 to convert it to a percentage
    end_x *= 0.01;    //we multiply the end_x value by 0.01 to convert it to a percentage
    end_y *= 0.01;    //we multiply the end_y value by 0.01 to convert it to a percentage

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

/*
// note; below code works, just trying to do the same without the '->' operator (which I find confusing at times)
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node -> distance(*end_node);
}
*/

float RoutePlanner::CalculateHValue(const RouteModel::Node *node) {
    return (*node).distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node -> FindNeighbors();
    for (RouteModel::Node *neighbor : current_node -> neighbors) {
        neighbor -> parent = current_node;
        neighbor -> h_value = CalculateHValue(neighbor);
        neighbor -> g_value = (current_node -> g_value) + (current_node -> distance(*neighbor));
        neighbor -> visited = true;

        open_list.push_back(neighbor);
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

/*
bool Compare(RouteModel::Node const *a, RouteModel::Node const *b) {
    int fa = (a -> h_value) + (a -> g_value);
    int fb = (b -> h_value) + (b -> g_value);
    return fa > fb;  // return true if first element's F value is greater than the second element's F value
                     // which means that fa should be placed before fb in the open_list via sort (below)
}
*/

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list from largets F value to smallest F value
    //std::sort(open_list.begin(), open_list.end(), Compare);  //https://www.geeksforgeeks.org/sort-c-stl/ go to end and read
    // /*
    std::sort(open_list.begin(), open_list.end(),
        [](auto const &a, auto const &b) {
            return (a -> h_value + a -> g_value) > (b -> h_value + b -> g_value);
        });
    // */
    // The open_list is now sorted by F value, the largest F value nodes are at the front of the open_list vector
    // The last node in open_list is the one we want (it has the smallest F value), we grab it to pass out and then remove it from the open_list
    //reverse(open_list.begin(), open_list.end());
    auto nextNode = open_list.back();
    //auto nextNode = open_list.front();
    open_list.pop_back();  // We can now remove this node from open_list
    //open_list.erase(open_list.begin());
    return nextNode;  // And we pass out the node that was in the open_list that closest to the end location (had the smallest F value)
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {  // Note; current_node is the final node in the path
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;  // We fill this vector up with all of the nodes from the final node back to the start node

    // TODO: Implement your solution here.
    while (current_node -> parent != nullptr) { 
        path_found.push_back(*current_node);
        RouteModel::Node parent_node = *(current_node -> parent);
        distance += current_node -> distance(parent_node);
        current_node = current_node -> parent; 
    }
    path_found.push_back(*current_node);  // Don't forget to add the Start node (won't be in 'while' loop)
    reverse(path_found.begin(), path_found.end());  // Reverse the order of the vector or it will not pass testing
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;  // clear/empty the current_node??? (if needed?)

    // TODO: Implement your solution here.
    current_node = start_node;  // Start with the start_node
    current_node -> visited = true;  // Set the starting node as visited
    open_list.push_back(current_node);  // Add start_node to the open_list
    // As long as there is something in the open_list, keep searching for nearest neighbor
    while (open_list.size() > 0) {
        current_node = NextNode();  // Get the next node (if this is the first time through, next node will be the start_node that we just put in open_list)
        // if this is the last node, make the path
        if (current_node -> distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // if not the last node, got get another one
        AddNeighbors(current_node);
    }

}