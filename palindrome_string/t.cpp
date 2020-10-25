#include "network.h"
#include <map>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>
#include <stack>

constexpr int size = 303;
constexpr float R = 6356752.0F; // convert to meters
constexpr float speed = 105*1000.0F/3600.0F; // convert to m/s
constexpr float mileage = 320000.0F; // convert to meters
constexpr float pi_over_180 = (22.0F/7.0F)/180.0F;

struct node{
    // int id;
    float distance_from_src;
    int prev_node;

    node(){distance_from_src = std::numeric_limits<float>::max(); prev_node = -2;}
    node(float dist_val, int prev_node_val){distance_from_src = dist_val; prev_node = prev_node_val;}
};

void print_graph(std::array<std::array<int, size> size> &graph){
    for(int i = 0; i < size; i++){
        for(int j = 0; j < size; j++) std::cout << graph.at(i).at(j) << " " ;
        std::cout << std::endl;
    }
    std::cout << std::endl << std::endl;
}

template <class T>
void print_vector(std::vector<T> &path){
    for(auto &i: path) std::cout << i << " " ;
    std::cout << std::endl;
}

void print_dist(std::map<int, std::pair<float, int>> &dist){
    for(auto &i : dist){
        std::cout << i.first << " " << i.second.first << " " << i.second.second <<  std::endl;
    }
}

void print_final_output(std::vector<int>&path, std::vector<float>&trip_time){
    std::cout << network.at(path.at(0)).name << ", " ;
    for(int i = 0; i< trip_time.size(); i++){
        std::cout <<network.at(path.at(i+1)).name << ", " <<  trip_time.at(i) << ", " ;
    }
    std::cout << network.at(path.back()).name << std::endl;
}


void calculate_charging_time(const std::vector<int> &path, 
                             const std::array<std::array<float, size>, size> &graph,
                             std::vector<float> &trip_time){

    // iterate through map, find time required to charge at each station to reach next station
    int curr_city = path.at(0);
    int next_city = path.at(1);
    float distance = graph.at(curr_city).at(next_city);
    // float travel_time = (distance / speed) / 3600.0F; // time in sec, converted to hrs
    // trip_time.push_back(travel_time);
    float charge_time;
    // std::cout << "curr_city " << network.at(curr_city).name <<
    //            "  next_city " << network.at(next_city).name <<
    //            "  distance " << distance <<
    //            "  travel time " << travel_time << std::endl;
    float prev_mileage = mileage;
    float remaining_miles = prev_mileage - distance;

    for(int i = 2; i < path.size(); i++){

        // calculate travel time for the next leg
        curr_city = next_city;
        next_city = path.at(i);
        distance = graph.at(curr_city).at(next_city);
        // if(distance > mileage){std::cout << "error4 : " << network.at(curr_city).name << " " << network.at(next_city).name << std::endl; return;}
        // travel_time = (distance / speed)/3600.0F;
        // std::cout << "curr_city " << network.at(curr_city).name <<
        //        "  next_city " << network.at(next_city).name <<
        //        "  distance " << distance <<
        //        "  travel time " << travel_time <<
        //        "  remaining miles " << remaining_miles;

        if(remaining_miles > distance) std::cout << "why did you get here dummy!! " << std::endl;

        float dist_to_charge = distance - remaining_miles;
        prev_mileage = dist_to_charge + remaining_miles;
        remaining_miles = 0;  // as in this scheme we are only charging enough to get to the next charger
        float charge_rate = network.at(curr_city).rate;
        charge_time = (dist_to_charge / (charge_rate*1000.0F));  // converted rate to m/s, time to hrs.
        
        // append values to trip_time
        trip_time.push_back(charge_time);
            std::cout << "  charge rate " << charge_rate <<
            "  distance to charge " << dist_to_charge <<
            "  charge time " << charge_time << std::endl;

    }


}

void update_connected_nodes_dist(const int curr_node, 
                                 const float dist_from_src,
                                 std::map<int, node> &dist, 
                                 const std::array<std::array<float, size>, size>& graph){
    // update the distance of all nodes connected to current node.
    // connections given in graph,
    // distances to be updated in dist

    std::array<float, size> list = graph.at(curr_node);
    
    // go through components connected to curr_node, if weight!=max_float, update weight in "dist" map
    for(int i = 0; i < size; i++){
        if(list.at(i) == 0) continue;  // this is the invalid value we set when creating the graph. meaning this connection doesnt exist.
        auto it = dist.find(i);
        if(it == dist.end()){std::cout << "error3" << std::endl; return;}

        float new_dist = dist_from_src + list.at(i);
        if(new_dist < it->second.distance_from_src) {
            it->second.distance_from_src = new_dist;
            it->second.prev_node = curr_node;
        }
    }
}


int find_new_node(const std::map<int, std::pair<float, int>> &dist, const std::array<bool, size> &visited){
    int out = -1;
    float min_dist = std::numeric_limits<float>::max();
    for(auto it = dist.begin(); it != dist.end(); it++){
        // check the unvisited nodes with the least distance from src
        if(it->second.distance_from_src <= min_dist && !visited.at(it->first)){
            min_dist = it->second.distance_from_src;
            out = it->first;
        }
    }
    return out;
}


void initialize_status(std::array<bool, size> &visited, std::map<int, node> &dist){
    for(int i = 0; i < size; i++){
        visited.at(i) = false;
        dist.insert({i, node()});
    }
    // std::cout << "initialized status..." << std::endl;
}

void find_shortest_path(const int initial, const int final_node, std::vector<int> &path, const std::array<std::array<float, size>, size>& graph){

    std::array<bool, size> visited;
    std::map<int, node> dist; // <id, node(id, prev node, dist)>

    initialize_status(visited, dist);

    float dist_from_src = 0.0F;
    auto it = dist.find(initial);

    // make distance of first node from itself as 0, previous node as invalid.
    if(it!= dist.end()){ it->second = node(0.0F, -1);}
    else {std::cout<< "error 1" << std::endl; return;}
    
    // start finding the shortest path by visiting every node
    for(int count = 0; count < size; count++){
        
        // begin at the start node, set the distance from node to itself as 0
        int curr_node = it->first;
        dist_from_src = it->second.distance_from_src;
        visited.at(curr_node) = true;

        // update distance of connected nodes
        update_connected_nodes_dist(curr_node, dist_from_src, dist, graph);
        int new_node = find_new_node(dist, visited);
 
        it = dist.find(new_node);
    }
    
    // retrace the path and add the path to the output vector.
    std::stack<int> rev_path;
    rev_path.push(final_node);
    int k = final_node;
    int temp = dist.find(k)->second.prev_node;
    if(temp == -2){std::cout << "No path found" << std::endl; return;}

    while(k!=initial) {
        auto it = dist.find(k);
        k = it->second.prev_node;
        rev_path.push(k);
    }

    while(!rev_path.empty()) {
        path.push_back(rev_path.top());
        rev_path.pop();
    }

}


float calculate_great_circle_distance(const row &charger1, const row &charger2){
    float dist = 0.0F;
    // formula, code ref: https://www.movable-type.co.uk/scripts/latlong.html
    float dlon = charger2.lon * pi_over_180 - charger1.lon*pi_over_180;
    float dlat = charger2.lat*pi_over_180 - charger1.lat*pi_over_180;
    float a = std::sin(dlat/2.0F)*std::sin(dlat/2.0F) + std::cos(charger1.lat*pi_over_180) * cos(charger2.lat*pi_over_180) * std::sin(dlon/2)*std::sin(dlon/2);
    float c = 2.0F * std::atan2(std::sqrt(a),std::sqrt(1.0F-a));
    float d = R * c;
    return d;
}


void create_graph(std::array<std::array<float, size>, size> &graph){

    // TODO: optimize here to not calculate each value twice.
    for(int i = 0; i < graph.at(0).size(); i++){
        for(int j = 0; j < graph.at(0).size(); j++){
            float dist = calculate_great_circle_distance(network.at(i), network.at(j));
            if(dist < mileage) graph.at(i).at(j) = dist;  // connection established
            else graph.at(i).at(j) = 0; // no connection
            // std::cout << " " << graph.at(i).at(j) << " ";
        }
    }
}

void create_map(std::map<std::string, int> &my_map){
    for(int i = 0; i < network.size(); i++){
        my_map.insert({network.at(i).name, i});
    }
}


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;
        return -1;
    }

    // std::cout << "Starting calculations..." << std::endl;
    std::string initial_charger = argv[1];
    std::string goal_charger = argv[2];

    // create map to facilitate looking up chargers based on names.
    std::map<std::string, int> my_map ;
    create_map(my_map);

    // preprocess the network to create a weighted graph
    std::array<std::array<float, size>, size> graph{{0}};
    create_graph(graph);
    // std::cout << "Created an adjacency matrix..." << std::endl;

    int initial = my_map.find(initial_charger)->second;
    int final_node = my_map.find(goal_charger)->second;
    if(initial == my_map.end() || final_node == my_map.end()){std::cout << "Cities not found, no path found";return 0;}
    
    // find shortest path between the initial and final nodes, return path as vector of node ids
    std::vector<int> path{};
    find_shortest_path(initial, final_node, path, graph);
    
    if(path.size() ==0){std::cout << "no path found";return 0;}

    // calculate charging time
    std::vector<float> trip_time{};
    calculate_charging_time(path, graph, trip_time);

    print_final_output(path, trip_time);

    return 0;
}
