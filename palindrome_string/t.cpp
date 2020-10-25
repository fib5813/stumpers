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
    int id;
    float distance_from_src;
    int prev_node;
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


void calculate_charging_time(std::vector<int> &path, std::map<std::string, int> &my_map,
                             std::array<row, size> &network,  std::array<std::array<float, size>, size>& graph,
                             std::vector<float> &trip_time){

    // iterate through map, find time required to charge at each station to reach next station
    int city1 = path.at(0);
    int city2 = path.at(1);
    float distance = graph.at(city1).at(city2);
    float travel_time = (distance / speed) / 3600.0F; // time in sec, converted to hrs
    // trip_time.push_back(travel_time);
    float charge_time;
    // std::cout << "city1 " << network.at(city1).name <<
    //            "  city2 " << network.at(city2).name <<
    //            "  distance " << distance <<
    //            "  travel time " << travel_time << std::endl;
    float prev_mileage = mileage;
    float remaining_miles = prev_mileage - distance;

    for(int i = 2; i < path.size(); i++){

        // caculate travel time for the next leg
        city1 = city2;
        city2 = path.at(i);
        distance = graph.at(city1).at(city2);
        // if(distance > mileage){std::cout << "error4 : " << network.at(city1).name << " " << network.at(city2).name << std::endl; return;}
        travel_time = (distance / speed)/3600.0F;
        // std::cout << "city1 " << network.at(city1).name <<
        //        "  city2 " << network.at(city2).name <<
        //        "  distance " << distance <<
        //        "  travel time " << travel_time <<
        //        "  remaining miles " << remaining_miles;

        if(remaining_miles > distance) std::cout << "why did you get here dummy!! " << std::endl;

        float dist_to_charge = distance - remaining_miles;
        prev_mileage = dist_to_charge + remaining_miles;
        remaining_miles = 0;
        float charge_rate = network.at(city1).rate;
        charge_time = (dist_to_charge / (charge_rate*1000.0F));  // converted rate to m/s, time to hrs.
        // append values to trip_time
        trip_time.push_back(charge_time);
            std::cout << "  charge rate " << charge_rate <<
            "  distance to charge " << dist_to_charge <<
            "  charge time " << charge_time << std::endl;

    }


}

void update_connected_nodes_dist(int curr_node, std::map<int, std::pair<float, int>> &dist, float dist_from_src, std::array<std::array<float, size>, size>& graph){
    // update the distance of all nodes connected to current node.
    // connections given in graph,
    // distances to be updated in dist

    std::array<float, size> list = graph.at(curr_node);
    // go through components connected to curr_node, if weight!=max_float, update weight in "dist" map
    for(int i = 0; i < size; i++){
        if(list.at(i) == 0) continue;
        auto it = dist.find(i);
        if(it == dist.end()){std::cout << "error3" << std::endl; return;}

        float new_dist = dist_from_src + list.at(i);
        if(new_dist < it->second.first) {
            it->second.first = new_dist;
            it->second.second = curr_node;
        }
    }
}


int find_new_node(std::map<int, std::pair<float, int>> &dist, std::array<bool, size> &visited){
    int out = -1;
    float min_dist = std::numeric_limits<float>::max();
    for(auto it = dist.begin(); it != dist.end(); it++){
        if(it->second.first <= min_dist && !visited.at(it->first)){
            min_dist = it->second.first;
            out = it->first;
        }
    }
    return out;
}


void initialize_status(std::array<bool, size> &visited, std::map<int, std::pair<float, int>> &dist){
    for(int i = 0; i < size; i++){
        visited.at(i) = false;
        dist.insert({i, std::make_pair(std::numeric_limits<float>::max(),-2)});
    }
    // std::cout << "initialized status..." << std::endl;
}

void find_shortest_path(int initial, int final_node, std::vector<int> &path, std::array<std::array<float, size>, size>& graph){

    std::array<bool, size> visited;
    std::map<int, std::pair<float, int>> dist; // id, min_distance, previous node

    initialize_status(visited, dist);

    // start finding the shortest path
    // visit every node
    float dist_from_src = 0;
    auto it = dist.find(initial);

    if(it!= dist.end()){ it->second = std::make_pair(0, -1);}
    else {std::cout<< "error 1" << std::endl; return;}
    for(int count = 0; count < size; count++){
        // begin at the start node, set the distance from node to itself as 0
        int curr_node = it->first;
        dist_from_src = it->second.first;
        visited.at(curr_node) = true;

        // update distance of connected nodes
        update_connected_nodes_dist(curr_node, dist, dist_from_src, graph);
        int new_node = find_new_node(dist, visited);
        if(new_node == -1) {
            // std::cout << "error2" << std::endl;
            // return;
        }

        it = dist.find(new_node);
    }
    std::stack<int> rev_path;
    rev_path.push(final_node);
    int k = final_node;
    auto it1 = dist.find(k);
    int temp = it1->second.second;
    if(temp == -2){std::cout << "No path found" << std::endl; return;}

    while(k!=initial) {
        auto it = dist.find(k);
        k = it->second.second;
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

    // std::vector<<name, id_in_array>> = find shortest path between the 2 given nodes.
    int initial = my_map.find(initial_charger)->second;
    int final_node = my_map.find(goal_charger)->second;
    if(initial == my_map.end() || final_node == my_map.end()){std::cout << "Cities not found, no path found";return 0;}
    
    std::vector<int> path{};
    find_shortest_path(initial, final_node, path, graph);
    if(path.size() ==0){std::cout << "no path found";return 0;}
    // print_vector(path);

    // calculate time = ()
    std::vector<float> trip_time{};
    calculate_charging_time(path, my_map, network, graph, trip_time);
    // print_vector(trip_time);

    print_final_output(path, trip_time);

    return 0;
}
