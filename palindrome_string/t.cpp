#include "network.h"
#include <map>
#include <algorithm>
#include <cmath>

constexpr int size = 13;
constexpr float R = 6356752.0F; // convert to meters
constexpr float speed = 105*1000.0F/3600.0F; // convert to m/s
constexpr float full_charge = 320000.0F; // convert to meters
constexpr float pi_over_180 = (22.0F/7.0F)/180.0F;


float calculate_great_circle_distance(row &charger1, row &charger2){
    float dist = 0.0F;
    // formula ref: https://www.movable-type.co.uk/scripts/gis-faq-5.1.html 
    // https://www.movable-type.co.uk/scripts/latlong.html
    float dlon = charger2.lon * pi_over_180 - charger1.lon*pi_over_180;
    float dlat = charger2.lat*pi_over_180 - charger1.lat*pi_over_180;
    float a = std::sin(dlat/2.0F)*std::sin(dlat/2.0F) + std::cos(charger1.lat*pi_over_180) * cos(charger2.lat*pi_over_180) * std::sin(dlon/2)*std::sin(dlon/2);
    float c = 2.0F * std::atan2(std::sqrt(a),std::sqrt(1.0F-a));
    float d = R * c;
    // if(charger1.name == "Albany_NY" && charger2.name == "Edison_NJ") std::cout << "albany and edison: " << d << std::endl;
    return d;
}


void preprocess_network(std::array<row, size> &network2, std::array<std::array<float, size>, size>  &graph){
    
    // TODO: optimize here to not calculate each value twice.
    for(int i = 0; i < graph.at(0).size(); i++){
        for(int j = 0; j < graph.at(0).size(); j++){
            // std::cout << " here"<< i << "_" << j;
            float dist = calculate_great_circle_distance(network2.at(i), network2.at(j));
            if(dist < full_charge) graph.at(i).at(j) = dist;
            else graph.at(i).at(j) = -1.0F; 
            // std::cout << " " << graph.at(i).at(j) << " ";
        }
    }


}

void create_map(std::map<std::string, int> &my_map){
    for(int i = 0; i < network.size(); i++){
        my_map.insert({network.at(i).name, i});
    }
}

std::array<row, size> network2 = 
{{
{"Albany_NY", 40.710356, -73.819109, 131.0},{"Edison_NJ", 40.544595, -74.334113, 159.0},
{"Dayton_OH", 39.858702, -84.277027, 133.0},{"West_Wendover_NV", 40.738399, -114.058998, 106.0},
{"Salina_KS", 38.877342, -97.618699, 177.0},{"Glen_Allen_VA", 37.66976, -77.461414, 128.0},
{"Beaver_UT", 38.249149, -112.652524, 109.0},{"Pleasant_Prairie_WI", 42.518715, -87.950428, 144.0},
{"Independence_MO", 39.040814, -94.369265, 107.0},{"Redondo_Beach_CA", 33.894227, -118.367407, 114.0},
{"Yuma_AZ", 32.726686, -114.619093, 116.0},{"Milford_CT", 41.245823, -73.009059, 130.0},
{"Liverpool_NY", 43.102424, -76.187446, 138.0}
}};

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;        
        return -1;
    }
    
    std::cout << "Starting calculations..." << std::endl;
    std::string initial_charger_name = argv[1];
    std::string goal_charger_name = argv[2];

    // my_map<name, id_in_array> = add the network nodes to a map (std::array)
    // std::map<std::string, int> my_map ;
    // create_map(my_map);

    // preprocess the network to create a weighted graph
    // std::sort(network2.begin(), network2.end());
    // for(auto &i : network2) std::cout <<  i.lat << " " << i.lon << " " << i.rate << std::endl;
    std::array<std::array<float, size>, size> graph{{0}};
    // std::cout << "here: " << graph[0][0] ;
    preprocess_network(network2, graph);
    for(int i = 0; i < size; i++){
        for(int j = 0; j < size; j++) std::cout << graph.at(i).at(j) << " " ;
        std::cout << std::endl;
    }
    std::cout << std::endl << std::endl;

    // std::vector<<name, id_in_array>> = find shortest path between the 2 given nodes.
    


    // calculate time = ()

    return 0;
}