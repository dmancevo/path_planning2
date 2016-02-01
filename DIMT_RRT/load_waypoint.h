//
// Created by Olga Mikheeva on 31/01/16.
//

//
// Created by Olga Mikheeva on 25/01/16.
//

#include <fstream>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

struct dynamicWaypoint {
    double acc_max;
    double vel_max;
    std::vector<std::pair<double,double>> points;
};

dynamicWaypoint loadDynamicWaipoint(std::string path_to_txt) {
    std::ifstream file(path_to_txt);
    dynamicWaypoint result;
    std::string str;
    std::getline(file, str);
    result.acc_max = std::stod(str);
    std::getline(file, str);
    result.vel_max = std::stod(str);

    while (std::getline(file, str))
    {
        std::pair<double,double> point;
        std::vector<std::string> numbers;
        boost::split(numbers, str, boost::is_any_of(", "), boost::token_compress_on);

        point.first = std::stod(numbers[0]);
        point.second = std::stod(numbers[1]);

        result.points.push_back(point);
    }
    return result;
}
