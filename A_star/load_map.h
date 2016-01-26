//
// Created by Olga Mikheeva on 25/01/16.
//

#include <fstream>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

std::vector<std::vector<bool> > loadObstacleMapFromTxt(std::string path_to_txt) {
    std::ifstream file(path_to_txt);
    std::string str;
    std::vector<std::vector<bool> > obstacle_map;
    while (std::getline(file, str))
    {
        std::vector<bool> row;
        std::vector<std::string> numbers;
        boost::split(numbers, str, boost::is_any_of(", "), boost::token_compress_on);

        for (auto l: numbers) {
            row.push_back((bool) atoi(l.c_str()));
        }

        obstacle_map.push_back(row);
    }
    return obstacle_map;
}