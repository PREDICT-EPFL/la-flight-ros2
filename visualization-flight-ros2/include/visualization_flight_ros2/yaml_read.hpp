#ifndef SRC_YAML_READ_HPP
#define SRC_YAML_READ_HPP

#include <iostream>
#include <cassert>
#include <cmath>

#include <yaml-cpp/yaml.h>

namespace yaml_read {

static bool get_as_double(const YAML::Node &node, const std::string &name, double &value_write)
{
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        switch (it->second.Type())
        {
        case YAML::NodeType::Scalar:
            if (key == name)
            {
                value_write = it->second.as<double>();
                return true;
            }
            break;
        case YAML::NodeType::Map:
            /** iterate over the map */
            for (auto iter = it->second.begin(); iter != it->second.end(); ++iter)
            {
                if ((iter->first).as<std::string>() == name)
                {
                    value_write = iter->second.as<double>();
                    return true;
                }
            }
            break;
        case YAML::NodeType::Null:
            std::cout << "real lox \n";
            break;
        default :
            std::cout << "lox \n";
            break;
        }
    }
    return false; // Success / modified
}

}

#endif //SRC_YAML_READ_HPP
