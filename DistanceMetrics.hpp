//
// Created by Seth Wiesman on 2/23/16.
//

#ifndef THESIS_DISTANCEMETRICS_HPP
#define THESIS_DISTANCEMETRICS_HPP

#include <vector>
#include <string>

namespace Thesis {
    namespace Metrics {
        double norm2(const std::vector<double> &v1, const std::vector<double> &v2);
        double editDistance(const std::string &s1, const std::string &s2);
        double hammingDistance(const int &x, const int &y);
    };
};

#endif //THESIS_DISTANCEMETRICS_HPP
