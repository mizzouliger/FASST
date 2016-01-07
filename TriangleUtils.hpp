//
// Created by Seth Wiesman on 12/23/15.
//

#ifndef THESIS_TRIANGLEUTILS_HPP
#define THESIS_TRIANGLEUTILS_HPP

#include <limits>
#include <vector>

namespace Thesis {
    namespace TriangleUtils {

        static constexpr double infinity = std::numeric_limits<double>::max();

        double maximize_minimum_triangle(std::vector<double> rootToNodeLengths,
                                           std::vector<double> rootToTargetLengths);

        double minimize_maximum_triangle(std::vector<double> rootToNodeLengths,
                                           std::vector<double> rootToTargetLengths);
    }
}

#endif //THESIS_TRIANGLEUTILS_HPP
