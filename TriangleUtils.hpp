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

        double maximize_minimum_triangle_length(std::vector<double> side1, std::vector<double> side2);

        double minimize_maximum_triangle_length(std::vector<double> side1, std::vector<double> side2);
    }
}

#endif //THESIS_TRIANGLEUTILS_HPP
