//
// Created by Seth Wiesman on 12/23/15.
//

#include <cmath>
#include "TriangleUtils.hpp"

namespace Thesis {
    namespace TriangleUtils {
        inline bool triangle_undefined(double side1, double side2) {
            return side1 == infinity || side2 == infinity;
        }

        double maximize_minimum_triangle(std::vector<double> rootToNodeLengths,
                                           std::vector<double> rootToTargetLengths) {
            double max = 0;
            for (auto i = 0; i < rootToNodeLengths.size(); i++) {
                if (triangle_undefined(rootToNodeLengths[i], rootToTargetLengths[i])) {
                    continue;
                }
                const auto nodeToTargetLength = std::fabs(rootToNodeLengths[i] - rootToTargetLengths[i]);

                if (max < nodeToTargetLength) {
                    max = nodeToTargetLength;
                }
            }

            return max;
        }

        double minimize_maximum_triangle(std::vector<double> rootToNodeLengths,
                                           std::vector<double> rootToTargetLengths) {
            double min = infinity;
            for (auto i = 0; i < rootToNodeLengths.size(); i++) {
                if (triangle_undefined(rootToNodeLengths[i], rootToTargetLengths[i])) {
                    continue;
                }

                const auto nodeToTargetLength = rootToNodeLengths[i] + rootToTargetLengths[i];

                if (nodeToTargetLength < min) {
                    min = nodeToTargetLength;
                }

            }

            return min;
        }
    }
}