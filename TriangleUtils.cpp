//
// Created by Seth Wiesman on 12/23/15.
//

#include <cmath>
#include "TriangleUtils.hpp"

namespace Thesis {
    namespace TriangleUtils {
        bool triangle_undefined(double side1, double side2) {
            return side1 == infinity || side2 == infinity;
        }

        double maximize_minimum_triangle_length(std::vector<double> side1, std::vector<double> side2) {
            double max = 0;
            for (auto i = 0; i < side1.size(); i++) {
                if (!triangle_undefined(side1[i], side2[i])) {
                        const auto dist = std::fabs(side1[i] - side2[i]);

                    if (max < dist) {
                        max = dist;
                    }
                }
            }

            return max;
        }

        double minimize_maximum_triangle_length(std::vector<double> side1, std::vector<double> side2) {
            double min = infinity;
            for (auto i = 0; i < side1.size(); i++) {
                if (!triangle_undefined(side1[i], side2[i])) {
                    const auto dist = side1[i] + side2[i];

                    if (dist < min) {
                        min = dist;
                    }
                }
            }

            return min;
        }
    }
}