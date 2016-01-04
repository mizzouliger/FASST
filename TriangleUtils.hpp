//
// Created by Seth Wiesman on 12/23/15.
//

#ifndef THESIS_TRIANGLEUTILS_HPP
#define THESIS_TRIANGLEUTILS_HPP

#include <limits>
#include <vector>

namespace Thesis {
    namespace TriangleUtils {

        class Triangle {
            double rootToTarget;
            double rootToNode;
            double nodeToTarget;

        public:
            Triangle(double a, double b, double c) : rootToTarget(a), rootToNode(b), nodeToTarget(c) {}

            double getRootToTarget() const { return rootToTarget; }
            double getRootToNode()   const { return rootToNode;   }
            double getNodeToTarget() const { return nodeToTarget; }
        };

        static constexpr double infinity = std::numeric_limits<double>::max();

        Triangle maximize_minimum_triangle(std::vector<double> rootToNodeLengths,
                                           std::vector<double> rootToTargetLengths);

        Triangle minimize_maximum_triangle(std::vector<double> rootToNodeLengths,
                                           std::vector<double> rootToTargetLengths);
    }
}

#endif //THESIS_TRIANGLEUTILS_HPP
