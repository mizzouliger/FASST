//
// Created by Seth Wiesman on 2/23/16.
//

#include <math.h>
#include "DistanceMetrics.hpp"

double ::Thesis::Metrics::norm2(const std::vector<double> &v1, const std::vector<double> &v2) {
    double sum = 0.0;
    for (auto i = 0; i < v1.size(); i++) {
        sum += ((v1[i] - v2[i]) * (v1[i] - v2[i]));
    }

    return sqrt(sum);
}
