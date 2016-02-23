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

double ::Thesis::Metrics::editDistance(const std::string &s1, const std::string &s2) {
    auto length1 = s1.length();
    auto length2 = s2.length();

    std::vector<std::vector<int>>  table(length1 + 1, std::vector<int>(length2));

    for (auto i = 0; i <= length1; i++) {
        table[i][0] = i;
    }

    for (auto i = 1; i <= length2; i++) {
        table[0][i] = i;
    }

    for (auto i = 1; i <= length1; i++) {
        for (auto j = 1; j <= length2; j++) {
            auto min = std::min(table[i - 1][j], table[i][j-1]) + 1;
            table[i][j] = std::min(min, table[i - 1][j - 1] + (s1[i - 1] == s2[j-1] ? 0 : 1));
        }
    }

    return static_cast<double>(table[length1][length2]);
}
