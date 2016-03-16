//
// Created by Seth Wiesman on 2/23/16.
//

#include <math.h>
#include <stdlib.h>
#include "DistanceMetrics.hpp"
const double LARGE_NUMBER = 65536.;
const double GAP_OPENING_COST = 10.;
const double GAP_EXTENSION_COST = .1;
const double NEW_GAP_COST = GAP_OPENING_COST + GAP_EXTENSION_COST;
const signed char BLOSUM62[][25] = { // the blosum 62 scoring matrix
        {4, 0, 0, -2, -1, -2, 0, -2, -1, 0, -1, -1, // A
                                             -1, -2, 0, -1, -1, -1, 1, 0, 0, 0, -3, 0, -2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 9, -3, -4, -2, -3, -3, -1, 0, -3, -1, // C
                                             -1, -3, 0, -3, -3, -3, -1, -1, 0, -1, -2, 0, -2},
        {-2, 0, -3, 6, 2, -3, -1, -1, -3, 0, -1, -4, // D
                                             -3, 1, 0, -1, 0, -2, 0, -1, 0, -3, -4, 0, -3},
        {-1, 0, -4, 2, 5, -3, -2, 0, -3, 0, 1, -3, // E
                                             -2, 0, 0, -1, 2, 0, 0, -1, 0, -2, -3, 0, -2},
        {-2, 0, -2, -3, -3, 6, -3, -1, 0, 0, -3, 0, // F
                                             0, -3, 0, -4, -3, -3, -2, -2, 0, -1, 1, 0, 3},
        {0, 0, -3, -1, -2, -3, 6, -2, -4, 0, -2, -4, // G
                                             -3, 0, 0, -2, -2, -2, 0, -2, 0, -3, -2, 0, -3},
        {-2, 0, -3, -1, 0, -1, -2, 8, -3, 0, -1, -3, // H
                                             -2, 1, 0, -2, 0, 0, -1, -2, 0, -3, -2, 0, 2},
        {-1, 0, -1, -3, -3, 0, -4, -3, 4, 0, -3, 2, // I
                                             1, -3, 0, -3, -3, -3, -2, -1, 0, 3, -3, 0, -1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {-1, 0, -3, -1, 1, -3, -2, -1, -3, 0, 5, -2, // K
                                             -1, 0, 0, -1, 1, 2, 0, -1, 0, -2, -3, 0, -2},
        {-1, 0, -1, -4, -3, 0, -4, -3, 2, 0, -2, 4, // L
                                             2, -3, 0, -3, -2, -2, -2, -1, 0, 1, -2, 0, -1},
        {-1, 0, -1, -3, -2, 0, -3, -2, 1, 0, -1, 2, // M
                                             5, -2, 0, -2, 0, -1, -1, -1, 0, 1, -1, 0, -1},
        {-2, 0, -3, 1, 0, -3, 0, 1, -3, 0, 0, -3, // N
                                             -2, 6, 0, -2, 0, 0, 1, 0, 0, -3, -4, 0, -2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {-1, 0, -3, -1, -1, -4, -2, -2, -3, 0, -1, -3, // P
                                             -2, -2, 0, 7, -1, -2, -1, -1, 0, -2, -4, 0, -3},
        {-1, 0, -3, 0, 2, -3, -2, 0, -3, 0, 1, -2, // Q
                                             0, 0, 0, -1, 5, 1, 0, -1, 0, -2, -2, 0, -1},
        {-1, 0, -3, -2, 0, -3, -2, 0, -3, 0, 2, -2, // R
                                             -1, 0, 0, -2, 1, 5, -1, -1, 0, -3, -3, 0, -2},
        {1, 0, -1, 0, 0, -2, 0, -1, -2, 0, 0, -2, // S
                                             -1, 1, 0, -1, 0, -1, 4, 1, 0, -2, -3, 0, -2},
        {0, 0, -1, -1, -1, -2, -2, -2, -1, 0, -1, -1, // T
                                             -1, 0, 0, -1, -1, -1, 1, 5, 0, 0, -2, 0, -2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, -1, -3, -2, -1, -3, -3, 3, 0, -2, 1, // V
                                             1, -3, 0, -2, -2, -3, -2, 0, 0, 4, -3, 0, -1},
        {-3, 0, -2, -4, -3, 1, -2, -2, -3, 0, -3, -2, // W
                                             -1, -4, 0, -4, -2, -3, -3, -2, 0, -3, 11, 0, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {-2, 0, -2, -3, -2, 3, -3, 2, -1, 0, -2, -1, // Y
                                             -1, -2, 0, -3, -1, -2, -2, -2, 0, -1, 2, 0, 7}
};

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

    std::vector<std::vector<int>> table(length1 + 1, std::vector<int>(length2 + 1));

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

double ::Thesis::Metrics::hammingDistance(const int &x, const int &y) {
    short dist = 0;
    int val = x^y;

    while(val) {
        ++dist;
        val &= val - 1;
    }

    return static_cast<double>(dist);
}

std::vector<std::vector<double>> vector2d(unsigned long n, unsigned long m) {
    std::vector<std::vector<double>> v;
    v.reserve(n);
    for (auto i = 0; i < n; i++) {
        v[i].reserve(m);
    }

    return v;
}

double ::Thesis::Metrics::blosum(const std::string& _s1, const std::string& _s2) {
    auto s1 = _s1;
    auto s2 = _s2;
    int n = s1.length() + 1, m = s2.length() + 1, i, j;

    //std::vector<std::vector<double>> r(n, std::vector<double>(m));
    //std::vector<std::vector<double>> t(n, std::vector<double>(m));
    //std::vector<std::vector<double>> s(n, std::vector<double>(m));
    double** r = (double**)malloc(sizeof(double*) * n);
    double** t = (double**)malloc(sizeof(double*) * n);
    double** s = (double**)malloc(sizeof(double*) * n);

    for (i = 0; i < n; i++) {
        r[i] = (double*)malloc(sizeof(double) * m);
        t[i] = (double*)malloc(sizeof(double) * m);
        s[i] = (double*)malloc(sizeof(double) * m);
    }
    //====
    // initialization

    r[0][0] = t[0][0] = s[0][0] = 0;

    for (i = 1; i < n; i++) {
        r[i][0] = -LARGE_NUMBER;
        s[i][0] = t[i][0] = -GAP_OPENING_COST - i * GAP_EXTENSION_COST;
    }

    for (j = 1; j < m; j++) {
        t[0][j] = -LARGE_NUMBER;
        s[0][j] = r[0][j] = -GAP_OPENING_COST - j * GAP_EXTENSION_COST;
    }

    //====
    // Smith-Waterman with affine gap costs

    for (i = 1; i < n; i++) {
        for (j = 1; j < m; j++) {
            r[i][j] = std::max(r[i][j - 1] - GAP_EXTENSION_COST, s[i][j - 1] - NEW_GAP_COST);
            t[i][j] = std::max(t[i - 1][j] - GAP_EXTENSION_COST, s[i - 1][j] - NEW_GAP_COST);
            s[i][j] = std::max(std::max(s[i - 1][j - 1] + BLOSUM62[s1[i - 1] - 'A'][s2[j - 1] - 'A'], r[i][j]), t[i][j]);
        }
    }

    //====
    // back tracking

    i = n - 1, j = m - 1;

    while (i > 0 || j > 0) {
        if (s[i][j] == r[i][j]) {
            s1.insert(i, 1, '-');
            j--;
        } else if (s[i][j] == t[i][j]) {
            s2.insert(j, 1, '-');
            i--;
        } else {
            i--, j--;
        }
    }

    //====
    // final score

    auto final_score = s[n-1][m-1];

    for (i = 0; i < n; i++) {
        free(r[i]);
        free(t[i]);
        free(s[i]);
    }

    free(r);
    free(t);
    free(s);
    return final_score;
}
