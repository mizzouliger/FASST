//
// Created by Seth Wiesman on 11/22/15.
//

#include <algorithm>
#include <functional>
#include <numeric>
#include <string>
#include "Point.h"

using namespace Thesis;

Point::Point(std::vector<double> elements): elements(elements) {}

double& Thesis::Point::operator[](int const i) {
    return this->elements[i];
}

double const& Thesis::Point::operator[](int const i) const {
    return this->elements[i];
}

std::string Thesis::Point::to_string() {
    std::string elems;
    for (auto i = 0; i < this->elements.size(); i++) {
        elems += std::to_string(this->elements[i]);

        if (i + 1 != this->elements.size()) {
            elems += ",";
        }
    }

    return "(" + elems + ")";
}

double Thesis::Point::EuclideanDistance(Point a, Point b) {
    check_size(a, b, "Euclidean distance calculation requires points of equal dimensionality");

    return std::inner_product(a.elements.begin(), a.elements.end(),
                              b.elements.begin(), double(0), std::plus<double>(),
                              [](double x1, double x2) {
                                    return (x2 - x1) * (x2 - x1);
                              }
    );
}

void Point::check_size(Point const& a, Point const& b, std::string msg) {
    if (a.elements.size() != b.elements.size()) {
        throw std::domain_error(msg);
    }
}
