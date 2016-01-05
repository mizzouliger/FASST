//
// Created by Seth Wiesman on 11/22/15.
//

#include <numeric>
#include <string>
#include <cmath>
#include <functional>

#include "Point.h"

Point::Point(std::vector<double> elements): elements(elements) {}

double& Point::operator[](int const i) {
    return this->elements[i];
}

double const& Point::operator[](int const i) const {
    return this->elements[i];
}

bool Point::operator==(Point const& that) const {
    return std::equal(this->elements.begin(), this->elements.end(), that.elements.begin());
}

unsigned long Point::size() const {
    return elements.size();
}

std::string Point::to_string() const {
    std::string elements;
    for (auto i = 0; i < this->elements.size(); i++) {
        elements += std::to_string(this->elements[i]);

        if (i + 1 != this->elements.size()) {
            elements += ",";
        }
    }

    return "(" + elements + ")";
}

double Point::euclidean_distance(const Point& a, const Point& b) {
    //check_size(a, b, "Euclidean distance calculation requires points of equal dimensionality");

    double sum = 0.0;
    for (auto i = 0; i < a.elements.size(); i++) {
        sum += (a.elements[i] - b.elements[i]) * (a.elements[i] - b.elements[i]);
    }

    return sqrt(sum);
}

void Point::check_size(Point const& a, Point const& b, std::string msg) {
    if (a.elements.size() != b.elements.size()) {
        throw std::domain_error(msg);
    }
}

Point Point::origin(std::size_t dim) {
    return Point(std::vector<double>(dim, 0.0));
}
