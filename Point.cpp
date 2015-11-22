//
// Created by Seth Wiesman on 11/22/15.
//

#include "Point.h"

using namespace std;

template<typename T, int dim>
Thesis::Point::Point(vector<T> elements): elements(elements) {}

template<typename T, int dim>
T & Thesis::Point::operator[](int const i) {
    return this->elements[i];
}

template<typename T, int dim>
T const& Thesis::Point::operator[](int const i) const {
    return this->elements[i];
}

template<typename T, int dim>
void Thesis::Point::operator+=(Point const& that) {
    for (int i = 0; i < dim; i++) {
        this->elements[i] += that.elements[i];
    }
};

template<typename T, int dim>
void Thesis::Point::operator-=(Point const& that) {
    for (int i = 0; i < dim; i++) {
        this->elements[i] -= that.elements[i];
    }
};

friend Point Thesis::Point::operator+(Point const& a, Point const& b) {
    Point ret(a);
    ret += a;
    return ret;
}

friend Point Thesis::Point::operator-(Point const& a, Point const& b) {
    Point ret(a);
    ret -= a;
    return ret;
}