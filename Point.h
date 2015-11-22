//
// Created by Seth Wiesman on 11/22/15.
//

#ifndef THESIS_POINT_H
#define THESIS_POINT_H

#include <vector>

namespace Thesis {

    template<typename T, int dim>
    class Point {
    public:
        Point(std::vector<T> elements);

        T& operator[] (int const i);
        T const& operator[](int const i) const;

        void operator+=(Point const& that);
        void operator-=(Point const& that);

        friend Point operator+(Point const& a, Point const& b);
        friend Point operator-(Point const& a, Point const& b);

        static double EuclideanDistance(Point p1, Point p2);
    private:
        std::vector<T> elements;
    };
}


#endif //THESIS_POINT_H
