//
// Created by Seth Wiesman on 11/22/15.
//

#ifndef THESIS_POINT_H
#define THESIS_POINT_H

#include <vector>


class Point {
public:
    Point(std::vector<double> elements);

    double& operator[] (int const i);
    double const& operator[](int const i) const;
    bool operator==(Point const& that) const;
    std::string to_string() const;

    unsigned long size() const;
    static double euclidean_distance(const Point& a, const Point& b);
    static Point  origin(std::size_t dim);

private:
    std::vector<double> elements;

    static inline void check_size(Point const& a, Point const& b, std::string msg);
};

namespace std {
    template<>
    struct hash<Point> {
        size_t operator()(const Point & x) const {
            unsigned long h = 0, g;
            for (auto i = 0; i < x.size(); i++) {
                h = (h << 4) + static_cast<unsigned long>(x[i]);
                if ((g = h & 0xF0000000L)) {
                    h ^= g >> 24;
                }
                h &= ~g;
            }

            return h;
        }
    };
}

#endif //THESIS_POINT_H
