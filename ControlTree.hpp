//
// Created by Seth Wiesman on 12/12/15.
//

#ifndef THESIS_CONTROLTREE_H
#define THESIS_CONTROLTREE_H

#include <vector>

#include "IMetricTree.hpp"

namespace Thesis {
    template<typename T, double(*distance)(const T &, const T &)>
    class ControlTree : public IMetricTree<T, distance> {
    public:
        ControlTree(std::vector<T> points) : points(points) { }
        std::vector<T> search(const T &target, double radius) const;
        int getCalls() const;

    private:
        mutable int calls = 0;
        std::vector<T> points;
    };

    template<typename T, double(*distance)(const T &, const T &)>
    std::vector<T> ControlTree<T,distance>::search(const T &target, double radius) const {
        this->calls = 0;

        std::vector<T> inRange;
        for (auto point : points) {
            this->calls++;
            if (distance(point, target) <= radius) {
                inRange.push_back(point);
            }
        }

        return inRange;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    int ControlTree<T,distance>::getCalls() const {
        return this->calls;
    }
}

#endif //THESIS_CONTROLTREE_H
