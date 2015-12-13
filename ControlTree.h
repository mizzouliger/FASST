//
// Created by Seth Wiesman on 12/12/15.
//

#ifndef THESIS_CONTROLTREE_H
#define THESIS_CONTROLTREE_H


#include "IMetricTree.h"

template<typename T, double(*distance)(const T&, const T&)>
class ControlTree : public IMetricTree<T, distance>{
public:
    ControlTree(std::vector<T> points) : points(points) {}

    std::vector<T> search(const T& target, double radius) const {
        this->calls = 0;

        std::vector<T> found;
        for (auto point : points) {
            this->calls++;
            if (distance(point, target) <= radius) {
                found.push_back(point);
            }
        }

        return found;
    }

    int getCalls() const {
        return calls;
    }

private:

    mutable int calls = 0;
    std::vector<T> points;
};


#endif //THESIS_CONTROLTREE_H
