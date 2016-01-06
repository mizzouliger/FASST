//
// Created by Seth Wiesman on 11/30/15.
//

#ifndef THESIS_IMETRICTREE_H
#define THESIS_IMETRICTREE_H

#include <vector>

namespace Thesis {
    template<typename T, double(*distance)(const T &, const T &)>
    class IMetricTree {
    public:
        virtual std::vector<T> search(const T &target, double radius) const = 0;

        virtual int getCalls() const = 0;
    };
}

#endif //THESIS_IMETRICTREE_H
