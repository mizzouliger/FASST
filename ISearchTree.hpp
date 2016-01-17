//
// Created by Seth Wiesman on 11/30/15.
//

#ifndef THESIS_FASSTTREE_H
#define THESIS_FASSTTREE_H

#include <vector>

namespace Thesis {
    template<typename T, double(*distance)(const T &, const T &)>
    class ISearchTree {
    public:
        virtual std::vector<T> search(const T &target, double radius) const = 0;

        virtual int getCalls() const = 0;

        virtual int getNodesVisited() const = 0;
    };
}

#endif
