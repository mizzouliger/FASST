//
// Created by Seth Wiesman on 11/22/15.
//

#ifndef THESIS_METRICTREE_H
#define THESIS_METRICTREE_H

#include <vector>
#include <queue>
#include <algorithm>
#include <functional>

#include "ISearchTree.hpp"

namespace Thesis {

    template<typename T, double(*distance)(const T &, const T &)>
    class MetricTree : public ISearchTree<T, distance> {
    public:
        MetricTree(std::vector<T> points);

        std::vector<T> search(const T &target, double radius) const;

        int getCalls() const;

        int getNodesVisited() const;
    private:
        struct Node {
            T point;

            double innerRadius;
            double outerRadius;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point), innerRadius(0), outerRadius(0) { }
        };

        using node_itr = typename std::vector<std::shared_ptr<typename MetricTree<T, distance>::Node>>::iterator;

        mutable int calls;
        mutable int nodes_visited;
        std::shared_ptr<Node> root;

        std::shared_ptr<Node> build_tree(const node_itr low, const node_itr high) const;

        void search(const std::shared_ptr<Node> node, std::vector<T> &inRange, const T &target, const double radius) const;
    };

    template<typename T, double(*distance)(const T &, const T &)>
    MetricTree<T, distance>::MetricTree(std::vector<T> points) : calls(0), nodes_visited(0) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }
        this->root = build_tree(nodes.begin(), nodes.end());
    }

    template<typename T, double(*distance)(const T &, const T &)>
    int MetricTree<T, distance>::getCalls() const {
        return this->calls;
    }

    template<typename T, double(*distance)(const T&, const T&)>
    int MetricTree<T, distance>::getNodesVisited() const {
        return this->nodes_visited;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    std::vector<T> MetricTree<T, distance>::search(const T &target, double radius) const {
        std::vector<T> inRange;
        this->calls = 0;
        this->nodes_visited = 0;

        search(root, inRange, target, radius);
        return inRange;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    std::shared_ptr<typename MetricTree<T, distance>::Node>
    MetricTree<T, distance>::build_tree(const node_itr low, const node_itr high) const {
        if (low == high) {
            return nullptr;
        }

        if ((high - low) == 1) {
            return *low;
        }

        for (auto itr = low + 1; itr != high; itr++) {
            (*itr)->innerRadius = distance((*low)->point, (*itr)->point);
        }

        const auto median = low + (high - low) / 2;

        std::nth_element(low + 1, median, high, [](const auto n1, const auto n2) {
            return n1->innerRadius < n2->innerRadius;
        });

        (*low)->outerRadius = (*median)->innerRadius;

        const auto pointOnInnerRadius = std::max_element(low + 1, median, [](const auto n1, const auto n2) {
            return n1->innerRadius < n2->innerRadius;
        });

        (*low)->innerRadius = (*pointOnInnerRadius)->innerRadius;

        (*low)->left  = build_tree(low + 1, median);
        (*low)->right = build_tree(median, high);

        return *low;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    void MetricTree<T, distance>::search(
            const std::shared_ptr<Node> node,
            std::vector<T> &inRange,
            const T &target,
            const double radius
    ) const {
        if (node == nullptr) {
            return;
        }

        this->nodes_visited++;

        const auto dist = distance(node->point, target);
        this->calls++;

        if (dist <= radius) {
            inRange.push_back(node->point);
        }

        if (dist - radius <= node->innerRadius) {
            search(node->left, inRange, target, radius);
        }

        if (dist + radius >= node->outerRadius) {
            search(node->right, inRange, target, radius);
        }
    }
}

#endif
