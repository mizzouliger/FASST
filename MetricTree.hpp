//
// Created by Seth Wiesman on 11/22/15.
//

#ifndef THESIS_METRICTREE_H
#define THESIS_METRICTREE_H

#include <vector>
#include <queue>
#include <functional>

namespace Thesis {
    namespace MetricTreeBench {
        int distanceCalls = 0;
        int nodesVisited  = 0;
    }
    template<typename T, double(*distance)(const T &, const T &)>
    class MetricTree {
        struct Node {
            T point;

            double innerRadius;
            double outerRadius;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point), innerRadius(0), outerRadius(0) { }

            void search(const T& target, const double radius, std::vector<T> &inRange) const;
        };

        using node_itr = typename std::vector<std::shared_ptr<typename MetricTree<T, distance>::Node>>::iterator;

        std::shared_ptr<Node> root;

        std::shared_ptr<Node> buildTree(const node_itr begin, const node_itr high) const;

    public:
        MetricTree(std::vector<T> points);

        std::vector<T> search(const T &target, double radius) const;

        int getCalls() const;

        int getNodesVisited() const;
    };

    template<typename T, double(*distance)(const T &, const T &)>
    MetricTree<T, distance>::MetricTree(std::vector<T> points) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }
        this->root = buildTree(nodes.begin(), nodes.end());
    }

    template<typename T, double(*distance)(const T &, const T &)>
    int MetricTree<T, distance>::getCalls() const {
        return MetricTreeBench::distanceCalls;
    }

    template<typename T, double(*distance)(const T&, const T&)>
    int MetricTree<T, distance>::getNodesVisited() const {
        return MetricTreeBench::nodesVisited;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    std::vector<T> MetricTree<T, distance>::search(const T &target, double radius) const {
        MetricTreeBench::distanceCalls = 0;
        MetricTreeBench::nodesVisited  = 0;

        std::vector<T> inRange;
        if (this->root != nullptr) {
            this->root->search(target, radius, inRange);
        }
        return inRange;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    std::shared_ptr<typename MetricTree<T, distance>::Node>
    MetricTree<T, distance>::buildTree(const node_itr begin, const node_itr end) const {
        if (begin == end) {
            return nullptr;
        }

        if ((end - begin) == 1) {
            return *begin;
        }

        for (auto itr = begin + 1; itr != end; itr++) {
            (*itr)->innerRadius = distance((*begin)->point, (*itr)->point);
        }

        const auto median = begin + (end - begin) / 2;

        std::nth_element(begin + 1, median, end, [](const auto left, const auto right) {
            return left->innerRadius < right->innerRadius;
        });

        (*begin)->outerRadius = (*median)->innerRadius;

        const auto pointOnInnerRadius = std::max_element(begin + 1, median, [](const auto left, const auto right) {
            return left->innerRadius < right->innerRadius;
        });

        (*begin)->innerRadius = (*pointOnInnerRadius)->innerRadius;

        (*begin)->left  = buildTree(begin + 1, median);
        (*begin)->right = buildTree(median, end);

        return *begin;
    }

    template<typename T, double(*distance)(const T&, const T&)>
    void MetricTree<T, distance>::Node::search(const T &target, const double radius, std::vector<T> &inRange) const {
        MetricTreeBench::distanceCalls++;
        MetricTreeBench::nodesVisited++;

        const auto dist = distance(this->point, target);

        if (dist <= radius) {
            inRange.push_back(this->point);
        }

        if (this->left != nullptr && dist - radius <= this->innerRadius) {
            this->left->search(target, radius, inRange);
        }

        if (this->right != nullptr && dist - radius <= this->outerRadius) {
            this->right->search(target, radius, inRange);
        }
    }
}

#endif
