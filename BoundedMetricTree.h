//
// Created by Seth Wiesman on 12/22/15.
//

#ifndef THESIS_BOUNDEDMETRICTREE_H
#define THESIS_BOUNDEDMETRICTREE_H

#include <queue>

#include "IMetricTree.h"

namespace Thesis {
    template<typename T, double (*distance)(const T &, const T &)>
    class BoundedMetricTree : public IMetricTree<T, distance> {
        struct Node {
            struct Distances {
                std::vector<double> nearest;
                std::vector<double> furthest;
            };

            T point;

            Distances left_distances;
            Distances right_distances;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point) { }
        };

        using node_itr = typename std::vector<std::shared_ptr<typename BoundedMetricTree<T, distance>::Node>>::iterator;

        std::shared_ptr<Node> root;
        mutable int calls;

        std::shared_ptr<Node> build_tree(const node_itr low, const node_itr high, std::vector<T> &ancestors) const;

        void set_bounds(
                std::shared_ptr<Node> &root,
                std::shared_ptr<Node> left,
                const std::vector<T> &ancestors,
                std::function<void(std::shared_ptr<Node>&, double)> push
        ) const;

        void set_bounds(std::shared_ptr<Node> &root, std::shared_ptr<Node> right, const std::vector<T> &ancestors) const;

        void search(
                std::shared_ptr<Node> node,
                std::vector<T> &inRange,
                const T &target,
                const double radius
        ) const;

    public:
        BoundedMetricTree(std::vector<T> points);

        std::vector<T> search(const T &target, const double radius) const;

        //super naive implementation, will fix this later
        T nearest_neighbor(const std::shared_ptr<Node> node, const T &target) const;

        T furthest_neighbor(const std::shared_ptr<Node> node, const T &target) const;

        int getCalls() const;
    };

    template<typename T, double (*distance)(const T &, const T &)>
    BoundedMetricTree<T, distance>::BoundedMetricTree(std::vector<T> points) : calls(0) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }

        std::vector<T> ancestors;
        this->root = build_tree(nodes.begin(), nodes.end(), ancestors);
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::vector<T> BoundedMetricTree<T, distance>::search(const T &target, double radius) const {
        std::vector<T> inRange;
        this->calls = 0;

        search(root, inRange, target, radius);
        return inRange;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    int BoundedMetricTree<T, distance>::getCalls() const {
        return this->calls;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    T BoundedMetricTree<T, distance>::nearest_neighbor(const std::shared_ptr<Node> node, const T &target) const {
        T nearest_point = node->point;
        double nearest_distance = distance(target, nearest_point);

        std::queue<std::shared_ptr<Node>> queue;
        queue.push(node);

        while (!queue.empty()) {
            auto next = queue.front();
            queue.pop();

            double next_distance = distance(next->point, target);

            if (next_distance < nearest_distance) {
                nearest_distance = next_distance;
                nearest_point = next->point;
            }

            if (next->left) {
                queue.push(next->left);
            }

            if (next->right) {
                queue.push(next->right);
            }
        }

        return nearest_point;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    T BoundedMetricTree<T, distance>::furthest_neighbor(const std::shared_ptr<Node> node, const T &target) const {
        T furthest_point = node->point;
        double furthest_distance = distance(target, furthest_point);

        std::queue<std::shared_ptr<Node>> queue;
        queue.push(node);

        while (!queue.empty()) {
            auto next = queue.front();
            queue.pop();

            double next_distance = distance(next->point, target);

            if (furthest_distance < next_distance) {
                furthest_distance = next_distance;
                furthest_point = next->point;
            }

            if (next->left) {
                queue.push(next->left);
            }

            if (next->right) {
                queue.push(next->right);
            }
        }

        return furthest_point;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::shared_ptr<typename BoundedMetricTree<T, distance>::Node>
    BoundedMetricTree<T, distance>::build_tree(const node_itr low, const node_itr high, std::vector<T> &ancestors) const {
        if (low == high) {
            return nullptr;
        }

        if ((high - low) == 1) {
            (*low)->left_distances.nearest.clear();

            configure_left(*low,  nullptr, ancestors);
            configure_right(*low, nullptr, ancestors);

            return *low;
        }

        for (auto itr = low + 1; itr != high; itr++) {
            (*itr)->left_distances.nearest[0] = distance((*low)->point, (*itr)->point);
        }

        const auto median = low + (high - low) / 2;

        std::nth_element(low + 1, median, high, [](const auto n1, const auto n2) {
            return n1->left_distances.nearest[0] < n2->left_distances.nearest[0];
        });

        ancestors.push_back((*low)->point);

        auto left  = build_tree(low + 1, median, ancestors);
        auto right = build_tree(median, high, ancestors);

        (*low)->left_distances.nearest.clear();

        configure_left(*low, left, ancestors);
        configure_right(*low, right, ancestors);

        ancestors.pop_back();

        (*low)->left = left;
        (*low)->right = right;

        return (*low);
    }
}


#endif //THESIS_BOUNDEDMETRICTREE_H
