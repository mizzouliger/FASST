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

            Node(T point) : point(point) {
                this->left_distances.nearest.reserve(1);
            }
        };

        using node_itr = typename std::vector<std::shared_ptr<typename BoundedMetricTree<T, distance>::Node>>::iterator;

        std::shared_ptr<Node> root;
        mutable int calls;

        std::shared_ptr<Node> build_tree(const node_itr low, const node_itr high, std::vector<T> &ancestors) const;

        T nearest_neighbor(const std::shared_ptr<Node> node, const T target) const;

        T furthest_neighbor(const std::shared_ptr<Node> node, const T target) const;

        bool visit_node(
                const typename Node::Distances,
                const std::vector<double> target_ancestors,
                const double radius
        ) const;

        void search(
                std::shared_ptr<Node> node,
                std::vector<T> &inRange,
                const T &target,
                const double radius,
                std::vector<double> &ancestors
        ) const;

    public:
        BoundedMetricTree(std::vector<T> points);

        std::vector<T> search(const T &target, const double radius) const;

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

        std::vector<double> ancestors;

        search(root, inRange, target, radius, ancestors);
        return inRange;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    int BoundedMetricTree<T, distance>::getCalls() const {
        return this->calls;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::shared_ptr<typename BoundedMetricTree<T, distance>::Node>
    BoundedMetricTree<T, distance>::build_tree(const node_itr low, const node_itr high, std::vector<T> &ancestors) const {
        if (low == high) {
            return nullptr;
        }

        if ((high - low) == 1) {
            const std::vector<double> empty(ancestors.size(), 0);

            (*low)->left_distances.nearest  = empty;
            (*low)->left_distances.furthest = empty;

            (*low)->right_distances.nearest = empty;
            (*low)->right_distances.furthest = empty;
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
        (*low)->left_distances.nearest.clear();

        const auto left  = build_tree(low + 1, median, ancestors);

        if (left != nullptr) {
            for (auto &point : ancestors) {
                const T nearest  = nearest_neighbor(left, point);
                const T furthest = furthest_neighbor(left, point);

                (*low)->left_distances.nearest.push_back(distance((*low)->point, nearest));
                (*low)->left_distances.furthest.push_back(distance((*low)->point, furthest));
            }
        } else {
            const std::vector<double> empty(ancestors.size(), 0);

            (*low)->left_distances.nearest  = empty;
            (*low)->left_distances.furthest = empty;
        }

        const auto right = build_tree(median, high, ancestors);

        if (right != nullptr) {
            for (auto &point : ancestors) {
                const T nearest  = nearest_neighbor(right, point);
                const T furthest = furthest_neighbor(right, point);

                (*low)->right_distances.nearest.push_back(distance((*low)->point, nearest));
                (*low)->right_distances.furthest.push_back(distance((*low)->point, furthest));
            }
        } else {
            const std::vector<double> empty(ancestors.size(), 0);

            (*low)->right_distances.nearest = empty;
            (*low)->right_distances.furthest = empty;
        }

        ancestors.pop_back();

        (*low)->left  = left;
        (*low)->right = right;

        return (*low);
    }

    template<typename T, double (*distance)(const T &, const T &)>
    T BoundedMetricTree<T, distance>::nearest_neighbor(const std::shared_ptr<Node> node, const T target) const {
        std::queue<std::shared_ptr<Node>> nodes;
        nodes.push(node);

        T best_point = node->point;
        double best_distance = distance(target, best_point);

        while (!nodes.empty()) {
            const auto next_node = nodes.front();
            nodes.pop();

            const double next_distance = distance(target, next_node->point);

            if (next_distance < best_distance) {
                best_point = next_node->point;
                best_distance = next_distance;
            }

            if (next_node->left) {
                nodes.push(next_node->left);
            }

            if (next_node->right) {
                nodes.push(next_node->right);
            }
        }

        return best_point;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    T BoundedMetricTree<T, distance>::furthest_neighbor(const std::shared_ptr<Node> node, const T target) const {
        std::queue<std::shared_ptr<Node>> nodes;
        nodes.push(node);

        T best_point = node->point;
        double best_distance = distance(target, best_point);

        while (!nodes.empty()) {
            const auto next_node = nodes.front();
            nodes.pop();

            const double next_distance = distance(target, next_node->point);

            if (next_distance < best_distance) {
                best_point = next_node->point;
                best_distance = next_distance;
            }

            if (next_node->left) {
                nodes.push(next_node->left);
            }

            if (next_node->right) {
                nodes.push(next_node->right);
            }
        }

        return best_point;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    void BoundedMetricTree<T, distance>::search(
            std::shared_ptr<Node> node,
            std::vector<T> &inRange,
            const T &target,
            const double radius,
            std::vector<double> &ancestors
    ) const {
        if (node == nullptr) {
            return;
        }

        const auto dist = distance(target, node->point);
        ancestors.push_back(dist);

        if (dist <= radius) {
            inRange.push_back(node->point);
            this->calls++;
        }

        if (node->left && visit_node(node->left_distances, ancestors, radius)) {
            search(node->left, inRange, target, radius, ancestors);
        }

        if (node->right && visit_node(node->right_distances, ancestors, radius)) {
            search(node->right, inRange, target, radius, ancestors);
        }

        ancestors.pop_back();
    }

    template<typename T, double (*distance)(const T &, const T &)>
    bool BoundedMetricTree<T, distance>::visit_node(
            const typename Node::Distances distances,
            const std::vector<double> target_ancestors,
            const double radius
    ) const {
        if (distances.nearest.size() == 0) {
            return false;
        }
        for (auto i = distances.nearest.size() - 1; i >= 0; i++) {
            if (distances.nearest[i] <= target_ancestors[i] && distances.furthest[i] <= target_ancestors[i]) {
                return true;
            } else if (target_ancestors[i] <= distances.nearest[i] && target_ancestors[i] - distances.nearest[i] <= radius) {
                return true;
            } else if (target_ancestors[i] - distances.furthest[i] <= radius){
                return true;
            }
        }

        return false;
    }
}


#endif //THESIS_BOUNDEDMETRICTREE_H
