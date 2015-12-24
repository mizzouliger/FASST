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
    public:
        BoundedMetricTree(std::vector<T> points);

        std::vector<T> search(const T &target, const double radius) const;

        //super naive implementation, will fix this latter
        T nearest_neighbor(const T &target);

        int getCalls() const;

    private:

        struct Node {
            struct Distances {
                double nearest;
                double furthest;

                Distances() : nearest(0), furthest(0) { }
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

        std::shared_ptr<Node> build_tree(const node_itr low, const node_itr high) const;

        void search(
                std::shared_ptr<Node> node,
                std::vector<T> &inRange,
                const T &target,
                const double radius
        ) const;
    };

    template<typename T, double (*distance)(const T &, const T &)>
    BoundedMetricTree<T,distance>::BoundedMetricTree(std::vector<T> points) : calls(0) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }
        this->root = build_tree(nodes.begin(), nodes.end());
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::vector<T> BoundedMetricTree<T,distance>::search(const T &target, double radius) const {
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
    T BoundedMetricTree<T, distance>::nearest_neighbor(const T &target) {
        T nearest_point = root->point;
        T nearest_distance = distance(target, root->point);

        std::queue<T> q;
        q.push(root->point);

        while (!q.empty()) {
            auto next = q.front();
            q.pop();

            const auto d = distance(next, target);
            if (d < nearest_distance) {
                nearest_distance = d;
                nearest_point = next;
            }

            if (next->left) {
                q.push(next->left);
            }

            if (next->right) {
                q.push(next->right);
            }
        }

        return nearest_point;
    }


    template<typename T, double (*distance)(const T &, const T &)>
    std::shared_ptr<typename BoundedMetricTree<T, distance>::Node>
    BoundedMetricTree<T, distance>::build_tree(const node_itr low, const node_itr high) const {
        if (low == high) {
            return nullptr;
        }

        if ((high - low) == 1) {
            return *low;
        }

        for (auto itr = low + 1; itr != high; itr++) {
            (*itr)->left_distances.nearest = distance((*low)->point, (*itr)->point);
        }

        const auto median = low + (high - low) / 2;

        std::nth_element(low + 1, median, high, [](const auto n1, const auto n2) {
            return n1->left_distances.nearest < n2->left_distances.nearest;
        });

        const auto left_min = std::min_element(low + 1, median, [](const auto n1, const auto n2) {
            return n1->left_distances.nearest < n2->left_distances.nearest;
        });

        const auto left_max = std::max_element(low + 1, median, [](const auto n1, const auto n2) {
            return n1->left_distances.nearest < n2->left_distances.nearest;
        });

        const auto right_max = std::max_element(median, high, [](const auto n1, const auto n2) {
            return n1->left_distances.nearest < n2->left_distances.nearest;
        });

        (*low)->left_distances.nearest = (*left_min)->left_distances.nearest;
        (*low)->left_distances.furthest = (*left_max)->left_distances.nearest;

        (*low)->right_distances.nearest = (*median)->left_distances.nearest;
        (*low)->right_distances.furthest = (*right_max)->left_distances.nearest;

        (*low)->left = build_tree(low + 1, median);
        (*low)->right = build_tree(median, high);

        return (*low);
    }

    template<typename T, double (*distance)(const T &, const T &)>
    void BoundedMetricTree<T, distance>::search(
            std::shared_ptr<Node> node,
            std::vector<T> &inRange,
            const T &target,
            const double radius
    ) const {
        if (node == nullptr) {
            return;
        }

        const auto dist = distance(node->point, target);
        this->calls++;

        if (dist <= radius) {
            inRange.push_back(node->point);
        }

        if (dist + radius >= node->left_distances.nearest && dist - radius < node->left_distances.furthest) {
            search(node->left, inRange, target, radius);
        }

        if (dist + radius >= node->right_distances.nearest && dist - radius < node->right_distances.furthest) {
            search(node->right, inRange, target, radius);
        }
    }
}


#endif //THESIS_BOUNDEDMETRICTREE_H
