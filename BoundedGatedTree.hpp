//
// Created by Seth Wiesman on 12/28/15.
//

#ifndef THESIS_BOUNDEDGATEDTREE_HPP
#define THESIS_BOUNDEDGATEDTREE_HPP

#include "IMetricTree.h"
#include "TriangleUtils.hpp"

namespace Thesis {
    template<typename T, double(*distance)(const T &, const T &)>
    class BoundedGatedTree : public IMetricTree<T, distance> {
        struct Node {
            struct Distances {
                double nearest;
                double furthest;

                Distances() : nearest(0), furthest(0) { }
            };

            T point;

            Distances left_distances;
            Distances right_distances;

            std::vector<double> parent_distances;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point) {
                this->parent_distances.push_back(TriangleUtils::infinity);
            }
        };

        std::shared_ptr<Node> root;
        mutable int calls;

        using node_itr = typename std::vector<std::shared_ptr<typename BoundedGatedTree<T, distance>::Node>>::iterator;

        std::shared_ptr<Node> build_tree(const node_itr low, const node_itr high) const;

        void search(
                const std::shared_ptr<Node> node,
                std::vector<T> &inRange,
                const T &target,
                const double radius,
                std::vector<double> ancestors
        ) const;

    public:

        BoundedGatedTree(std::vector<T> points);

        std::vector<T> search(const T &target, double radius) const;

        int getCalls() const;
    };

    template<typename T, double(*distance)(const T &, const T &)>
    BoundedGatedTree::BoundedGatedTree(std::vector<T> points) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }

        this->root = build_tree(nodes.begin(), nodes.end());
    }

    template<typename T, double(*distance)(const T &, const T &)>
    std::vector<T> BoundedGatedTree<T, distance>::search(const T &target, double radius) const {
        std::vector<T> inRange;
        this->calls = 0;

        search(root, inRange, target, radius, {TriangleUtils::infinity});
        return inRange;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    int BoundedGatedTree<T, distance>::getCalls() const {
        return this->calls;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    std::shared_ptr<typename BoundedGatedTree<T, distance>::Node>
    BoundedGatedTree::build_tree(const node_itr low, const node_itr high) const {
        if (low == high) {
            return nullptr;
        }

        if ((high - low) == 1) {
            return *low;
        }

        for (auto itr = low + 1; itr != high; itr++) {
            (*itr)->left_distances.nearest = distance((*low)->point, (*itr)->point);
            (*itr)->parent_distances.push_back((*itr)->left_distances.nearest);
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

    template<typename T, double(*distance)(const T &, const T &)>
    void BoundedGatedTree::search(
            const std::shared_ptr<Node> node,
            std::vector<T> &inRange,
            const T &target,
            const double radius,
            std::vector<double> ancestors
    ) const {
        if (node == nullptr) {
            return;
        }

        const auto minDistance = TriangleUtils::maximize_minimum_triangle_length(node->parent_distances, ancestors);
        const auto maxDistance = TriangleUtils::minimize_maximum_triangle_length(node->parent_distances, ancestors);

        if (radius < minDistance || maxDistance <= radius) {

            if (maxDistance <= radius) {
                inRange.push_back(node->point);
            }

            ancestors.push_back(TriangleUtils::infinity);
            if (minDistance - radius <= node->innerRadius) {
                search(node->left, inRange, target, radius, ancestors);
            }

            if (maxDistance + radius >= node->outerRadius) {
                search(node->right, inRange, target, radius, ancestors);
            }

        } else {
            const auto dist = distance(node->point, target);
            this->calls++;

            ancestors.push_back(dist);

            if (dist <= radius) {
                inRange.push_back(node->point);
            }

            if (dist - radius <= node->innerRadius) {
                search(node->left, inRange, target, radius, ancestors);
            }

            if (dist + radius >= node->outerRadius) {
                search(node->right, inRange, target, radius, ancestors);
            }
        }
    }
}


#endif //THESIS_BOUNDEDGATEDTREE_HPP
