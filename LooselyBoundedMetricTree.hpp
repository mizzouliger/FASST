//
// Created by Seth Wiesman on 1/3/16.
//

#ifndef THESIS_LOOSELYBOUNDEDMETRICTREE_HPP
#define THESIS_LOOSELYBOUNDEDMETRICTREE_HPP

#include "IMetricTree.h"

namespace Thesis {
    template<typename T, double (*distance)(const T &, const T &)>
    class LooselyBoundedMetricTree : public IMetricTree<T, distance> {
    public:
        LooselyBoundedMetricTree(std::vector<T> points);

        std::vector<T> search(const T &target, const double radius) const;

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

            std::vector<double> parent_distances;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point) {
                this->parent_distances.push_back(TriangleUtils::infinity);
            }
        };

        using node_itr = typename std::vector<std::shared_ptr<typename LooselyBoundedMetricTree<T, distance>::Node>>::iterator;

        std::shared_ptr<Node> root;
        mutable int calls;

        std::shared_ptr<Node> build_tree(const node_itr low, const node_itr high) const;

        void search(std::shared_ptr<Node> node, std::vector<T> &inRange, const T &target,
                    const double radius, std::vector<double> ancestors) const;
    };

    template<typename T, double (*distance)(const T &, const T &)>
    LooselyBoundedMetricTree<T, distance>::LooselyBoundedMetricTree(std::vector<T> points) : calls(0) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }
        this->root = build_tree(nodes.begin(), nodes.end());
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::vector<T> LooselyBoundedMetricTree<T, distance>::search(const T &target, double radius) const {
        std::vector<T> inRange;
        this->calls = 0;

        search(root, inRange, target, radius, {TriangleUtils::infinity});
        return inRange;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    int LooselyBoundedMetricTree<T, distance>::getCalls() const {
        return this->calls;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::shared_ptr<typename LooselyBoundedMetricTree<T, distance>::Node>
    LooselyBoundedMetricTree<T, distance>::build_tree(const node_itr low, const node_itr high) const {
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

    template<typename T, double (*distance)(const T &, const T &)>
    void LooselyBoundedMetricTree<T, distance>::search(
            std::shared_ptr<Node> node,
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
            if (maxDistance + radius >= node->left_distances.nearest &&
                minDistance - radius < node->left_distances.furthest) {
                search(node->left, inRange, target, radius, ancestors);
            }

            if (maxDistance + radius >= node->right_distances.nearest &&
                minDistance - radius < node->right_distances.furthest) {
                search(node->right, inRange, target, radius, ancestors);
            }

        } else {
            const auto dist = distance(node->point, target);
            ancestors.push_back(dist);

            this->calls++;

            if (dist <= radius) {
                inRange.push_back(node->point);
            }

            if (dist + radius >= node->left_distances.nearest && dist - radius < node->left_distances.furthest) {
                search(node->left, inRange, target, radius, ancestors);
            }

            if (dist + radius >= node->right_distances.nearest && dist - radius < node->right_distances.furthest) {
                search(node->right, inRange, target, radius, ancestors);
            }
        }
    }
}


#endif //THESIS_LOOSELYBOUNDEDMETRICTREE_HPP
