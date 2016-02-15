//
// Created by Seth Wiesman on 1/7/16.
//

#ifndef THESIS_FASSTTREE_HPP
#define THESIS_FASSTTREE_HPP

#include <queue>

#include "ISearchTree.hpp"
#include "TriangleUtils.hpp"

namespace Thesis {
    template<typename T, double (*distance)(const T &, const T &)>
    class FasstTree : public ISearchTree<T, distance> {

        struct Annulus {
            double shortRadius;
            double longRadius;

            Annulus(double shortRadius, double longRadius) : shortRadius(shortRadius), longRadius(longRadius) { }
        };

        struct Node {
            T point;

            std::vector<Annulus> left_annuli;
            std::vector<Annulus> right_annuli;

            std::vector<double> pivots;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point) { }
        };

        using node_itr = typename std::vector<std::shared_ptr<typename FasstTree<T, distance>::Node>>::iterator;

        std::shared_ptr<Node> root;
        mutable int calls;
        mutable int nodes_visited;

        std::shared_ptr<Node> build_tree(const node_itr low, const node_itr high, int depth) const;

        bool visit_node(
                const typename std::vector<Annulus>,
                const std::vector<double> pivots,
                const double radius
        ) const;

        void search(
                std::shared_ptr<Node> node,
                std::vector<T> &inRange,
                const T &target,
                const double radius,
                std::vector<double> &pivots
        ) const;

    public:
        FasstTree(std::vector<T> points);

        std::vector<T> search(const T &target, const double radius) const;

        int getCalls() const;

        int getNodesVisited() const;
    };

    template<typename T, double (*distance)(const T &, const T &)>
    FasstTree<T, distance>::FasstTree(std::vector<T> points) : calls(0) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }

        this->root = build_tree(nodes.begin(), nodes.end(), 0);
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::vector<T> FasstTree<T, distance>::search(const T &target, double radius) const {
        std::vector<T> inRange;
        this->calls = 0;
        this->nodes_visited = 0;

        std::vector<double> pivots;

        search(root, inRange, target, radius, pivots);
        return inRange;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    int FasstTree<T, distance>::getCalls() const {
        return this->calls;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    int FasstTree<T, distance>::getNodesVisited() const {
        return this->nodes_visited;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::shared_ptr<typename FasstTree<T, distance>::Node>
    FasstTree<T, distance>::build_tree(const node_itr low, const node_itr high, int depth) const {
        if (low == high) {
            return nullptr;
        }

        if ((high - low) == 1) {
            return *low;
        }

        for (auto itr = low + 1; itr != high; itr++) {
            auto dist = distance((*low)->point, (*itr)->point);
            (*itr)->pivots.push_back(dist);
        }

        const auto median = low + (high - low) / 2;

        std::nth_element(low + 1, median, high, [](const auto left, const auto right) {
            return left->pivots.back() < right->pivots.back();
        });

        (*low)->left = build_tree(low + 1, median, depth + 1);
        (*low)->right = build_tree(median, high, depth + 1);

        for (auto i = 0; i < depth; i++) {

            if ((*low)->left != nullptr) {
                auto nearest = std::min_element(low + 1, median, [i](const auto left, const auto right) {
                    return left->pivots[i] < right->pivots[i];
                });

                auto furthest = std::max_element(low + 1, median, [i](const auto left, const auto right) {
                    return left->pivots[i] < right->pivots[i];
                });

                (*low)->left_annuli.push_back(Annulus((*nearest)->pivots[i], (*furthest)->pivots[i]));
            }

            if ((*low)->right != nullptr) {
                auto nearest = std::min_element(median, high, [i](const auto left, const auto right) {
                    return left->pivots[i] < right->pivots[i];
                });

                auto furthest = std::max_element(median, high, [i](const auto left, const auto right) {
                    return left->pivots[i] < right->pivots[i];
                });

                (*low)->right_annuli.push_back(Annulus((*nearest)->pivots[i], (*furthest)->pivots[i]));
            }
        }
        return (*low);
    }

    template<typename T, double (*distance)(const T &, const T &)>
    void FasstTree<T, distance>::search(
            std::shared_ptr<Node> node,
            std::vector<T> &inRange,
            const T &target,
            const double radius,
            std::vector<double> &pivots
    ) const {
        if (node == nullptr) {
            return;
        }

        this->nodes_visited++;

        const auto minDistance = TriangleUtils::maximize_minimum_triangle(node->pivots, pivots);
        const auto maxDistance = TriangleUtils::minimize_maximum_triangle(node->pivots, pivots);

        if (radius <= minDistance || maxDistance <= radius) {

            if (maxDistance <= radius) {
                inRange.push_back(node->point);
            }

            pivots.push_back(TriangleUtils::infinity);
        } else {

            const auto dist = distance(target, node->point);
            this->calls++;

            pivots.push_back(dist);

            if (dist <= radius) {
                inRange.push_back(node->point);
            }
        }

        if (node->left && visit_node(node->left_annuli, pivots, radius)) {
            search(node->left, inRange, target, radius, pivots);
        }

        if (node->right && visit_node(node->right_annuli, pivots, radius)) {
            search(node->right, inRange, target, radius, pivots);
        }

        pivots.pop_back();
    }

    template<typename T, double (*distance)(const T &, const T &)>
    bool FasstTree<T, distance>::visit_node(
            const typename std::vector<Annulus> annuli,
            const std::vector<double> pivots,
            const double radius
    ) const {

        for (auto i = 0; i < annuli.size(); i++) {
            if (pivots[i] == TriangleUtils::infinity) {
                continue;
            }
            if (pivots[i] < annuli[i].shortRadius && pivots[i] + radius < annuli[i].shortRadius) {
                return false;
            } else if (annuli[i].longRadius < pivots[i] && pivots[i] - radius > annuli[i].longRadius) {
                return false;
            }
        }

        return true;
    }
}

#endif
