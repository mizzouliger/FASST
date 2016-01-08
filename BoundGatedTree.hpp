//
// Created by Seth Wiesman on 1/7/16.
//

#ifndef THESIS_BOUNDGATEDTREE_HPP
#define THESIS_BOUNDGATEDTREE_HPP

#include <queue>

#include "IMetricTree.hpp"
#include "TriangleUtils.hpp"

namespace Thesis {
    template<typename T, double (*distance)(const T &, const T &)>
    class BoundGatedTree : public IMetricTree<T, distance> {
        struct Node {
            struct Distances {
                std::vector<double> nearest;
                std::vector<double> furthest;
            };

            T point;

            Distances left_distances;
            Distances right_distances;

            std::vector<double> parent_distances;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point) { }
        };

        using node_itr = typename std::vector<std::shared_ptr<typename BoundGatedTree<T, distance>::Node>>::iterator;

        std::shared_ptr<Node> root;
        mutable int calls;
        mutable int nodes_visited;

        std::shared_ptr<Node> build_tree(const node_itr low, const node_itr high, std::vector<T> &ancestors) const;

        T nearest_neighbor(const std::shared_ptr<Node> node, const T target) const;

        T furthest_neighbor(const std::shared_ptr<Node> node, const T target) const;

        bool visit_node(
                const typename Node::Distances,
                const std::vector<double> ancestors,
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
        BoundGatedTree(std::vector<T> points);

        std::vector<T> search(const T &target, const double radius) const;

        int getCalls() const;

        int getNodesVisited() const;
    };

    template<typename T, double (*distance)(const T &, const T &)>
    BoundGatedTree<T, distance>::BoundGatedTree(std::vector<T> points) : calls(0) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }

        std::vector<T> ancestors;
        this->root = build_tree(nodes.begin(), nodes.end(), ancestors);
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::vector<T> BoundGatedTree<T, distance>::search(const T &target, double radius) const {
        std::vector<T> inRange;
        this->calls = 0;
        this->nodes_visited = 0;

        std::vector<double> ancestors;

        search(root, inRange, target, radius, ancestors);
        return inRange;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    int BoundGatedTree<T, distance>::getCalls() const {
        return this->calls;
    }

    template<typename T, double(*distance)(const T&, const T&)>
    int BoundGatedTree<T, distance>::getNodesVisited() const {
        return this->nodes_visited;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::shared_ptr<typename BoundGatedTree<T, distance>::Node>
    BoundGatedTree<T, distance>::build_tree(const node_itr low, const node_itr high, std::vector<T> &ancestors) const {
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
            auto dist = distance((*low)->point, (*itr)->point);
            (*itr)->parent_distances.push_back(dist);
        }

        const auto median = low + (high - low) / 2;

        std::nth_element(low + 1, median, high, [](const auto n1, const auto n2) {
            return n1->parent_distances.back() < n2->parent_distances.back();
        });

        ancestors.push_back((*low)->point);

        (*low)->left  = build_tree(low + 1, median, ancestors);

        if ((*low)->left != nullptr) {
            for (auto &point : ancestors) {
                const T nearest  = nearest_neighbor((*low)->left, point);
                const T furthest = furthest_neighbor((*low)->left, point);

                (*low)->left_distances.nearest.push_back(distance((*low)->point, nearest));
                (*low)->left_distances.furthest.push_back(distance((*low)->point, furthest));
            }
        } else {
            const std::vector<double> empty(ancestors.size(), 0);

            (*low)->left_distances.nearest  = empty;
            (*low)->left_distances.furthest = empty;
        }

        (*low)->right = build_tree(median, high, ancestors);

        if ((*low)->right != nullptr) {
            for (auto &point : ancestors) {
                const T nearest  = nearest_neighbor((*low)->right, point);
                const T furthest = furthest_neighbor((*low)->right, point);

                (*low)->right_distances.nearest.push_back(distance((*low)->point, nearest));
                (*low)->right_distances.furthest.push_back(distance((*low)->point, furthest));
            }
        } else {
            const std::vector<double> empty(ancestors.size(), 0);

            (*low)->right_distances.nearest = empty;
            (*low)->right_distances.furthest = empty;
        }

        ancestors.pop_back();

        return (*low);
    }

    template<typename T, double (*distance)(const T &, const T &)>
    T BoundGatedTree<T, distance>::nearest_neighbor(const std::shared_ptr<Node> node, const T target) const {
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
    T BoundGatedTree<T, distance>::furthest_neighbor(const std::shared_ptr<Node> node, const T target) const {
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
    void BoundGatedTree<T, distance>::search(
            std::shared_ptr<Node> node,
            std::vector<T> &inRange,
            const T &target,
            const double radius,
            std::vector<double> &ancestors
    ) const {
        if (node == nullptr) {
            return;
        }

        this->nodes_visited++;

        const auto minDistance = TriangleUtils::maximize_minimum_triangle(node->parent_distances, ancestors);
        const auto maxDistance = TriangleUtils::minimize_maximum_triangle(node->parent_distances, ancestors);

        if (radius <= minDistance || maxDistance <= radius) {

            if (maxDistance <= radius) {
                inRange.push_back(node->point);
            }

            ancestors.push_back(TriangleUtils::infinity);
        } else {

            const auto dist = distance(target, node->point);
            this->calls++;

            ancestors.push_back(dist);

            if (dist <= radius) {
                inRange.push_back(node->point);
            }
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
    bool BoundGatedTree<T, distance>::visit_node(
            const typename Node::Distances distances,
            const std::vector<double> ancestors,
            const double radius
    ) const {

        for (auto i = 0; i < distances.nearest.size(); i++) {
            if (ancestors[i] == TriangleUtils::infinity) {
                continue;
            }
            if (ancestors[i] < distances.nearest[i] && ancestors[i] + radius < distances.nearest[i]) {
                return false;
            } else if (distances.furthest[i] < ancestors[i] && distances.furthest[i] + radius < ancestors[i]){
                return false;
            }
        }

        return true;
    }
}



#endif //THESIS_BOUNDGATEDTREE_HPP
