//
// Created by Seth Wiesman on 11/23/15.
//

#ifndef THESIS_ENHANCEDMETRICTRREE_H
#define THESIS_ENHANCEDMETRICTRREE_H

#include <functional>
#include <vector>
#include <queue>
#include <unordered_map>
#include <iomanip>

#include "IMetricTree.h"

template<
        typename T,
        double(*distance)(const T&, const T&),
        class hash  = std::hash<T>,
        class equal = std::equal_to<T>
>
class EnhancedMetricTree : public IMetricTree<T, distance> {
public:
    EnhancedMetricTree(std::vector<T> points) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto& point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }

        this->root = build_tree(nodes, 0, points.size());
    }

    std::vector<T> search(const T &target, double radius) const {
        std::vector<T> results;
        std::vector<double> predecessors;
        search(this->root, results, target, radius, predecessors);
        return results;
    }

    T nearest_neighbor(const T &target) const {
        std::unordered_map<T, double, hash, equal> distance_by_point;
        std::priority_queue<
                std::pair<double, std::shared_ptr<Node>>,
                std::vector<std::pair<double, std::shared_ptr<Node>>>,
                pair_compare> next_branch;

        auto nearest_point    = this->root->point;
        auto nearest_distance = distance(target, nearest_point);

        next_branch.push(std::make_pair(0.0, this->root));

        while (!next_branch.empty()) {
            const auto queue_node = next_branch.top();

            const auto branch_distance = std::get<0>(queue_node);
            const auto node            = std::get<1>(queue_node);

            if (branch_distance >= nearest_distance) {
                return nearest_point;
            }

            next_branch.pop();

            auto distance_to_node = distance(target, node->point);

            if (node->triangle_prune(distance_to_node, distance_by_point)) {
                continue;
            }

            distance_by_point[node->point] = distance_to_node;

            if (distance_to_node < nearest_distance) {
                nearest_distance = distance_to_node;
                nearest_point    = node->point;
            }

            if (node->left != nullptr) {
                if (distance_to_node > node->innerRadius) {
                    next_branch.push(std::make_pair(distance_to_node - node->innerRadius, node->left));
                } else {
                    next_branch.push(std::make_pair(0.0, node->left));
                }
            }

            if (node->right != nullptr){
                if (distance_to_node < node->outerRadius) {
                    if (distance_to_node < node->outerRadius) {
                        next_branch.push(std::make_pair(node->outerRadius - distance_to_node, node->right));
                    } else {
                        next_branch.push(std::make_pair(0.0, node->right));
                    }
                }
            }
        }

        return nearest_point;
    }

    void print(void) {
        Node::prettyprint(this->root);
    }

private:
    struct Node {
        T point;

        std::vector<double> predecessors;

        double innerRadius;
        double outerRadius;

        std::weak_ptr<Node> parent;

        std::shared_ptr<Node> right;
        std::shared_ptr<Node> left;

        Node(T point) : point(point), innerRadius(0), outerRadius(0) {}

        static void prettyprint(std::shared_ptr<Node> root, int depth=0) {
            if (root == nullptr) {
                std::cout << std::setw(2*depth) << ' ' << '~' << std::endl;                return;
            }

            prettyprint(root->right, depth + 4);
            std::cout << std::setw(depth) << ' ' << root->point.to_string() << std::endl;
            prettyprint(root->left, depth + 4);
        }

        bool triangle_prune(double dist, std::vector<double> distances) const {

            for (auto i = 0; i < predecessors.size(); i++) {

                const auto sum = predecessors[i] + distances[i];

                if (sum < dist) {
                    return true;
                }
            }

            return false;
        }

        bool triangle_prune(double dist, std::unordered_map<T, double> distance_by_point) const {
            auto weak_parent = this->parent;

            for (auto i = static_cast<int>(predecessors.size()) - 1; i >= 0 ; i--) {
                auto parent = weak_parent.lock();
                if (!parent) break;

                auto sum = predecessors[i] + distance_by_point[parent->point];

                if (dist > sum) {
                    return true;
                }

                weak_parent = parent->parent;
            }

            return false;
        }
    };

    std::shared_ptr<Node> root;

    struct pair_compare {
        bool operator()(std::pair<double, std::shared_ptr<Node>> first, std::pair<double, std::shared_ptr<Node>> second) {
            return std::get<0>(first) < std::get<0>(second);
        }
    };

    std::shared_ptr<Node> build_tree(std::vector<std::shared_ptr<Node>> points, std::size_t low, std::size_t high) {
        if (high == low) {
            return nullptr;
        }

        if (high - low == 1) {
            return points[low];
        }

        for (auto i = low + 1; i < high; i++) {
            points[i]->innerRadius = distance(points[low]->point, points[i]->point);
            points[i]->predecessors.push_back(points[i]->innerRadius);
        }

        const auto mid = (high + low) / 2;

        const auto first  = points.begin() + low + 1;
        const auto median = points.begin() + mid;
        const auto last   = points.begin() + high;

        const auto inner_radius_compare = [](const std::shared_ptr<Node> n1, const std::shared_ptr<Node> n2){
            return n1->innerRadius < n2->innerRadius;
        };

        std::nth_element(first, median, last, inner_radius_compare);

        points[low]->outerRadius = (*median)->innerRadius;

        const auto pointOnInnerRadius = std::max_element(first, median, inner_radius_compare);

        points[low]->innerRadius = (*pointOnInnerRadius)->innerRadius;

        points[low]->left  = build_tree(points, low + 1, mid);
        points[low]->right = build_tree(points, mid,     high);

        if (points[low]->left != nullptr) {
            points[low]->left->parent = points[low];
        }

        if (points[low]->right != nullptr) {
            points[low]->right->parent = points[low];
        }

        return points[low];
    }

    void search(std::shared_ptr<Node> node, std::vector<T> &result, const T &target, double radius, std::vector<double> predecessors) const {
        if (node == nullptr) {
            return;
        }

        const auto dist = distance(node->point, target);

        if (node->triangle_prune(dist, predecessors)) {
            return;
        }

        if (dist <= radius) {
            result.push_back(node->point);
        }

        predecessors.push_back(dist);

        search(node->right, result, target, radius, predecessors);
        search(node->left, result, target, radius, predecessors);

        predecessors.pop_back();
    }
};

#endif //THESIS_ENHANCEDMETRICTRREE_H
