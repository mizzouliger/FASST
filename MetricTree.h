//
// Created by Seth Wiesman on 11/22/15.
//

#ifndef THESIS_METRICTREE_H
#define THESIS_METRICTREE_H

#include <vector>
#include <queue>
#include <memory>
#include "IMetricTree.h"

template<
        typename T,
        double(*distance)(const T&, const T&)
>
class MetricTree : public IMetricTree<T, distance> {
public:
    MetricTree(std::vector<T> points) {
        std::vector<std::shared_ptr<Node>> nodes;
        for (auto& point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }
        this->root = build_tree(nodes, 0, points.size());
    }

    std::vector<T> search(const T& target, double radius) const {
        std::vector<T> results;
        search(root, results, target, radius);
        return results;
    }

    T nearest_neighbor(const T& target) const {
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

            const auto distance_to_node = distance(target, node->point);

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

private:
    struct Node {
        T point;

        double innerRadius;
        double outerRadius;

        std::shared_ptr<Node> left;
        std::shared_ptr<Node> right;

        Node(T point) : point(point), innerRadius(0), outerRadius(0) {}
    };

    std::shared_ptr<Node> root;

    struct pair_compare {
        bool operator()(std::pair<double, std::shared_ptr<Node>> first, std::pair<double, std::shared_ptr<Node>> second) {
            return std::get<0>(first) < std::get<0>(second);
        }
    };

    std::shared_ptr<Node> build_tree(std::vector<std::shared_ptr<Node>> points, std::size_t low, std::size_t high) {
        if (low == high) {
            return nullptr;
        }

        if ((high - low) == 1) {
            return points[low];
        }

        for (auto i = low + 1; i < high; i++) {
            points[i]->innerRadius = distance(points[low]->point, points[i]->point);
        }

        const auto mid = (high + low) / 2;

        const auto first  = points.begin() + low + 1;
        const auto median = points.begin() + mid;
        const auto last   = points.begin() + high;

        const auto inner_radius_compare = [](const std::shared_ptr<Node> n1, const std::shared_ptr<Node> n2){
            return n1->innerRadius < n2->innerRadius;
        };

        nth_element(first, median, last, inner_radius_compare);

        points[low]->outerRadius = (*median)->innerRadius;

        const auto pointOnInnerRadius = max_element(first, median, inner_radius_compare);

        points[low]->innerRadius = (*pointOnInnerRadius)->innerRadius;

        points[low]->left  = build_tree(points, low + 1, mid);
        points[low]->right = build_tree(points, mid, high);

        return points[low];
    }

    void search(std::shared_ptr<Node> root, std::vector<T>& result, const T& target, double radius) const {
        if (root == nullptr) {
            return;
        }

        const auto dist = distance(root->point, target);

        if (dist <= radius) {
            result.push_back(root->point);
        }

        if (dist + radius >= root->outerRadius) {
            search(root->right, result, target, radius);
        }

        if (dist - radius <= root->innerRadius) {
            search(root->left, result, target, radius);
        }
    }
};

#endif //THESIS_METRICTREE_H
