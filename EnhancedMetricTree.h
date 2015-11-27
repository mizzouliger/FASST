//
// Created by Seth Wiesman on 11/23/15.
//

#ifndef THESIS_ENHANCEDMETRICTRREE_H
#define THESIS_ENHANCEDMETRICTRREE_H

#include <vector>
#include <functional>

bool triangle_prune(std::vector<double> first, std::vector<double> second, double dist) {
    for (auto i = 0; i < first.size(); i++) {

        const auto sum = first[i] + second[i];

        if (dist > sum) {
            return true;
        }
    }

    return false;
}

template<typename T>
class EnhancedMetricTree {
public:
    EnhancedMetricTree(std::vector<T> points, std::function<double(const T&, const T&)> distance) : distance(distance) {
        std::vector<Node*> nodes;
        nodes.reserve(points.size());

        for (auto& point : points) {
            nodes.push_back(new Node(point));
        }

        this->root = build_tree(nodes, 0, points.size());
    }

    ~EnhancedMetricTree() {
        delete root;
    }

    std::vector<T> search(const T &target, double radius) {
        std::vector<T> results;
        std::vector<double> predecessors;
        search(this->root, results, target, radius, predecessors);
        return results;
    }

private:

    std::function<double(const T&, const T&)> distance;

    struct Node {
        T point;

        std::vector<double> predecessors;

        double innerRadius;
        double outerRadius;

        Node *right;
        Node *left;

        Node(T point) : point(point), innerRadius(0), outerRadius(0), left(nullptr), right(nullptr) {}

        ~Node() {
            delete left;
            delete right;
        }
    } *root;

    Node *build_tree(std::vector<EnhancedMetricTree<T>::Node *> points, std::size_t low, std::size_t high) {
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

        nth_element(first, median, last, [](const Node* n1, const Node* n2) {
            return n1->innerRadius < n2->innerRadius;
        });

        points[low]->outerRadius = (*median)->innerRadius;

        const auto pointOnInnerRadius = max_element(first, median, [](const Node* n1, const Node* n2) {
            return n1->innerRadius < n2->innerRadius;
        });

        points[low]->innerRadius = (*pointOnInnerRadius)->innerRadius;

        points[low]->left  = build_tree(points, low + 1, mid);
        points[low]->right = build_tree(points, mid, high);

        return points[low];
    }

    void search(EnhancedMetricTree<T>::Node *root, std::vector<T> &result, const T &target, double radius,
                std::vector<double> predecessors) {
        if (root == nullptr) {
            return;
        }

        const auto dist = this->distance(root->point, target);

        if (dist <= radius) {
            result.push_back(root->point);
        }

        if (!triangle_prune(predecessors, root->predecessors, dist)) {
            predecessors.push_back(dist);
            if (dist + radius >= root->outerRadius) {
                search(root->right, result, target, radius, predecessors);
            }

            if (dist - radius <= root->innerRadius) {
                search(root->left, result, target, radius, predecessors);
            }
            predecessors.pop_back();
        }
    }
};

#endif //THESIS_ENHANCEDMETRICTRREE_H
