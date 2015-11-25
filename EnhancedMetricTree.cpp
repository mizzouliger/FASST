//
// Created by Seth Wiesman on 11/23/15.
//

#define ENHANCED_METRIC_TREE_CPP

#include <numeric>
#include <iostream>
#include "EnhancedMetricTree.h"

using namespace std;

template<typename T>
EnhancedMetricTree<T>::EnhancedMetricTree(std::vector<T> points, std::function<double(const T&, const T&)> distance) {
    vector<Node*> nodes;
    nodes.reserve(points.size());

    for (auto& point : points) {
        nodes.push_back(new Node(point));
    }

    this->distance = distance;
    this->root     = build_tree(nodes, 0, points.size());
}

template<typename T>
EnhancedMetricTree<T>::~EnhancedMetricTree() {
    delete root;
}

template<typename T>
std::vector<T> EnhancedMetricTree<T>::search(const T &target, double radius) {
    vector<T> results;
    deque<double> predecessors;
    search(this->root, results, target, radius, predecessors);
    return results;
}

template<typename T>
void EnhancedMetricTree<T>::search(EnhancedMetricTree<T>::Node *root, vector<T> &result, const T &target, double radius,
                                   deque<double> predecessors) {

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

template<typename T>
bool EnhancedMetricTree<T>::triangle_prune(std::deque<double> first, std::deque<double> second, double dist) {
    if (first.size() == 0) {
        return false;
    }
    for (auto i = 0; i < first.size(); i++) {

        const auto sum = first[i] + second[i];

        if (dist > sum) {
            return true;
        }
    }

    return false;
}

template<typename T>
EnhancedMetricTree<T>::Node::Node(T point)
        : point(point),
          innerRadius(0), outerRadius(0),
          left(nullptr),  right(nullptr) {}

template<typename T>
EnhancedMetricTree<T>::Node::~Node() {
    delete left;
    delete right;
}

template<typename T>
typename EnhancedMetricTree<T>::Node * EnhancedMetricTree<T>::build_tree(std::vector<EnhancedMetricTree<T>::Node *> points, unsigned long low, unsigned long high) {
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