//
// Created by Seth Wiesman on 11/22/15.
//
#define METRIC_TREE_CPP
#include "MetricTree.h"

using namespace std;

template<typename T>
MetricTree<T>::MetricTree(vector<T> points, function<double(const T&, const T&)> distance) {
    vector<Node*> nodes;
    nodes.reserve(points.size());

    for (auto& point : points)  {
        nodes.push_back(new Node(point));
    }

    this->distance = distance;
    this->root     = build_tree(nodes, 0, points.size());
}

template<typename T>
MetricTree<T>::~MetricTree() {
    delete root;
}

template<typename T>
vector<T> MetricTree<T>::search(const T& target, double radius) {
    vector<T> results;
    search(this->root, results, target, radius);
    return results;
}

template<typename T>
void MetricTree<T>::search(MetricTree<T>::Node* root, vector<T>& result, const T&target, double radius) {
    if (root == nullptr) {
        return;
    }

    const auto dist = this->distance(root->point, target);

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

template<typename T>
MetricTree<T>::Node::Node(T point): point(point), innerRadius(0), outerRadius(0), left(nullptr), right(nullptr) {}

template<typename T>
MetricTree<T>::Node::~Node() {
    delete left;
    delete right;
}

template<typename T>
typename MetricTree<T>::Node* MetricTree<T>::build_tree(vector<Node *> points, unsigned long low, unsigned long high) {
    if (high == low) {
        return nullptr;
    }

    if (high - low == 1) {
        return points[low];
    }

    for (auto i = low + 1; i < high; i++) {
        points[i]->innerRadius = this->distance(points[low]->point, points[i]->point);
    }

    const auto mid = (high + low) / 2;

    const auto first  = points.begin() + low + 1;
    const auto median = points.begin() + mid;
    const auto last   = points.begin() + high;

    nth_element(first, median, last, [](const Node* n1, const Node* n2){
        return n1->innerRadius < n2->innerRadius;
    });

    points[low]->outerRadius = (*median)->innerRadius;

    const auto pointOnInnerRadius = max_element(first, median, [](const Node* n1, const Node* n2){
        return n1->innerRadius < n2->innerRadius;
    });

    points[low]->innerRadius = (*pointOnInnerRadius)->innerRadius;

    points[low]->left  = build_tree(points, low + 1, mid);
    points[low]->right = build_tree(points, mid, high);

    return points[low];
}