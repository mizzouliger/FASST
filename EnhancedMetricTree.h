//
// Created by Seth Wiesman on 11/23/15.
//

#ifndef THESIS_ENHANCEDMETRICTRREE_H
#define THESIS_ENHANCEDMETRICTRREE_H

#include <vector>
#include <deque>
#include <functional>

template<typename T>
class EnhancedMetricTree {
public:
    EnhancedMetricTree(std::vector<T> points, std::function<double(const T&, const T&)> distance);
    ~EnhancedMetricTree();

    std::vector<T> search(const T &target, double radius);
private:

    std::function<double(const T&, const T&)> distance;

    struct Node {
        T point;

        std::deque<double> predecessors;

        double innerRadius;
        double outerRadius;

        Node *right;
        Node *left;

        Node(T point);
        ~Node();
    } *root;

    Node *build_tree(std::vector<EnhancedMetricTree<T>::Node *> points, unsigned long low, unsigned long high);

    void search(EnhancedMetricTree<T>::Node *root, std::vector<T> &result, const T &target, double radius,
                std::deque<double> predecessors);

    static bool triangle_prune(std::deque<double> first, std::deque<double> second, double dist);
};

#ifndef ENHANCED_METRIC_TREE_CPP
#include "EnhancedMetricTree.cpp"
#endif

#endif //THESIS_ENHANCEDMETRICTRREE_H
