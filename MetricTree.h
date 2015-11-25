//
// Created by Seth Wiesman on 11/22/15.
//

#ifndef THESIS_METRICTREE_H
#define THESIS_METRICTREE_H

#include <functional>
#include <vector>
#include <queue>

template<typename T> class MetricTree {
public:
    MetricTree(std::vector<T> points, std::function<double(const T&, const T&)> distance);
    ~MetricTree();

    std::vector<T> search(const T& target, double radius);
private:
    struct Node {
        T point;

        double innerRadius;
        double outerRadius;

        Node *left;
        Node *right;

        Node(T point);
        ~Node();
    } *root;

    std::function<double(const T&, const T&)> distance;


    MetricTree<T>::Node* build_tree(std::vector<MetricTree<T>::Node *> points, unsigned long low, unsigned long high);
    void search(MetricTree<T>::Node* root, std::vector<T>& result, const T& target, double radius);
};

#ifndef METRIC_TREE_CPP
#include "MetricTree.cpp"
#endif

#endif //THESIS_METRICTREE_H
