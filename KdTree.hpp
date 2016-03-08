//
// Created by Seth Wiesman on 3/8/16.
//

#ifndef THESIS_KDTREE_HPP
#define THESIS_KDTREE_HPP


#include <memory>
#include <vector>

#include "ISearchTree.hpp"

class KdTree {
    std::shared_ptr<KdTree> left;
    std::shared_ptr<KdTree> right;

    enum {Internal, Leaf} tag;

    double line;
    std::vector<double> point;

    void search(std::vector<double> target, double radius, std::vector<std::vector<double>>& results, int dim);

public:
    KdTree(std::vector<std::vector<double>>::iterator begin, std::vector<std::vector<double>>::iterator end, int dim);
    std::vector<std::vector<double>> search(std::vector<double> target, double radius);
};


#endif //THESIS_KDTREE_HPP
