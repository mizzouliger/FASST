//
// Created by Seth Wiesman on 3/8/16.
//

#include "KdTree.hpp"
#include "DistanceMetrics.hpp"

KdTree::KdTree(std::vector<std::vector<double>>::iterator begin, std::vector<std::vector<double>>::iterator end, int dim) {
    if (begin + 1 == end) {
        this->tag = Leaf;
        this->point = *begin;
        this->left = nullptr;
        this->right = nullptr;
    } else {

        const auto median = begin + (end - begin) / 2;

        std::nth_element(begin, median, end, [dim](const auto left, const auto right) {
            return left[dim] < right[dim];
        });

        this->tag = Internal;
        this->line = (*median)[dim];

        const auto next = (dim + 1) % begin->size();

        this->left = std::make_shared<KdTree>(begin, median + 1, next);
        this->right = std::make_shared<KdTree>(median + 1, end, next);
    }
}

std::vector<std::vector<double>> KdTree::search(std::vector<double> target, double radius) {
    std::vector<std::vector<double>> results;
    search(target, radius, results, 0);
    return results;
}

void KdTree::search(std::vector<double> target, double radius, std::vector<std::vector<double>> &results, int dim) {
    if (this->tag == Leaf) {
        if (Thesis::Metrics::norm2(target, this->point) <= radius) {
            results.push_back(this->point);
        }
    } else {
        if (target[dim] - radius <= this->line && this->line <= target[dim] + radius) {
            const auto next = (dim + 1) % target.size();
            this->left->search(target, radius, results, next);
            this->right->search(target, radius, results, next);
        } else if (this->line <= target[dim] - radius) {
            const auto next = (dim + 1) % target.size();
            this->left->search(target, radius, results, next);
        } else {
            const auto next = (dim + 1) % target.size();
            this->left->search(target, radius, results, next);
        }
    }
}
