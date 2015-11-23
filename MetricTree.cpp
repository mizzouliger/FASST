//
// Created by Seth Wiesman on 11/22/15.
//
#include <algorithm>
#include <functional>
#include <iostream>
#include <iomanip>
#include "MetricTree.h"

using namespace Thesis;

class MetricTreeWrapper {
public:
    MetricTree* operator()(Point point) {
        return new MetricTree(point);
    }
};

MetricTree::MetricTree(Point element) : MetricTree(element, nullptr, nullptr) { }

MetricTree::MetricTree(Point element, MetricTree *left, MetricTree *right) : element(element),
                                                                             left(left),
                                                                             right(right) {
    this->innerRadius = 0.0;
    this->outerRadius = 0.0;
}

int Thesis::MetricTree::height(void) {
    int left  = this->left  == nullptr ? 0 : this->left->height();
    int right = this->right == nullptr ? 0 : this->right->height();

    return 1 + std::max(left, right);
}

int Thesis::MetricTree::size(void) {
    int left  = this->left  == nullptr ? 0 : this->left->size();
    int right = this->right == nullptr ? 0 : this->right->size();

    return 1 + left + right;
}

void Thesis::MetricTree::postorder(int indent) {
    if (this->right != nullptr) {
        this->right->postorder(indent + 4);
    }

    if (indent) {
        std::cout << std::setw(indent) << ' ';
    }

    if (this->right) {
        std::cout << " /\n" << std::setw(indent) << ' ';
    }

    std::cout << this->element.to_string() << "\n ";

    if (this->left != nullptr) {
        std::cout << std::setw(indent) << ' ' << " \\\n";
        this->left->postorder(indent + 4);
    }
}

std::string Thesis::MetricTree::to_string(void) {
    return this->to_string(0);
}

std::string Thesis::MetricTree::to_string(int depth) {
    std::string padding = "";
    for (int i = 0; i < depth; i++) {
        padding += "\t";
    }

     std::string left = this->left == nullptr
                       ? "\t" + padding + "nullptr"
                       : this->left->to_string(depth + 1);

    std::string right = this->right == nullptr
                        ? "\t" + padding + "nullptr"
                        : this->right->to_string(depth + 1);

    return padding + "MetricTree(" + this->element.to_string() + ", "
           + std::to_string(this->innerRadius) + ", "
           + std::to_string(this->outerRadius) + "\n"
           + left  + "\n"
           + right + "\n"
           + padding + ")";
}

MetricTree::~MetricTree() {
    delete(this->left);
    delete(this->right);
}

std::vector<Point> MetricTree::searchRadius(Point search, double radius, std::function<double(Point, Point)> distance) {
    auto results = std::vector<Point>();

    const auto distToNode = distance(search, this->element);

    if (distToNode <= radius) {
        results.push_back(this->element);
    }

    if (this->left != nullptr && distToNode + radius >= this->outerRadius) {
        const auto leftResults = this->left->searchRadius(search, radius, distance);
        results.insert(results.end(), leftResults.begin(), leftResults.end());
    }

    if (this->right != nullptr && distToNode - radius <= this->innerRadius) {
        const auto rightResults = this->right->searchRadius(search, radius, distance);
        results.insert(results.end(), rightResults.begin(), rightResults.end());
    }

    return results;
}

MetricTree* MetricTree::constructTree(std::vector<MetricTree*>::iterator first,
                                      std::vector<MetricTree*>::iterator last,
                                      std::function<double(Point, Point)> distance) {
    if (first > last) {
        return nullptr;
    }

    if (first == last) {
        return *first;
    }

    const long numElements = std::distance(first, last);

    if (numElements == 1L) {
        auto root = *first;
        root->innerRadius = root->outerRadius = distance(root->element, (*last)->element);
        root->left = (*last);
        return root;
    }

    for (auto iter = first + 1; iter != last; iter++) {
        (*iter)->innerRadius = distance((*first)->element, (*iter)->element);
    }

    const auto median = first + numElements / 2;

    nth_element(first + 1, median, last, [](MetricTree* const a, MetricTree* const b) {
        return a->innerRadius < b->innerRadius;
    });

    auto elemOnInnerRadius = max_element(first + 1, median, [](MetricTree* const a, MetricTree* const b) {
       return a->innerRadius < b->innerRadius;
    });

    (*first)->innerRadius = (*elemOnInnerRadius)->innerRadius;

    (*first)->left  = constructTree(first + 1, median - 1, distance);
    (*first)->right = constructTree(median, last, distance);

    return *first;
}

MetricTree* MetricTree::BuildMetricTree(std::vector<Point> elements, std::function<double(Point, Point)> distance) {
    if (elements.empty()) {
        return nullptr;
    }

    std::vector<MetricTree*> nodes;
    nodes.reserve(elements.size());

    std::transform(elements.begin(), elements.end(), std::back_inserter(nodes), MetricTreeWrapper());

    return constructTree(nodes.begin(), nodes.end() - 1, distance);
}