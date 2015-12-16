//
// Created by Seth Wiesman on 11/22/15.
//

#ifndef THESIS_METRICTREE_H
#define THESIS_METRICTREE_H

#include <vector>
#include <functional>

#include "IMetricTree.h"
namespace Spatial {
    template<
            typename T,
            double(*distance)(const T &, const T &)
    >
    class MetricTree : public IMetricTree<T, distance> {
    public:
        MetricTree(std::vector<T> points) {
            std::vector<std::shared_ptr<Node>> nodes;
            nodes.reserve(points.size());

            for (auto &point : points) {
                nodes.push_back(std::make_shared<Node>(point));
            }
            this->root = build_tree(nodes.begin(), nodes.end());
        }

        int getCalls() const {
            return this->calls;
        }

        std::vector<T> search(const T &target, double radius) const {
            std::vector<T> inRange;
            this->calls = 0;
            search(root, inRange, target, radius);
            return inRange;
        }

    private:
        mutable int calls;

        struct Node {
            T point;

            double innerRadius;
            double outerRadius;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point), innerRadius(0), outerRadius(infinity) { }
        };

        std::shared_ptr<Node> root;

        std::shared_ptr<Node> build_tree(const typename std::vector<std::shared_ptr<Node>>::iterator low,
                                         const typename std::vector<std::shared_ptr<Node>>::iterator high) const {
            if (low == high) {
                return nullptr;
            }

            if ((high - low) == 1) {
                return *low;
            }

            for (auto itr = low + 1; itr != high; itr++) {
                (*itr)->innerRadius = distance((*low)->point, (*itr)->point);
            }

            const auto median = low + (high - low) / 2;

            std::nth_element(low + 1, median, high, [](const auto n1, const auto n2) {
                return n1->innerRadius < n2->innerRadius;
            });

            (*low)->outerRadius = (*median)->innerRadius;

            const auto pointOnInnerRadius = std::max_element(low + 1, median, [](const auto n1, const auto n2) {
                return n1->innerRadius < n2->innerRadius;
            });

            (*low)->innerRadius = (*pointOnInnerRadius)->innerRadius;

            (*low)->left = build_tree(low + 1, median);
            (*low)->right = build_tree(median, high);

            return *low;
        }

        void search(std::shared_ptr<Node> node, std::vector<T> &inRange, const T &target, double radius) const {
            if (node == nullptr) {
                return;
            }

            const auto dist = distance(node->point, target);
            this->calls++;

            if (dist <= radius) {
                inRange.push_back(node->point);
            }

            if (dist - radius <= node->innerRadius) {
                search(node->left, inRange, target, radius);
            }

            if (dist + radius >= node->outerRadius) {
                search(node->right, inRange, target, radius);
            }
        }
    };
}
#endif //THESIS_METRICTREE_H
