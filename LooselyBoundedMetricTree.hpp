//
// Created by Seth Wiesman on 1/3/16.
//

#ifndef THESIS_LOOSELYBOUNDEDMETRICTREE_HPP
#define THESIS_LOOSELYBOUNDEDMETRICTREE_HPP

#include "IMetricTree.hpp"
#include "TriangleUtils.hpp"

namespace Thesis {
    template<typename T, double (*distance)(const T &, const T &)>
    class LooselyBoundedMetricTree : public IMetricTree<T, distance> {
    public:
        LooselyBoundedMetricTree(std::vector<T> points) : calls(0) {
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

        std::vector<T> search(const T &target, const double radius) const {
            std::vector<T> inRange;
            this->calls = 0;
            search(root, inRange, target, radius);
            return inRange;
        }

    private:
        mutable int calls;

        struct Node {
            struct Distances {
                double nearest;
                double furthest;

                Distances() : nearest(0), furthest(0) { }
            };

            T point;

            Distances left_distances;
            Distances right_distances;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point) { }
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
                (*itr)->left_distances.nearest = distance((*low)->point, (*itr)->point);
            }

            const auto median = low + (high - low) / 2;

            std::nth_element(low + 1, median, high, [](const auto n1, const auto n2) {
                return n1->left_distances.nearest < n2->left_distances.nearest;
            });

            const auto left_min = std::min_element(low + 1, median, [](const auto n1, const auto n2) {
                return n1->left_distances.nearest < n2->left_distances.nearest;
            });

            const auto left_max = std::max_element(low + 1, median, [](const auto n1, const auto n2) {
                return n1->left_distances.nearest < n2->left_distances.nearest;
            });

            const auto right_max = std::max_element(median, high, [](const auto n1, const auto n2) {
                return n1->left_distances.nearest < n2->left_distances.nearest;
            });

            (*low)->left_distances.nearest = (*left_min)->left_distances.nearest;
            (*low)->left_distances.furthest = (*left_max)->left_distances.nearest;

            (*low)->right_distances.nearest = (*median)->left_distances.nearest;
            (*low)->right_distances.furthest = (*right_max)->left_distances.nearest;

            (*low)->left = build_tree(low + 1, median);
            (*low)->right = build_tree(median, high);

            return (*low);
        }

        void search(std::shared_ptr<Node> node, std::vector<T> &inRange, const T &target, const double radius) const {
            if (node == nullptr) {
                return;
            }

            const auto dist = distance(node->point, target);
            this->calls++;

            if (dist <= radius) {
                inRange.push_back(node->point);
            }

            if (dist + radius >= node->left_distances.nearest && dist - radius <= node->left_distances.furthest) {
                search(node->left, inRange, target, radius);
            }

            if (dist + radius >= node->right_distances.nearest && dist - radius <= node->right_distances.furthest) {
                search(node->right, inRange, target, radius);
            }
        }
    };
}


#endif //THESIS_LOOSELYBOUNDEDMETRICTREE_HPP
