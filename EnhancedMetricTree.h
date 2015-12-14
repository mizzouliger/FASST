#ifndef THESIS_ENHANCEDMETRICTRREE_H
#define THESIS_ENHANCEDMETRICTRREE_H

#include <vector>
#include <functional>
#include <cmath>

#include "IMetricTree.h"

template<
        typename T,
        double(*distance)(const T&, const T&)
>
class EnhancedMetricTree : public IMetricTree<T, distance> {
public:
    EnhancedMetricTree(std::vector<T> points) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto& point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }

        std::vector<T> predecessors;

        this->root     = build_tree(nodes.begin(), nodes.end());
    }

    int getCalls() const {
        return this->calls;
    }

    std::vector<T> search(const T& target, double radius) const {
        std::vector<T> inRange;
        this->calls = 0;

        search(root, inRange, target, radius, {infinity});
        return inRange;
    }

private:
    mutable int calls;

    struct Node {
        T point;

        double innerRadius;
        double outerRadius;

        std::vector<double> parent_distance;

        std::shared_ptr<Node> left;
        std::shared_ptr<Node> right;

        Node(T point) : point(point), innerRadius(0), outerRadius(infinity) {
            this->parent_distance.push_back(infinity);
        }

        bool triangle_unknown(double side1, double side2) const {
            return side1 == infinity || side2 == infinity;
        }

        double min_triangle(std::vector<double> ancestor_distances) const {
            double max = 0;
            for (auto i = 0; i < this->parent_distance.size(); i++) {
                if (!triangle_unknown(this->parent_distance[i], ancestor_distances[i])) {
                    const auto dist = std::fabs(this->parent_distance[i] - ancestor_distances[i]);

                    if (max < dist) {
                        max = dist;
                    }
                }
            }

            return max;
        }

        double max_triangle(std::vector<double> ancestor_distances) const {
            double min = infinity;
            for (auto i = 0; i < this->parent_distance.size(); i++) {
                if (!triangle_unknown(this->parent_distance[i], ancestor_distances[i])) {
                    const auto dist = std::ceil(this->parent_distance[i] + ancestor_distances[i]);

                    if (dist < min) {
                        min = dist;
                    }
                }
            }

            return min;
        }
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
            (*itr)->innerRadius     = distance((*low)->point, (*itr)->point);
            (*itr)->parent_distance.push_back((*itr)->innerRadius);
        }

        const auto median = low + (high - low) / 2;

        std::nth_element(low + 1, median, high, [](const auto n1, const auto n2) {
            return n1->innerRadius < n2->innerRadius;
        });

        (*low)->outerRadius = (*median)->innerRadius;

        const auto pointOnInnerRadius = std::max_element(low, median, [](const auto n1, const auto n2) {
            return n1->innerRadius < n2->innerRadius;
        });

        (*low)->innerRadius = (*pointOnInnerRadius)->innerRadius;

        (*low)->left  = build_tree(low + 1, median);
        (*low)->right = build_tree(median, high);

        return *low;
    }

    void search(std::shared_ptr<Node> node, std::vector<T> &inRange, const T& target, double radius, std::vector<double> last) const {
        if (node == nullptr) {
            return;
        }

        const auto minDistance = node->min_triangle(last);
        const auto maxDistance = node->max_triangle(last);

        //This distance calculation is just for running the asserts and testing
        //It is not used in any logic and so it is not counted towards distance
        //calls
        const auto d = distance(target, node->point);

        assert(minDistance <= d);
        assert(d <= maxDistance);

        if (radius < minDistance || maxDistance <= radius) {

            if (maxDistance <= radius) {
                inRange.push_back(node->point);
            }

            last.push_back(infinity);
            if (minDistance - radius <= node->innerRadius) {
                search(node->left, inRange, target, radius, last);
            }

            if (maxDistance + radius >= node->outerRadius) {
                search(node->right, inRange, target, radius, last);
            }
            
        } else {
            const auto dist = distance(node->point, target);
            this->calls++;

            last.push_back(dist);

            if (dist <= radius) {
                inRange.push_back(node->point);
            }

            if (dist - radius <= node->innerRadius) {
                search(node->left, inRange, target, radius, last);
            }

            if (dist + radius >= node->outerRadius) {
                search(node->right, inRange, target, radius, last);
            }
        }
    }
};


#endif //THESIS_ENHANCEDMETRICTRREE_H
