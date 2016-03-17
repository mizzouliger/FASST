//
// Created by Seth Wiesman on 1/7/16.
//

#ifndef THESIS_FASSTTREE_HPP
#define THESIS_FASSTTREE_HPP

#include <queue>
#include "Range.hpp"

namespace Thesis {

    template<typename T, double (*distance)(const T &, const T &)>
    class FasstTree {

        struct Annulus {
            double shortRadius;
            double longRadius;

            Annulus(double shortRadius, double longRadius) : shortRadius(shortRadius), longRadius(longRadius) { }
        };

        struct Node {
            T point;

            std::vector<Annulus> leftAnnuli;
            std::vector<Annulus> rightAnnuli;

            std::vector<double> pivots;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;

            Node(T point) : point(point) { }

            void search(const T &target, double radius, std::vector<double> &pivots, std::vector<T> &inRange, int &distanceCalls,
                                    int &nodesVisited);
            bool intersectsLeftAnnuli(std::vector<double> &pivots, double radius);
            bool intersectsRightAnnuli(std::vector<double> &pivots, double radius);
        };

        using node_itr = typename std::vector<std::shared_ptr<typename FasstTree<T, distance>::Node>>::iterator;

        std::shared_ptr<Node> root;

        std::shared_ptr<Node> buildTree(const node_itr begin, const node_itr high, int depth) const;

        static Range generateRange(std::vector<double> sideA, std::vector<double> sideB);

        mutable int distanceCalls;

        mutable int nodesVisited;

    public:
        FasstTree(std::vector<T> points);

        std::vector<T> search(const T &target, const double radius) const;

        int getCalls() const;

        int getNodesVisited() const;
    };

    template<typename T, double (*distance)(const T &, const T &)>
    FasstTree<T, distance>::FasstTree(std::vector<T> points) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }

        this->root = buildTree(nodes.begin(), nodes.end(), 0);
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::vector<T> FasstTree<T, distance>::search(const T &target, double radius) const {
        std::vector<T> inRange;
        std::vector<double> pivots;

        this->distanceCalls = 0;
        this->nodesVisited = 0;

        if (this->root != nullptr) {
            this->root->search(target, radius, pivots, inRange, this->distanceCalls, this->nodesVisited);
        }
        return inRange;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    int FasstTree<T, distance>::getCalls() const {
        return this->distanceCalls;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    int FasstTree<T, distance>::getNodesVisited() const {
        return this->nodesVisited;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::shared_ptr<typename FasstTree<T, distance>::Node>
    FasstTree<T, distance>::buildTree(const node_itr begin, const node_itr end, int depth) const {
        if (begin == end) {
            return nullptr;
        }

        if ((end - begin) == 1) {
            return *begin;
        }

        for (auto itr = begin + 1; itr != end; itr++) {
            auto dist = distance((*begin)->point, (*itr)->point);
            (*itr)->pivots.push_back(dist);
        }

        const auto median = begin + (end - begin) / 2;

        std::nth_element(begin + 1, median, end, [](const auto left, const auto right) {
            return left->pivots.back() < right->pivots.back();
        });

        (*begin)->left = buildTree(begin + 1, median, depth + 1);
        (*begin)->right = buildTree(median, end, depth + 1);

        for (auto i = 0; i < depth; i++) {

            if ((*begin)->left != nullptr) {
                auto nearest = std::min_element(begin + 1, median, [i](const auto left, const auto right) {
                    return left->pivots[i] < right->pivots[i];
                });

                auto furthest = std::max_element(begin + 1, median, [i](const auto left, const auto right) {
                    return left->pivots[i] < right->pivots[i];
                });

                (*begin)->leftAnnuli.push_back(Annulus((*nearest)->pivots[i], (*furthest)->pivots[i]));
            }

            if ((*begin)->right != nullptr) {
                auto nearest = std::min_element(median, end, [i](const auto left, const auto right) {
                    return left->pivots[i] < right->pivots[i];
                });

                auto furthest = std::max_element(median, end, [i](const auto left, const auto right) {
                    return left->pivots[i] < right->pivots[i];
                });

                (*begin)->rightAnnuli.push_back(Annulus((*nearest)->pivots[i], (*furthest)->pivots[i]));
            }
        }
        return (*begin);
    }

    template<typename T, double(*distance)(const T&, const T&)>
    Range FasstTree<T, distance>::generateRange(std::vector<double> sideA, std::vector<double> sideB) {
        const auto infinity = std::numeric_limits<double>::max();

        auto min = 0.0;
        auto max = infinity;

        for (auto i = 0; i < sideA.size(); i++) {
            if (sideA[i] == infinity || sideB[i] == infinity) {
                continue;
            }
            const auto minSide = std::fabs(sideA[i] - sideB[i]);
            const auto maxSide = sideA[i] + sideB[i];

            if (min < minSide) {
                min = minSide;
            }

            if (maxSide < max) {
                max = maxSide;
            }
        }
        return Range(min, max);
    }

    template<typename T, double(*distance)(const T &, const T &)>
    void FasstTree<T, distance>::Node::search(const T &target, double radius, std::vector<double> &pivots, std::vector<T> &inRange, int &distanceCalls,
                                                  int &nodesVisited) {

        nodesVisited++;

        auto range = generateRange(this->pivots, pivots);
        auto position = range.position(radius);

        double dist;
        switch (position) {
            case Range::Greater:
                inRange.push_back(this->point);
                pivots.push_back(std::numeric_limits<double>::max());
                break;
            case Range::Less:
                pivots.push_back(std::numeric_limits<double>::max());
                break;
            case Range::Inside:
                distanceCalls++;
                dist = distance(this->point, target);
                if (dist <= radius) {
                    inRange.push_back(this->point);
                }
                pivots.push_back(dist);
                break;
        }

        if (this->left != nullptr) {
            if (this->intersectsLeftAnnuli(pivots, radius)) {
                this->left->search(target, radius, pivots, inRange, distanceCalls, nodesVisited);
            }
        }

        if (this->right != nullptr) {
            if (this->intersectsRightAnnuli(pivots, radius)) {
                this->right->search(target, radius, pivots, inRange, distanceCalls, nodesVisited);
            }
        }
    }

    template<typename T, double(*distance)(const T&, const T&)>
    bool FasstTree<T, distance>::Node::intersectsLeftAnnuli(std::vector<double> &pivots, double radius) {
        for (auto i = 0; i < this->pivots.size(); i++) {
            if (pivots[i] == std::numeric_limits<double>::max()) {
                continue;
            }
            if (pivots[i] < this->leftAnnuli[i].shortRadius) {
                if (pivots[i] + radius < this->leftAnnuli[i].shortRadius) {
                    return false;
                }
            } else if (pivots[i] - radius > this->leftAnnuli[i].longRadius) {
                if (this->leftAnnuli[i].longRadius < pivots[i]) {
                        return false;
                }
            }
        }

        return true;
    }

    template<typename T, double(*distance)(const T&, const T&)>
    bool FasstTree<T, distance>::Node::intersectsRightAnnuli(std::vector<double> &pivots, double radius) {
        for (auto i = 0; i < this->pivots.size(); i++) {
            if (pivots[i] == std::numeric_limits<double>::max()) {
                continue;
            }
            if (pivots[i] < this->rightAnnuli[i].shortRadius) {
                if (pivots[i] + radius < this->rightAnnuli[i].shortRadius) {
                    return false;
                }
            } else if (pivots[i] - radius > this->rightAnnuli[i].longRadius) {
                if (this->rightAnnuli[i].longRadius < pivots[i]) {
                    return false;
                }
            }
        }

        return true;
    }
}

#endif
