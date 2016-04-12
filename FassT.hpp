//
// Created by Seth Wiesman on 12/22/15.
//

#ifndef THESIS_BOUNDEDTREE_H
#define THESIS_BOUNDEDTREE_H

#include <queue>
#include <cmath>

#include "ISearchTree.hpp"

namespace Thesis {

    namespace FassTreeBench {
        int distanceCalls;
        int nodesVisted;
    };

    template<typename T, double (*distance)(const T &, const T &)>
    class FassT {

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

            void collect(std::vector<T>& inRange);
            void search(const T &target, double radius, std::vector<double> &pivots, std::vector<T> &inRange);

            bool collectLeft(std::vector<double>& pivots, double radius);
            bool goLeft(std::vector<double> &pivots, double radius);

            bool collectRight(std::vector<double>& pivots, double radius);
            bool goRight(std::vector<double> &pivots, double radius);
        };

        using node_itr = typename std::vector<std::shared_ptr<typename FassT<T, distance>::Node>>::iterator;

        std::shared_ptr<Node> root;

        std::shared_ptr<Node> buildTree(const node_itr begin, const node_itr high, int depth) const;

    public:
        FassT(std::vector<T> points);

        std::vector<T> search(const T &target, const double radius) const;

        int getCalls() const;

        int getNodesVisited() const;
    };

    template<typename T, double (*distance)(const T &, const T &)>
    FassT<T, distance>::FassT(std::vector<T> points) {
        std::vector<std::shared_ptr<Node>> nodes;
        nodes.reserve(points.size());

        for (auto &point : points) {
            nodes.push_back(std::make_shared<Node>(point));
        }

        this->root = buildTree(nodes.begin(), nodes.end(), 0);
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::vector<T> FassT<T, distance>::search(const T &target, double radius) const {
        std::vector<T> inRange;
        std::vector<double> pivots;

        FassTreeBench::distanceCalls = 0;
        FassTreeBench::nodesVisted = 0;
        if (this->root != nullptr) {
            this->root->search(target, radius, pivots, inRange);
        }

        for (auto& p : inRange) {
            assert(distance(target, p) <= radius);
        }

        return inRange;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    int FassT<T, distance>::getCalls() const {
        return FassTreeBench::distanceCalls;
    }

    template<typename T, double(*distance)(const T &, const T &)>
    int FassT<T, distance>::getNodesVisited() const {
        return FassTreeBench::nodesVisted;
    }

    template<typename T, double (*distance)(const T &, const T &)>
    std::shared_ptr<typename FassT<T, distance>::Node>
    FassT<T, distance>::buildTree(const node_itr begin, const node_itr end, int depth) const {
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

    template<typename T, double(*distance)(const T &, const T &)>
    void FassT<T, distance>::Node::search(const T &target, double radius, std::vector<double> &pivots, std::vector<T> &inRange) {
        FassTreeBench::nodesVisted++;
        FassTreeBench::distanceCalls++;

        const auto dist = distance(target, this->point);
        if (dist <= radius) {
            inRange.push_back(this->point);
        }

        pivots.push_back(dist);

        if (this->left != nullptr && this->collectLeft(pivots, radius)) {
            this->left->collect(inRange);
        } else if (this->left != nullptr && this->goLeft(pivots, radius)) {
            this->left->search(target, radius, pivots, inRange);
        }

        if (this->right != nullptr && this->collectRight(pivots, radius)) {
            this->right->collect(inRange);
        } else if (this->right != nullptr && this->goRight(pivots, radius)) {
            this->right->search(target, radius, pivots, inRange);
        }
    }

    template<typename T, double(*distance)(const T&, const T&)>
    bool FassT<T, distance>::Node::collectLeft(std::vector<double>& pivots, double radius) {
        for (auto i = 0; i < this->rightAnnuli.size(); i++) {
            if (pivots[i] + this->leftAnnuli[i].longRadius <= radius) {
                return true;
            }
        }
        return false;
    }

    template<typename T, double(*distance)(const T&, const T&)>
    bool FassT<T, distance>::Node::goLeft(std::vector<double> &pivots, double radius) {
        for (auto i = 0; i < this->pivots.size(); i++) {
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
    bool FassT<T, distance>::Node::collectRight(std::vector<double>& pivots, double radius) {
        for (auto i = 0; i < this->rightAnnuli.size(); i++) {
            if (pivots[i] + this->rightAnnuli[i].longRadius <= radius) {
                return true;
            }
        }
        return false;
    }

    template<typename T, double(*distance)(const T&, const T&)>
    bool FassT<T, distance>::Node::goRight(std::vector<double> &pivots, double radius) {
        for (auto i = 0; i < this->pivots.size(); i++) {
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

    template<typename T, double(*distance)(const T&, const T&)>
    void FassT<T, distance>::Node::collect(std::vector<T>& inRange) {
        FassTreeBench::nodesVisted++;
        inRange.push_back(this->point);
        if (this->left) {
            this->left->collect(inRange);
        }

        if (this->right) {
            this->right->collect(inRange);
        }
    }
}
#endif
