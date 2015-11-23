//
// Created by Seth Wiesman on 11/22/15.
//

#ifndef THESIS_METRICTREE_H
#define THESIS_METRICTREE_H

#include <vector>
#include <string>
#include <functional>
#include "Point.h"

namespace Thesis {

    class MetricTree {
    public:
        MetricTree(Point element);
        MetricTree(Point element, MetricTree* left, MetricTree* right);
        ~MetricTree();

        int height(void);
        int size(void);

        void postorder(int);
        std::string to_string(void);

        std::vector<Point> searchRadius(Point search, double radius, std::function<double(Point, Point)> distance);

        static MetricTree* BuildMetricTree(std::vector<Point> elements, std::function<double(Point, Point)> distance);
    private:
        Point element;

        double innerRadius;
        double outerRadius;

        MetricTree* left;
        MetricTree* right;

        std::string to_string(int depth);

        static MetricTree* constructTree(std::vector<MetricTree*>::iterator first,
                                         std::vector<MetricTree*>::iterator last,
                                         std::function<double(Point, Point)> distance);
    };
}


#endif //THESIS_METRICTREE_H
