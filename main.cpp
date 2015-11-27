#include <iostream>
#include <vector>
#include <cmath>
#include "Point.h"
#include "MetricTree.h"
#include "EnhancedMetricTree.h"

using namespace std;

vector<Point> random_points(int num, int dim) {
    vector<Point> points;
    for (int i = 0; i < num; i++) {
        vector<double> elements;
        for (int j = 0; j < dim; j++) {
            elements.push_back(rand() % 100);
        }

        points.push_back(Point(elements));
    }

    return points;
}

unsigned long calls = 0;
double distance_calls(const Point& p1, const Point& p2) {
    calls++;
    return Point::euclidean_distance(p1, p2);
}

int main() {
    const Point origin({0, 0});

    for (auto i = 1; i < 2; i++) {
        const auto size = static_cast<int>(pow(10, i));
        auto points = random_points(size, 2);

        MetricTree<Point>         metric_tree(points, distance_calls);
        EnhancedMetricTree<Point> enhanced_metric_tree(points, distance_calls);


        calls = 0;
        auto results = metric_tree.search(origin, 50);

        for(auto& point : results) {
            cout << point.to_string() << endl;
        }

        cout << calls << endl;

        calls = 0;
        auto nearest = metric_tree.nearest_neighbor(origin);

        cout << endl << calls << endl << nearest.to_string() << endl;
    }
    return 0;
}