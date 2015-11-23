#include <iostream>
#include <vector>
#include "Point.h"
#include "MetricTree.h"

using namespace std;
using namespace Thesis;

int main() {

    vector<Point> points;

    for (int i = 0; i < 200; i++) {
        vector<double> elements;
        for (int j = 0; j < 2; j++) {
            elements.push_back(rand() % 100);
        }

        points.push_back(Point(elements));
    }
    int numDistanceCalls = 0;
    MetricTree* root = MetricTree::BuildMetricTree(points, [&numDistanceCalls](Point p1, Point p2) mutable {
        numDistanceCalls++;
        return Point::EuclideanDistance(p1, p2);
    });
    std::cout << root->size() << " " << root->height()  << " " << numDistanceCalls << endl;

    Point origin = Point(vector<double>{0, 0});

    numDistanceCalls = 0;
    auto found = root->searchRadius(origin, 2450, [&numDistanceCalls](Point p1, Point p2) mutable {
        numDistanceCalls++;
        return Point::EuclideanDistance(p1, p2);
    });

    cout << numDistanceCalls << endl;

    delete(root);
    return 0;
}