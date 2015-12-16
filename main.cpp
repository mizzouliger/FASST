#include <iostream>
#include <assert.h>
#include <algorithm>
#include <fstream>
#include <sstream>

#include "Point.h"
#include "MetricTree.h"
#include "EnhancedMetricTree.h"

using namespace Spatial;

struct result {
    std::vector<Point> result;
    int calls;
};

template<typename T>
struct result benchmark(std::vector<Point> points, const double radius) {
    static_assert(std::is_base_of<IMetricTree<Point, Point::euclidean_distance>, T>::value, "T must derive from IMetricTree");

    std::unique_ptr<IMetricTree<Point, Point::euclidean_distance>> tree(new T(points));
    auto results = tree->search(Point({0, 0}), radius);

    for(auto& point : results) {
        assert(Point::euclidean_distance(point, Point({0, 0})) <= radius);
    }

    return {results, tree->getCalls()};
}

std::vector<Point> read_points(std::string filename, std::size_t len) {
    std::vector<Point> points;

    std::ifstream file(filename);
    std::string input;

    for(std::size_t i = 0; i < len; i++) {
        std::getline(file, input);
        std::istringstream ss(input);

        std::vector<double> elements;
        std::string token;
        while (getline(ss, token, ',')) {
            elements.push_back(stod(token));
        }

        points.push_back(Point(elements));
    }
    file.close();
    return points;
}
/*
void show_progress(double progress) {

        int barWidth = 70;

        std::cout << "[";
        int pos = barWidth * progress;
        for (int i = 0; i < barWidth; ++i) {
            if (i < pos) std::cout << "=";
            else if (i == pos) std::cout << ">";
            else std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();
}
*/

int main(int argc, char* argv[]) {
    std::vector<std::string> files = {
            "/Users/sethwiesman/ClionProjects/Thesis/normal-2d.txt"
    };

    for (auto file : files) {
        //std::cout << file << std::endl;

        //std::cout << "#\tmcalls\tcontrol" << std::endl;

        for (std::size_t i = 100; i <= 100000; i += 100) {
            //show_progress(i / 50000.0);

	        auto points1 = read_points(file, i);
            auto points2 = read_points(file, i);

            auto radius = 5.0;

            auto metric_bench   = benchmark<MetricTree<Point, Point::euclidean_distance>>        (points1, radius);
            auto enhanced_bench = benchmark<EnhancedMetricTree<Point, Point::euclidean_distance>>(points2, radius);

            std::cout << i << " " << metric_bench.calls << " " << enhanced_bench.calls << std::endl;

            assert(metric_bench.result.size()  == enhanced_bench.result.size());

            for(auto& point : metric_bench.result) {
                const auto location = std::find(enhanced_bench.result.begin(), enhanced_bench.result.end(), point);
                assert(location != enhanced_bench.result.end());
            }
        }
    }
    return 0;
}
