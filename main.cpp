#include <iostream>
#include <assert.h>
#include <random>
#include <fstream>
#include <sstream>
#include <unordered_set>

#include "Point.h"
#include "MetricTree.h"
#include "EnhancedMetricTree.h"


double distance_function(const Point& p1,  const Point& p2) {
    return Point::euclidean_distance(p1, p2);
}

template<typename T>
std::pair<unsigned long, int> benchmark(std::vector<Point> points, const double radius) {
    static_assert(std::is_base_of<IMetricTree<Point, distance_function>, T>::value, "T must derive from IMetricTree");

    std::unique_ptr<IMetricTree<Point, distance_function>> tree(new T(points));
    auto results = tree->search(Point({0, 0}), radius);

    for(auto& point : results) {
        assert(distance_function(point, Point({0, 0})) <= radius);
    }

    return std::make_pair(results.size(), tree->getCalls());
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

int main(int argc, char* argv[]) {
    std::vector<std::string> files = {
            "/Users/sethwiesman/ClionProjects/Thesis/normal-2d.txt"
    };

    for (auto file : files) {
        std::cout << file << std::endl;

        //std::ofstream out(file + ".out.txt");

        for (std::size_t i = 5; i <= 50000; i += 1) {
            //show_progress(i / 50000.0);

	        auto points1 = read_points(file, i);
            auto points2 = points1;

            auto radius = 5.0;

            auto metric_bench   = benchmark<MetricTree<Point, distance_function>>(points1, radius);
            auto enhanced_bench = benchmark<EnhancedMetricTree<Point, distance_function>>(points2, radius);

            std::cout << i << " "
                      <<  std::get<1>(metric_bench)   << " " << std::get<1>(enhanced_bench) << " "
                      <<  std::get<0>(enhanced_bench) << " " << std::get<0>(metric_bench)   << std::endl;
        }

        //out.close();
    }
    return 0;
}
