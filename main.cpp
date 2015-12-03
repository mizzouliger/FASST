#include <iostream>
#include <assert.h>
#include <random>
#include <fstream>
#include <sstream>

#include "Point.h"
#include "MetricTree.h"
#include "EnhancedMetricTree.h"

int calls;

double distance_function(const Point& p1,  const Point& p2) {
    calls += 1;
    return Point::euclidean_distance(p1, p2);
}

template<typename T>
std::pair<unsigned long, int> benchmark(std::vector<Point> points, const double radius) {
    static_assert(std::is_base_of<IMetricTree<Point, distance_function>, T>::value, "T must derive from IMetricTree");

    std::unique_ptr<IMetricTree<Point, distance_function>> tree(new T(points));
    calls = 0;
    auto results = tree->search(Point({10, 10}), radius);
    return std::make_pair(results.size(), calls);
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
            "/Users/sethwiesman/ClionProjects/Thesis/normal-2d.txt",
            "/Users/sethwiesman/ClionProjects/Thesis/uniform-2d.txt"
    };


    //auto points = read_points(files[0], 10);

    //EnhancedMetricTree<Point, distance_function> tree(points);
    //tree.print();

    for (auto file : files) {
        std::vector<int> metric_calls;
        std::vector<int> enhanced_calls;

        std::cout << file << std::endl;
        for (std::size_t i = 1000; i < 100000; i += 1000) {
            show_progress(i / 100000.0);

            auto points = read_points(file, i);

            auto radius = 15.0;

            auto metric_bench   = benchmark<MetricTree<Point, distance_function>>(points, radius);
            auto enhanced_bench = benchmark<EnhancedMetricTree<Point, distance_function>>(points, radius);

            metric_calls.push_back(std::get<1>(metric_bench));
            enhanced_calls.push_back(std::get<1>(enhanced_bench));

            assert(std::get<0>(metric_bench) == std::get<0>(enhanced_bench));
            std::cout << std::get<1>(metric_bench) << " " << std::get<1>(enhanced_bench) << std::endl;
        }

        /*std::reverse(metric_calls.begin(), metric_calls.end());
        std::reverse(enhanced_calls.begin(), enhanced_calls.end());
        std::ofstream out(file + ".out.txt");
        auto num = 1000;
        for (auto i = 0; i < metric_calls.size(); i++) {
            out << num << " " << metric_calls[i] << " " << enhanced_calls[i] << std::endl;
            num += 1000;
        }
        out.close();*/
    }
    return 0;
}