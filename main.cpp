#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>
#include <dirent.h>
#include <future>

#include "Point.hpp"

#include "ISearchTree.hpp"
#include "MetricTree.hpp"
#include "BoundedTree.hpp"
#include "GatedTree.hpp"
#include "FasstTree.hpp"

using namespace Thesis;

struct benchmark {
    std::vector<Point> result;
    int calls;
    int nodes_visited;
    double build_time;
    double search_time;
};

template<typename T, typename U, double(*distance)(const U&, const U&)>
benchmark run_benchmark(std::vector<U> points, const U target, const double radius) {

    const auto start_build = std::clock();
    std::unique_ptr<T> tree(new T(points));
    const auto end_build = std::clock();

    const auto start_search = std::clock();
    auto results = tree->search(target, radius);
    const auto end_search = std::clock();

    for (auto &point : results) {
        const auto dist = distance(point, target);
        assert(dist <= radius);
    }

    return {
            results,
            tree->getCalls(),
            tree->getNodesVisited(),
            (end_build - start_build) / (double) (CLOCKS_PER_SEC / 1000),
            (end_search - start_search) / (double) (CLOCKS_PER_SEC / 1000)
    };
}

std::vector<Point> read_points(std::string filename);

std::vector<std::string> get_files(std::string directory);

void verify_results(struct benchmark control, struct benchmark variable);

void display_progress_bar(double progress);

template<typename T, double(*distance)(const T&, const T&)>
double find_radius(std::vector<T> points, T target) {
    MetricTree<T, distance> tree(points);

    double radius = 10.0;
    auto result = tree.search(target, radius);

    while (result.size() < 4 || result.size() > 6) {
        if (result.size() < 4) {
            radius = radius + (radius / 2);
        } else {
            radius = radius - (radius / 2);
        }

        result = tree.search(target, radius);
    }

    return radius;
}

template<typename T, double(*distance)(const T&, const T&)>
std::vector<std::vector<benchmark>> run_tests(std::vector<T> point_set, T target, long step_size, long iterations) {
    const auto benchmarks = {
            run_benchmark<BoundedTree<T, distance>, T, distance>,
            run_benchmark<FasstTree<T, distance>, T, distance>
    };

    std::cout << "Number of trees: " << benchmarks.size() + 1 << std::endl;

    std::vector<std::vector<benchmark>> final_results;
    final_results.reserve((unsigned long) (iterations + 1));

    for (auto i = step_size; i <= iterations; i++) {
        display_progress_bar(static_cast<double>(i) / static_cast<double>(iterations));

        std::vector<Point> points(point_set.begin(), point_set.begin() + i);
        auto radius = find_radius<T, distance>(points, target);

        auto metric_tree_future = std::async(std::launch::async, [&points, &target, radius]() {
            return run_benchmark<MetricTree<T, distance>, T, distance>(points, target, radius);
        });

        std::vector<decltype(metric_tree_future)> futures;
        futures.reserve(benchmarks.size());
        for (auto tree_bench : benchmarks) {
            futures.push_back(std::async(std::launch::async, [tree_bench, &points, &target, radius]() {
                return tree_bench(points, target, radius);
            }));
        }

        std::vector<benchmark> results = {metric_tree_future.get()};

        for (auto& future : futures) {
            auto test_result = future.get();
            verify_results(results[0], test_result);
            results.push_back(test_result);
        }

        final_results.push_back(results);
    }

    return final_results;
}

std::string now() {
    time_t _now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&_now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return std::string(buf);
}

int main(int argc, char *argv[]) {
    std::vector<std::string> options(argv + 1, argv + argc);

    if (options.size() < 2) {
        return 1;
    }

    std::string indir = options[0];
    std::string outdir = options[1];

    long iterations = 100;
    if (options.size() > 2) {
        iterations = std::stol(options[2]);
    }

    long step_size = 10;
    if (options.size() == 4) {
        step_size = std::stol(options[3]);
    }

    const auto files = get_files(indir);

    std::cout << "Thesis -- Metric Tree Benchmarking Tests" << std::endl;
    std::cout << "Author: Seth Wiesman Date: " << now() << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "Number of iterations: " << iterations << std::endl;
    std::cout << "Step size: " << step_size << std::endl;
    std::cout << "Number of files: " << files.size() << std::endl;

    unsigned long file_count = 1;
    for (auto file : files) {

        std::cout << "File " << file_count++ << " / " << files.size() << " : " << file << std::endl;

        std::ofstream distance_file;
        distance_file.open(outdir + "/distance_calls/" + file);

        std::ofstream node_visited_file;
        node_visited_file.open(outdir + "/node_visited/" + file);

        std::ofstream build_file;
        build_file.open(outdir + "/build_time/" + file);

        std::ofstream search_file;
        search_file.open(outdir + "/search_time/" + file);

        auto points = read_points(indir + "/" + file);
        auto origin = Point::origin(points[0].size());
        auto benchmarks_set = run_tests<Point, Point::euclidean_distance>(points, origin, step_size, iterations);

        for (auto& benchmarks : benchmarks_set) {
            for (auto& bench : benchmarks) {
                distance_file       << bench.calls          << "\t";
                node_visited_file   << bench.nodes_visited  << "\t";
                build_file          << bench.build_time     << "\t";
                search_file         << bench.search_time    << "\t";
            }

            distance_file       << "\n";
            node_visited_file   << "\n";
            build_file          << "\n";
            search_file         << "\n";

        }

        distance_file.close();
        node_visited_file.close();
        build_file.close();
        search_file.close();

        std::cout << std::endl << std::endl;
    }

    std::cout << "Benchmarking Complete" << std::endl;

    return 0;
}

void display_progress_bar(double progress) {

    int barWidth = 70;

    std::cout << "[";
    int pos = static_cast<int>(barWidth * progress);
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) {
            std::cout << "=";
        } else if (i == pos) {
            std::cout << ">";
        } else {
            std::cout << " ";
        }
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

std::vector<std::string> get_files(std::string directory) {
    DIR *dir;
    struct dirent *ent;

    std::vector<std::string> files;

    if ((dir = opendir(directory.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            std::string filename = ent->d_name;
            if (filename == "." || filename == "..") {
                continue;
            }
            files.push_back(ent->d_name);
        }
        closedir(dir);
    } else {
        throw std::domain_error("Unable to open directory");
    }

    return files;
}

std::vector<Point> read_points(std::string filename) {
    std::vector<Point> points;

    std::ifstream file(filename);
    std::string input;

    while(std::getline(file, input)) {
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


void verify_results(struct benchmark control, struct benchmark variable) {

    assert(control.result.size() == variable.result.size());

    for (auto &point : control.result) {
        const auto location = std::find(variable.result.begin(), variable.result.end(), point);
        assert(location != variable.result.end());
    }
}

