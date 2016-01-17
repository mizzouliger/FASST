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

struct result {
    std::vector<Point> result;
    int calls;
    int nodes_visited;
    double build_time;
    double search_time;
};

template<typename T>
struct result benchmark(std::vector<Point> points, const double radius);

std::vector<Point> read_points(std::string filename, std::size_t len);

std::vector<std::string> get_files(std::string directory);

void verify_results(struct result control, struct result variable);

void display_progress_bar(double progress);

double find_radius(std::vector<Point> points);

std::size_t dim = 0;

int main(int argc, char *argv[]) {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    std::cout << "Thesis -- Metric Tree Benchmarking Tests" << std::endl;
    std::cout << "Author: Seth Wiesman Date: " << buf << std::endl;
    std::cout << "----------------------------------------------" << std::endl;

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

    std::cout << "Number of iterations: " << iterations << std::endl;
    std::cout << "Step size: " << step_size << std::endl;


    const auto tree_tests = {
            benchmark<GatedTree<Point, Point::euclidean_distance>>,
            benchmark<BoundedTree<Point, Point::euclidean_distance>>,
            benchmark<FasstTree<Point, Point::euclidean_distance>>
    };

    std::cout << "Number of trees: " << tree_tests.size() + 1 << std::endl;

    const auto files = get_files(indir);

    std::cout << "Number of files: " << files.size() << std::endl;
    std::cout << std::endl;

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

        for (auto i = step_size; i <= iterations; i += step_size) {
            display_progress_bar(static_cast<double>(i) / static_cast<double>(iterations));

            auto points = read_points(indir + "/" + file, static_cast<std::size_t>(i));

            auto radius = find_radius(points);

            auto metric_tree_future = std::async(std::launch::async, [&points, radius]() {
                return benchmark<MetricTree<Point, Point::euclidean_distance>>(points, radius);
            });

            std::vector<decltype(metric_tree_future)> futures;
            futures.reserve(tree_tests.size());
            for (auto tree_test : tree_tests) {
                futures.push_back(std::async(std::launch::async, [tree_test, &points, radius]() {
                    return tree_test(points, radius);
                }));
            }

            std::vector<result> results = {metric_tree_future.get()};

            for (auto& future : futures) {
                auto test_result = future.get();
                verify_results(results[0], test_result);
                results.push_back(test_result);
            }

            distance_file << i << "\t" << results[0].result.size() << "\t";
            node_visited_file << i << "\t";
            build_file << i << "\t";
            search_file << i << "\t";

            for (auto result : results) {
                distance_file << result.calls << "\t";
                node_visited_file << result.nodes_visited << "\t";
                build_file << result.build_time << "\t";
                search_file << result.search_time << "\t";
            }

            distance_file << "\n";
            node_visited_file << "\n";
            build_file << "\n";
            search_file << "\n";
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

std::vector<Point> read_points(std::string filename, std::size_t len) {
    std::vector<Point> points;
    points.reserve(len);

    std::ifstream file(filename);
    std::string input;

    for (std::size_t i = 0; i < len; i++) {
        std::getline(file, input);
        std::istringstream ss(input);

        std::vector<double> elements;
        std::string token;
        while (getline(ss, token, ',')) {
            elements.push_back(stod(token));
        }

        points.push_back(Point(elements));
    }

    dim = points[0].size();

    file.close();
    return points;
}

template<typename T>
struct result benchmark(std::vector<Point> points, const double radius) {
    static_assert(std::is_base_of<ISearchTree<Point, Point::euclidean_distance>, T>::value,
                  "T must derive from ISearchTree");

    const auto start_build = std::clock();
    std::unique_ptr<ISearchTree<Point, Point::euclidean_distance>> tree(new T(points));
    const auto end_build = std::clock();

    const auto start_search = std::clock();
    auto results = tree->search(Point::origin(dim), radius);
    const auto end_search = std::clock();

    for (auto &point : results) {
        const auto distance = Point::euclidean_distance(point, Point::origin(dim));
        assert(distance <= radius);
    }

    return {
            results,
            tree->getCalls(),
            tree->getNodesVisited(),
            (end_build - start_build) / (double) (CLOCKS_PER_SEC / 1000),
            (end_search - start_search) / (double) (CLOCKS_PER_SEC / 1000)
    };
}

void verify_results(struct result control, struct result variable) {

    assert(control.result.size() == variable.result.size());

    for (auto &point : control.result) {
        const auto location = std::find(variable.result.begin(), variable.result.end(), point);
        assert(location != variable.result.end());
    }
}

double find_radius(std::vector<Point> points) {
    MetricTree<Point, Point::euclidean_distance> tree(points);

    double radius = 10.0;
    auto result = tree.search(Point::origin(dim), radius);

    while (result.size() < 4 || result.size() > 6) {
        if (result.size() < 4) {
            radius = radius + (radius / 2);
        } else {
            radius = radius - (radius / 2);
        }

        result = tree.search(Point::origin(dim), radius);
    }

    return radius;
}
