#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>
#include <dirent.h>
#include <future>

#include "ISearchTree.hpp"
#include "DistanceMetrics.hpp"
#include "MetricTree.hpp"
#include "BoundedTree.hpp"
#include "FasstTree.hpp"

#include "ezOptionParser.hpp"

using namespace Thesis;

enum class Metric {
    Norm2,
    Edit,
    Hamming,
};

Metric stringToMetric(const std::string str) {
    if ("norm2" == str) {
        return Metric::Norm2;
    }

    if ("edit" == str) {
        return Metric::Edit;
    }

	if ("hamming" == str) {
		return Metric::Hamming;
	}
    std::cout << "invalid metric " << str << std::endl;
    exit(0);
}

struct benchmark {
    int calls;
    int nodes_visited;
    double build_time;
    double search_time;
};

template<typename T, typename U>
std::pair<std::vector<U>, benchmark> run_benchmark(std::vector<U> points, const U target, const double radius) {

    const auto start_build = std::clock();
    std::unique_ptr<T> tree(new T(points));
    const auto end_build = std::clock();

    const auto start_search = std::clock();
    auto result = tree->search(target, radius);
    const auto end_search = std::clock();

    return std::make_pair(result, benchmark {
            tree->getCalls(),
            tree->getNodesVisited(),
            (end_build - start_build) / (double) (CLOCKS_PER_SEC / 1000),
            (end_search - start_search) / (double) (CLOCKS_PER_SEC / 1000)
    });
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

std::vector<std::string> read_lines(std::string filename) {
    std::vector<std::string> words;

    std::ifstream file(filename);
    std::string input;

    while (std::getline(file, input)) {
        words.push_back(input);
    }

    file.close();
    return words;
}

std::vector<std::vector<double>> read_points(std::string filename) {
    std::vector<std::vector<double>> points;

    std::ifstream file(filename);
    std::string input;

    while(std::getline(file, input)) {
        std::istringstream ss(input);
        std::vector<double> elements;
        std::string token;
        while (getline(ss, token, ',')) {
            elements.push_back(stod(token));
        }

        points.push_back(elements);
    }

    file.close();
    return points;
}

std::vector<int> read_ints(std::string filename) {
    std::vector<int> nums;

    std::ifstream file(filename);
    std::string input;

    while (std::getline(file, input)) {
        nums.push_back(atoi(input.c_str()));
    }

    file.close();
    return nums;
}

template<typename T, double(*distance)(const T&, const T&)>
void verify_results(std::vector<T> control, std::vector<T> variable, T target, double radius) {

    assert(control.size() == variable.size());

    for (auto &point : variable) {
        const auto dist = distance(point, target);
        assert(dist <= radius);
    }

    for (auto &point : control) {
        const auto location = std::find(variable.begin(), variable.end(), point);
        assert(location != variable.end());
    }
}

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
std::vector<std::vector<benchmark>>
run_tests(std::vector<T> &point_set, T target, long step, long iterations, double itrRadius) {
    const auto benchmarks = {
            run_benchmark<BoundedTree<T, distance>, T>,
            run_benchmark<FasstTree<T, distance>, T>
    };

    std::vector<std::vector<benchmark>> final_results;
    final_results.reserve((unsigned long) (iterations + 1));

    for (auto i = step; i <= iterations; i += step) {
        display_progress_bar(static_cast<double>(i) / static_cast<double>(iterations));

        std::vector<T> points(point_set.begin(), itrRadius == 0.0 ? point_set.begin() + i : point_set.end());
        std::random_shuffle(points.begin(), points.end());

        double radius;
        if (itrRadius == 0.0) {
            radius = find_radius<T, distance>(points, target);
        } else {
            radius = itrRadius;
            itrRadius += 1;
        }

        auto metric_tree_future = std::async(std::launch::async, [&points, &target, radius]() {
            return run_benchmark<MetricTree<T, distance>, T>(points, target, radius);
        });

        std::vector<decltype(metric_tree_future)> futures;
        futures.reserve(benchmarks.size());
        for (auto tree_bench : benchmarks) {
            futures.push_back(std::async(std::launch::async, [tree_bench, &points, &target, radius]() {
                return tree_bench(points, target, radius);
            }));
        }

        std::vector<T> control;
        benchmark metricBench = {0, 0, 0, 0};
        std::tie(control, metricBench) = metric_tree_future.get();
        std::vector<benchmark> results = {metricBench};

        for (auto& future : futures) {
            std::vector<T> variable;
            benchmark bench = {0, 0, 0, 0};
            std::tie(variable, bench) = future.get();
            verify_results<T, distance>(control, variable, target, radius);
            results.push_back(bench);
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

int main(int argc, const char *argv[]) {
    ez::ezOptionParser opt;

    opt.add("", 1, 1, 0, "Relative path to the directory containing the input files", "--input");
    opt.add("", 1, 1, 0, "Relative path to the directory were output files should be stored", "--output");
    opt.add("", 1, 1, 0, "Metric function to use", "--metric");
    opt.add("10", 0, 1, 0, "Step Size", "--step");
    opt.add("100", 0, 1, 0, "Number of iterations", "--itr");

    opt.parse(argc, argv);

    std::string indir;
    std::string outdir;
    std::string metricString;
    long iterations;
    long step_size;

    opt.get("--input")->getString(indir);
    opt.get("--output")->getString(outdir);
    opt.get("--metric")->getString(metricString);
    opt.get("--step")->getLong(step_size);
    opt.get("--itr")->getLong(iterations);

    auto files = get_files(indir);

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

        std::vector<std::vector<benchmark>> bench_set;
        switch (stringToMetric(metricString)) {
            case Metric::Norm2: {
                auto points = read_points(indir + "/" + file);
                auto origin = std::vector<double>(points[0].size(), 0.0);
                bench_set = run_tests<std::vector<double>, Metrics::norm2>(points, origin, step_size, iterations, 0);
            }
                break;

            case Metric::Edit: {
                auto words = read_lines(indir + "/" + file);
                auto target = "hello";
                bench_set = run_tests<std::string, Metrics::editDistance>(words, target, step_size, iterations, 1);
            }
                break;

            case Metric::Hamming:{
                auto nums = read_ints(indir + "/" + file);
                int target = 0;
                bench_set = run_tests<int, Metrics::hammingDistance>(nums, target, step_size, iterations, 1);
            }
                break;
        }

        int i = 1;
        for (auto& benchmarks : bench_set) {
            const auto size = step_size * i++;
            distance_file       << size << "\t";
            node_visited_file   << size << "\t";
            build_file          << size << "\t";
            search_file         << size << "\t";

            for (auto& bench : benchmarks) {
                distance_file       << bench.calls          << "\t";
                node_visited_file   << bench.nodes_visited  << "\t";
                build_file          << bench.build_time     << "\t";
                search_file         << bench.search_time    << "\t";
            }

            distance_file       << std::endl;
            node_visited_file   << std::endl;
            build_file          << std::endl;
            search_file         << std::endl;
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

