#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>
#include <dirent.h>

#include "optionparser.h"

#include "Point.h"
#include "IMetricTree.h"
#include "MetricTree.h"
#include "GatedMetricTree.h"
#include "LooselyBoundedMetricTree.hpp"
#include "BoundedMetricTree.h"

using namespace Thesis;

struct result {
    std::vector<Point> result;
    int calls;
    double build_time;
    double search_time;
};

enum optionIndex {
    INDIR,
    OUTDIR,
    ITERATIONS,
    STEP
};

const option::Descriptor usage[] = {
        {INDIR,      0, "", "input-dir", option::Arg::Optional, "--input-dir=<directory with input files>"},
        {OUTDIR,     0, "", "out-dir",   option::Arg::Optional, "--out-dir=<directory where output files will be written>"},
        {ITERATIONS, 0, "", "itr",       option::Arg::Optional, "--itr=<number of test runs>"},
        {STEP,       0, "", "step",      option::Arg::Optional, "--step=<number of nodes to add to each consecutive test>"}
};

template<typename T>
struct result benchmark(std::vector<Point> points, const double radius);

std::vector<Point> read_points(std::string filename, std::size_t len);

std::vector<std::string> get_files(std::string directory);

void verify_results(struct result control, struct result variable) {}

void display_progress_bar(double progress);

const auto dim = 2;

int main(int argc, char *argv[]) {
    argc -= (argc > 0);
    argv += (argc > 0);
    option::Stats stats(usage, argc, argv);
    option::Option* options = new option::Option[stats.options_max];
    option::Option* buffer  = new option::Option[stats.buffer_max];
    option::Parser parse(usage, argc, argv, options, buffer);

    if (parse.error()) {
        return 1;
    }

    if (!options[INDIR] || !options[OUTDIR]) {
        return 2;
    }

    std::string indir = options[INDIR].arg;
    std::string outdir = options[OUTDIR].arg;

    long iterations = 100;
    if (options[ITERATIONS]) {
        iterations = std::stol(options[ITERATIONS].arg);
    }

    long step_size = 10;
    if (options[STEP]) {
        step_size = std::stol(options[STEP].arg);
    }

    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    std::cout << "Thesis -- Metric Tree Benchmarking Tests" << std::endl;
    std::cout << "Author: Seth Wiesman Date: " << buf << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "Number of iterations: " << iterations << std::endl;
    std::cout << "Step size: " << step_size << std::endl;


    const auto tree_tests = {
            benchmark<GatedMetricTree<Point, Point::euclidean_distance>>,
            benchmark<LooselyBoundedMetricTree<Point, Point::euclidean_distance>>,
            benchmark<BoundedMetricTree<Point, Point::euclidean_distance>>
    };

    std::cout << "Number of trees: " << tree_tests.size() + 1 << std::endl;

    const auto files = get_files(indir);

    std::cout << "Number of files: " << files.size() << std::endl;
    std::cout << std::endl;

    unsigned long file_count = 1;
    for (auto file : files) {

        std::cout << "File " << file_count << " / " << files.size() << " : " << file << std::endl;

        std::ofstream output_file;
        output_file.open(outdir + "/" + file);
        for (auto i = step_size; i <= iterations; i += step_size) {
            display_progress_bar(static_cast<double>(i) / static_cast<double>(iterations));

            auto points = read_points(indir + "/" + file, static_cast<std::size_t>(i));

            auto metric_tree_result = benchmark<MetricTree<Point, Point::euclidean_distance>>(points, 1.0);

            std::vector<result> results = {metric_tree_result};

            for (auto tree_test : tree_tests) {
                points = read_points(indir + "/" + file, static_cast<std::size_t>(i));
                auto test_result = tree_test(points, 1.0);
                verify_results(metric_tree_result, test_result);
                results.push_back(test_result);
            }

            output_file << i << "\t";
            for (auto result : results) {
                output_file << result.calls << "\t";
            }
            output_file << "\n";
        }

        output_file.close();

        std::cout << std::endl;
    }

    delete options;
    delete buffer;

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

    if ((dir = opendir (directory.c_str())) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            std::string filename = ent->d_name;
            if (filename == "." || filename == "..") {
                continue;
            }
            files.push_back(ent->d_name);
        }
        closedir (dir);
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
    file.close();
    return points;
}

template<typename T>
struct result benchmark(std::vector<Point> points, const double radius) {
    static_assert(std::is_base_of<IMetricTree<Point, Point::euclidean_distance>, T>::value,
                  "T must derive from IMetricTree");

    const auto start_build = std::clock();
    std::unique_ptr<IMetricTree<Point, Point::euclidean_distance>> tree(new T(points));
    const auto end_build = std::clock();

    const auto start_search = std::clock();
    auto results = tree->search(Point::origin(dim), radius);
    const auto end_search = std::clock();

    for (auto &point : results) {
        assert(Point::euclidean_distance(point, Point::origin(dim)) <= radius);
    }

    return {
            results,
            tree->getCalls(),
            (end_build - start_build) / (double) (CLOCKS_PER_SEC / 1000),
            (end_search - start_search) / (double) (CLOCKS_PER_SEC / 1000)
    };
}