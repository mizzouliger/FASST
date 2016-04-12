#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <future>
#include <assert.h>
#include <dirent.h>

#include "DistanceMetrics.hpp"
#include "MetricTree.hpp"
#include "FassT.hpp"
#include "FassTGating.hpp"

#include "ezOptionParser.hpp"
#include "KdTree.hpp"

using namespace Thesis;

enum class Metric {
    Norm2,
    Edit,
    Hamming,
    Blosum,
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

    if ("blosum" == str) {
        return Metric::Blosum;
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
std::pair<std::vector<U>, benchmark> bench(std::vector<U> points, const U target, const double radius) {

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

template<typename T, typename U>
std::pair<std::vector<U>, benchmark> bench(T* tree, const U target, const double radius) {

    const auto start_search = std::clock();
    auto result = tree->search(target, radius);
    const auto end_search = std::clock();

    return std::make_pair(result, benchmark {
            tree->getCalls(),
            tree->getNodesVisited(),
            0,
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

std::vector<std::string> get_sequences(unsigned long num) {
    std::vector<std::string> sequences;
    sequences.reserve(num);

    for (auto i = 0; i < num; i++) {
        auto len = rand() % 800 + 200;
        char* seq = (char*)malloc(sizeof(char) * len);

        for (auto j = 0; j < len; j++) {
            switch (rand() % 4) {
                case 0:
                    seq[j] = 'G';
                    break;
                case 1:
                    seq[j] = 'T';
                    break;
                case 2:
                    seq[j] = 'A';
                    break;
                default:
                    seq[j] = 'C';
                    break;
            }
        }

        sequences.push_back(std::string(seq));
    }

    return sequences;
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
    if (points.size() < 4) {
        return 10;
    }

    MetricTree<T, distance> tree(points);

    double radius = 10.0;
    auto result = tree.search(target, radius);

    for (auto i = 0; i < 10 && (result.size() < 5 || result.size() > 20); i++) {
        if (result.size() < 4) {
            radius = radius + (radius / 2);
        } else {
            radius = radius - (radius / 2);
        }

        result = tree.search(target, radius);
    }

    return radius;
}

std::vector<std::vector<benchmark>> 
norm2(std::vector<std::vector<double>>& points, std::vector<double> target, long step, long iterations) {
    std::vector<std::vector<benchmark>> finalResults;
    finalResults.reserve((unsigned long)iterations + 1);

    for (auto i = 0; i < iterations; i++) {
        display_progress_bar(static_cast<double>(i) / static_cast<double>(iterations));
        std::vector<std::vector<double>> sample(points.begin(), points.begin() + (step * (1 + i)));
        double radius = find_radius<std::vector<double>, Metrics::norm2>(sample, target);

        auto kdtree_future = std::async(std::launch::async, [&sample, &target, radius]() {
            return bench<KDTree, std::vector<double>>(sample, target, radius);
        });

        auto mtree_future = std::async(std::launch::async, [&sample, &target, radius]() {
            return bench<MetricTree<std::vector<double>, Metrics::norm2>, std::vector<double>>(sample, target, radius);
        });

        auto boundedtree_future = std::async(std::launch::async, [&sample, &target, radius]() {
            return bench<FassT<std::vector<double>, Metrics::norm2>, std::vector<double>>(sample, target, radius);
        });

        auto ftree_future = std::async(std::launch::async, [&sample, &target, radius]() {
            return bench<FassTGating<std::vector<double>, Metrics::norm2>, std::vector<double>>(sample, target, radius);
        });

        std::vector<std::vector<double>> control;
        std::vector<std::vector<double>> variable;

        benchmark metricBench = {0, 0, 0, 0};
        std::tie(control, metricBench) = mtree_future.get();
        std::vector<benchmark> results = {metricBench};

        benchmark kdBench = {0, 0, 0, 0};
        std::tie(variable, kdBench) = kdtree_future.get();
        //verify_results<std::vector<double>, Metrics::norm2>(control, variable, target, radius);
        results.push_back(kdBench);

        benchmark boundedBench = {0, 0, 0, 0};
        std::tie(variable, boundedBench) = boundedtree_future.get();
        //verify_results<std::vector<double>, Metrics::norm2>(control, variable, target, radius);
        results.push_back(boundedBench);

        benchmark fBench = {0, 0, 0, 0};
        std::tie(variable, fBench) = ftree_future.get();
        //verify_results<std::vector<double>, Metrics::norm2>(control, variable, target, radius);
        results.push_back(fBench);

        finalResults.push_back(results);
    }

    return finalResults;
}

template<typename T, double(*distance)(const T&, const T&)>
std::vector<std::vector<benchmark>>
run_tests(std::vector<T> &points, T target, long step, long iterations) {
    auto maxRadius = 0.0;
   	for (auto& point : points) {
    	auto dist = distance(target, point);
        if (dist > maxRadius) {
            maxRadius = dist;
        }
    }

    std::cout << "Max Radius: " << maxRadius << std::endl;

    std::vector<std::vector<benchmark>> final_results;

    MetricTree<T, distance>* metricTree   = new MetricTree<T, distance>(points);
    FassT<T, distance>*fassT              = new FassT<T, distance>(points);
    FassTGating<T, distance>* fassTGating = new FassTGating<T, distance>(points);

    for (auto i = step; i <= maxRadius; i += step) {
        display_progress_bar(i / maxRadius);

        auto metric_tree_future = std::async(std::launch::async, [metricTree, &target, i]() {
            return bench<MetricTree<T, distance>, T>(metricTree, target, i);
        });

        auto boundedtree_future = std::async(std::launch::async, [fassT, &target, i]() {
            return bench<FassT<T, distance>, T>(fassT, target, i);
        });

        auto ftree_future = std::async(std::launch::async, [fassTGating, &target, i]() {
            return bench<FassTGating<T, distance>, T>(fassTGating, target, i);
        });

        std::vector<T> ignore;

        benchmark metricBench = {0, 0, 0};
        std::tie(ignore, metricBench) = metric_tree_future.get();
        std::vector<benchmark> results = {metricBench};

        benchmark boundedBench = {0, 0, 0};
        std::tie(ignore, boundedBench) = boundedtree_future.get();
        results.push_back(boundedBench);

        benchmark fBench = {0, 0, 0};
        std::tie(ignore, fBench) = ftree_future.get();
        results.push_back(fBench);

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
    opt.add("10", 0, 1, 0, "Number of iterations", "--itr");

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
    std::cout << "Metric: " << metricString << std::endl;

    unsigned long file_count = 1;
    for (auto file : files) {

        std::cout << "File " << file_count++ << " / " << files.size() << " : " << file << std::endl;

        std::ofstream distance_file;
        distance_file.open(outdir + "/distance_calls/" + file + ".csv");

        std::ofstream node_visited_file;
        node_visited_file.open(outdir + "/node_visited/" + file + ".csv");

        std::ofstream build_file;
        build_file.open(outdir + "/build_time/" + file + ".csv");

        std::ofstream search_file;
        search_file.open(outdir + "/search_time/" + file + ".csv");

        std::vector<std::vector<benchmark>> bench_set;
        auto metric = stringToMetric(metricString);
        switch (metric) {
            case Metric::Norm2: {
                auto points = read_points(indir + "/" + file);
                auto origin = std::vector<double>(points[0].size(), 0.0);
                bench_set = norm2(points, origin, step_size, iterations);
            }
                break;

            case Metric::Edit: {
                auto words = read_lines(indir + "/" + file);
                auto target = "hello";
                bench_set = run_tests<std::string, Metrics::editDistance>(words, target, step_size, iterations);
            }
                break;

            case Metric::Hamming:{
                auto nums = read_ints(indir + "/" + file);
                int target = 0;
                bench_set = run_tests<int, Metrics::hammingDistance>(nums, target, step_size, iterations);
            }
                break;
            case Metric::Blosum: {
                auto sequences = get_sequences(100000);
                auto target = sequences[0];
                bench_set = run_tests<std::string, Metrics::blosum>(sequences, target, step_size, iterations);
            }
                break;
        }

        const auto header = metric == Metric::Norm2
                            ? "#Points,Metric Tree,KD Tree,FassT,FassT w/ Gating,"
                            : "Radius,Metric Tree,FassT,FassT w/ Gating,";

        distance_file       << header << std::endl;
        node_visited_file   << header << std::endl;
        build_file          << header << std::endl;
        search_file         << header << std::endl;

        int i = 1;
        for (auto& benchmarks : bench_set) {
			const auto size = step_size * i++;
            distance_file       << size << ",";
            node_visited_file   << size << ",";
            build_file          << size << ",";
            search_file         << size << ",";

            for (auto& bench : benchmarks) {
                distance_file       << bench.calls          << ",";
                node_visited_file   << bench.nodes_visited  << ",";
                build_file          << bench.build_time     << ",";
                search_file         << bench.search_time    << ",";
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

