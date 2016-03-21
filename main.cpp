#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <future>
#include <dirent.h>

#include "DistanceMetrics.hpp"
#include "MetricTree.hpp"
#include "BoundedTree.hpp"
#include "FasstTree.hpp"

using namespace Thesis;

struct benchmark {
    int calls;
    int nodes_visited;
    double search_time;
};

template<typename T, typename U> benchmark bench(std::vector<U> points, const U target, const double radius) {

    std::unique_ptr<T> tree(new T(points));

    const auto start_search = std::clock();
    tree->search(target, radius);
    const auto end_search = std::clock();

    return benchmark {
            tree->getCalls(),
            tree->getNodesVisited(),
            (end_search - start_search) / (double) (CLOCKS_PER_SEC / 1000)
    };
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
        throw std::domain_error("Unable to open directory: " + directory);
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

template<typename T, double(*distance)(const T&, const T&)>
std::vector<std::vector<benchmark>> run_tests(std::vector<T> &points, T target) {

    std::vector<std::vector<benchmark>> final_results;
    final_results.reserve((unsigned long) (355 / 5));

    std::vector<std::future<std::vector<benchmark>>> futures;
    futures.reserve(355 / 5);

    for (auto i = 5; i <= 355; i += 5) {
         auto iteration = std::async(std::launch::async, [&points, &target, i]() {
            auto metric_tree_future = std::async(std::launch::async, [&points, &target, i]() {
                return bench<MetricTree<T, distance>, T>(points, target, i);
            });

            auto boundedtree_future = std::async(std::launch::async, [&points, &target, i]() {
                return bench<BoundedTree<T, distance>, T>(points, target, i);
            });

            auto ftree_future = std::async(std::launch::async, [&points, &target, i]() {
                return bench<FasstTree<T, distance>, T>(points, target, i);
            });

            benchmark metricBench = metric_tree_future.get();
            std::vector<benchmark> results = {metricBench};

            benchmark boundedBench = boundedtree_future.get();
            results.push_back(boundedBench);

            benchmark fBench = ftree_future.get();
            results.push_back(fBench);
            return results;
        });

        futures.push_back(std::move(iteration));
    }

	int i = 1; 
    for(auto& future : futures) {
        display_progress_bar((i++ * 5) / 71);
		final_results.push_back(future.get());
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

    std::string indir 		= argv[1];
    std::string outdir 		= argv[2];
    std::string metricString 	= argv[3];

    auto files = get_files(indir);

    std::cout << "Thesis -- Metric Tree Benchmarking Tests" << std::endl;
    std::cout << "Author: Seth Wiesman Date: " << now() << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "Number of files: " << files.size() << std::endl;
    std::cout << "Metric: " << metricString << std::endl;

    unsigned long file_count = 1;
    for (auto file : files) {

        std::cout << "File " << file_count++ << " / " << files.size() << " : " << file << std::endl;

        auto sequences = read_lines(indir + "/" + file);
        auto target = sequences[0];
        auto bench_set = run_tests<std::string, Metrics::blosum>(sequences, target);

        std::ofstream distance_file;
        distance_file.open(outdir + "/distance_calls/" + file);

        std::ofstream node_visited_file;
        node_visited_file.open(outdir + "/node_visited/" + file);

        std::ofstream search_file;
        search_file.open(outdir + "/search_time/" + file);

        int i = 1;
        for (auto& benchmarks : bench_set) {
            const auto size = 5 * i++;
            distance_file       << size << "\t";
            node_visited_file   << size << "\t";
            search_file         << size << "\t";

            for (auto& bench : benchmarks) {
                distance_file       << bench.calls          << "\t";
                node_visited_file   << bench.nodes_visited  << "\t";
                search_file         << bench.search_time    << "\t";
            }

            distance_file       << std::endl;
            node_visited_file   << std::endl;
            search_file         << std::endl;
        }

        distance_file.close();
        node_visited_file.close();
        search_file.close();

        std::cout << std::endl << std::endl;
    }

    std::cout << "Benchmarking Complete" << std::endl;

    return 0;
}

