#include <string>
#include "optionparser.h"

enum optionIndex {
    INDIR,
    OUTDIR,
    ITERATIONS,
    STEP
};

const option::Descriptor usage[] = {
        {INDIR,      0, "", "input-dir", option::Arg::Optional, "--input-dir=<directory with input files>"},
        {OUTDIR,     0, "", "out-dir",   option::Arg::Optional, "--out-dir=<directory where outputfiles will be written>"},
        {ITERATIONS, 0, "", "itr",       option::Arg::Optional, "--itr=<number of test runs>"},
        {STEP,       0, "", "step",      option::Arg::Optional, "--step=<number of nodes to add to each consecutive test>"}
};

int main(int argc, char *argv[]) {
    argc -= (argc > 0);
    argv += (argc > 0);
    option::Stats stats(usage, argc, argv);
    option::Option options[stats.options_max], buffer[stats.buffer_max];
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

    return 0;
}