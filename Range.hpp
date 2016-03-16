//
// Created by Seth Wiesman on 3/8/16.
//

#ifndef THESIS_RANGE_HPP
#define THESIS_RANGE_HPP

namespace Thesis {
    class Range {
        double min;
        double max;

    public:
        enum Position {
            Less,
            Inside,
            Greater,
        };

        Range(double min, double max) : min(min), max(max) { }

        Position position(double num);
    };
}


#endif //THESIS_RANGE_HPP
