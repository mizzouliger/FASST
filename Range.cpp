//
// Created by Seth Wiesman on 3/8/16.
//

#include "Range.hpp"
namespace Thesis {
    Range::Position Range::position(double num) {
        if (num < this->min) {
            return Less;
        } else if (num > this->max) {
            return Greater;
        } else {
            return Inside;
        }
    }
}
