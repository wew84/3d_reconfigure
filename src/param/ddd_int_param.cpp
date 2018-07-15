//
// Created by Noam Dori on 5/07/18.
//

#include <dddynamic_reconfigure/param/ddd_int_param.h>

namespace dddynamic_reconfigure {

    Value DDDInt::getDefault() const {
        return Value(def_);
    }

    void DDDInt::setDefault(Value val) {
        def_ = val.toInt();
    }

    void DDDInt::setLevel(unsigned int lvl) {
        level_ = lvl;
    }

    Value DDDInt::getMin() const {
        return Value(min_);
    }

    void DDDInt::setMin(Value val) {
        min_ = val.toInt();
    }

    Value DDDInt::getMax() const {
        return Value(max_);
    }

    void DDDInt::setMax(Value val) {
        max_ = val.toInt();
    }

    bool DDDInt::outOfMin() const {
        return min_ > val_;
    }

    bool DDDInt::outOfMax() const {
        return max_ < val_;
    }

    shared_ptr<DDParam> DDDInt::copy() {
        return shared_ptr<DDParam>(new DDDInt(*this));
    }
}
