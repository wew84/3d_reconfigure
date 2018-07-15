//
// Created by Noam Dori on 5/07/18.
//

#include <dddynamic_reconfigure/param/ddd_double_param.h>

namespace dddynamic_reconfigure {
    
    Value DDDDouble::getDefault() const {
        return Value(def_);
    }

    void DDDDouble::setDefault(Value val) {
        def_ = val.toDouble();
    }

    void DDDDouble::setLevel(unsigned int lvl) {
        level_ = lvl;
    }

    Value DDDDouble::getMin() const {
        return Value(min_);
    }

    void DDDDouble::setMin(Value val) {
        min_ = val.toDouble();
    }

    Value DDDDouble::getMax() const {
        return Value(max_);
    }

    void DDDDouble::setMax(Value val) {
        max_ = val.toDouble();
    }

    bool DDDDouble::outOfMin() const {
        return min_ > val_;
    }

    bool DDDDouble::outOfMax() const {
        return max_ < val_;
    }


    shared_ptr<DDParam> DDDDouble::copy() {
        return shared_ptr<DDParam>(new DDDDouble(*this));
    }
}
