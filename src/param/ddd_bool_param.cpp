//
// Created by Noam Dori on 5/07/18.
//

#include <dddynamic_reconfigure/param/ddd_bool_param.h>

namespace dddynamic_reconfigure {

    Value DDDBool::getDefault() const {
        return Value(def_);
    }

    void DDDBool::setDefault(Value val) {
        def_ = val.toBool();
    }

    void DDDBool::setLevel(unsigned int lvl) {
        level_ = lvl;
    }

    DDPtr DDDBool::copy() {
        return DDPtr(new DDDBool(*this));
    }
}
