//
// Created by Noam Dori on 5/07/18.
//

#include <dddynamic_reconfigure/param/ddd_string_param.h>

namespace dddynamic_reconfigure {

    Value DDDString::getDefault() const {
        return Value(def_);
    }

    void DDDString::setDefault(Value val) {
        def_ = val.toString();
    }

    void DDDString::setLevel(unsigned int lvl) {
        level_ = lvl;
    }


    shared_ptr<DDParam> DDDString::copy() {
        return shared_ptr<DDParam>(new DDDString(*this));
    }
}
