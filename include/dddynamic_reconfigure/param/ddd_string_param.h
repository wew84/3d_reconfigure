//
// Created by Noam Dori on 5/07/18.
//

#ifndef DDDYNAMIC_RECONFIGURE_DDD_STRING_PARAM_H
#define DDDYNAMIC_RECONFIGURE_DDD_STRING_PARAM_H

#include "dddynamic_reconfigure/ddd_param.h"
#include <ddynamic_reconfigure/param/dd_string_param.h>

namespace dddynamic_reconfigure {
    /**
     * @brief a dynamic string implementation of the parameter.
     * This is used to handle strings of characters of variable length.
     * Like string, each param value can hold up to 2^32-1 characters.
     */
    class DDDString : public DDDParam, public DDString {
    public:
        Value getDefault() const;

        void setDefault(Value val);

        void setLevel(unsigned int lvl);

        shared_ptr<DDParam> copy();

        /**
         * creates a new dynamic string param
         * @param name the name of the parameter
         * @param level the change level
         * @param description details about the parameter
         * @param def the default value
         */
        DDDString(const string &name, unsigned int level, const string &description, const string &def)
                  : DDString(name,level,description,def) {}
    };
}


#endif //DDDYNAMIC_RECONFIGURE_DDD_STRING_PARAM_H
