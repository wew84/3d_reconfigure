//
// Created by Noam Dori on 5/07/18.
//

#ifndef DDDYNAMIC_RECONFIGURE_DDD_INT_PARAM_H
#define DDDYNAMIC_RECONFIGURE_DDD_INT_PARAM_H

#include "dddynamic_reconfigure/ddd_ordered_param.h"
#include <ddynamic_reconfigure/param/dd_int_param.h>

namespace dddynamic_reconfigure {
    /**
     * @brief an integer implementation of the dynamic parameter.
     *        This is used to 32 bit signed integral numbers.
     *        This can also handle shorts, bytes, and other integrals provided they are not too big
     *        (by then looping will occur).
     *        This also implements DDDOrdered meaning you can change and get def,lvl,min,max.
     */
    class DDDInt : public DDDOrdered, public DDInt {
    public:

        Value getDefault() const;

        void setDefault(Value val);

        void setLevel(unsigned int lvl);

        bool outOfMin() const;

        Value getMin() const;

        void setMin(Value val);

        bool outOfMax() const;

        Value getMax() const;

        void setMax(Value val);

        shared_ptr<DDParam> copy();

        /**
         * creates a new dynamic int param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value
         * @param description details about the parameter
         * @param max the maximum allowed value. Defaults to INT32_MAX
         * @param min the minimum allowed value. Defaults to INT32_MIN
         */
        inline DDDInt(const string &name, unsigned int level, const string &description,
                int def, int min = INT32_MIN, int max = INT32_MAX) : DDInt(name,level,description,def, min, max) {}
    };
}

#endif //DDDYNAMIC_RECONFIGURE_DDD_INT_PARAM_H
