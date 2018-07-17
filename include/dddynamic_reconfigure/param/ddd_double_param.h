//
// Created by Noam Dori on 5/07/18.
//

#ifndef DDDYNAMIC_RECONFIGURE_DDD_DOUBLE_PARAM_H
#define DDDYNAMIC_RECONFIGURE_DDD_DOUBLE_PARAM_H

#include "dddynamic_reconfigure/ddd_ordered_param.h"
#include <ddynamic_reconfigure/param/dd_double_param.h>


namespace dddynamic_reconfigure {
    typedef numeric_limits<double> d_limit;
    /**
     * @brief a dynamic double implementation of the parameter.
     *        This is used to handle double-precision floating point numbers,
     *        though it can handle single precision as well.
     */
    class DDDDouble : public DDDOrdered, public DDDouble {
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

        DDPtr copy();

        /**
         * creates a new dynamic double param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value
         * @param description details about the parameter
         * @param max the maximum allowed value. Defaults to DBL_MAX
         * @param min the minimum allowed value. Defaults to -DBL_MAX
         */
        DDDDouble(const string &name, unsigned int level, const string &description, double def,
                 double min = -d_limit::infinity(), double max = d_limit::infinity())
                : DDDouble(name,level,description,def, min, max) {}
    };
}


#endif //DDDYNAMIC_RECONFIGURE_DDD_DOUBLE_PARAM_H
