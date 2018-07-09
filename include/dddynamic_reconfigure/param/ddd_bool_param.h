//
// Created by Noam Dori on 5/07/18.
//

#ifndef DDDYNAMIC_RECONFIGURE_DDD_BOOL_PARAM_H
#define DDDYNAMIC_RECONFIGURE_DDD_BOOL_PARAM_H

#include "dddynamic_reconfigure/ddd_param.h"
#include <ddynamic_reconfigure/param/dd_bool_param.h>

namespace dddynamic_reconfigure {
    /**
     * @brief a dynamic boolean implementation of the parameter.
     *        These are used to handle true/false values, or bit quantities if needed.
     *        In ROS, booleans are handled as u-bytes (u-int8), so be careful with these!
     */
    class DDDBool : public DDDParam, public DDBool {
    public:

        Value getDefault() const;

        void setDefault(Value val);

        void setLevel(unsigned int lvl);

        /**
         * @brief creates a new dynamic bool param
         * @param name the name of the parameter
         * @param level the change level
         * @param description details about the parameter
         * @param def the default value
         */
        DDDBool(const string &name, unsigned int level, const string &description, bool def)
                : DDBool(name,level,description,def) {}
    };
}


#endif //DDDYNAMIC_RECONFIGURE_DDD_BOOL_PARAM_H
