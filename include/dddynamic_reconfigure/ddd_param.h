//
// Created by Noam Dori on 5/07/18.
//

#ifndef DDDYNAMIC_RECONFIGURE_DDD_PARAM_H
#define DDDYNAMIC_RECONFIGURE_DDD_PARAM_H

#include <string>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <ddynamic_reconfigure/dd_param.h>

using namespace dynamic_reconfigure;
using namespace ddynamic_reconfigure;
using namespace std;
namespace dddynamic_reconfigure {
    /**
     * @brief The DDDParam interface is the abstraction of all dynamic parameter types, and is the template for creating them.
     *        This class extends the DDParam interface in a generic, dynamic manner.
     *        At this point, not much is known about the parameter, but the following:
     *
     *        - the parameter has a name
     *        - the parameter has a severity level, which can be queried and changed.
     *        - the parameter has a description
     *        - the parameter contains some value, though its type and contents are unknown.
     *        - the parameter has a default value, which can be queried and changed.
     *
     *        Other than storing data, the parameter also has specialised methods to interact with DDynamicReconfigure in order to apply changes and send them.
     *        These methods should not be touched by the user.
     *
     *        Since this class is abstract, the class has multiple implementations which are not directly exposed but are used,
     *        so its worth checking out their descriptions.
     *
     *         While this class is abstract, it does have two implemented things:
     *         - the stream operator (`<<`).
     *         - whether or not this value is ordered or not (by default no).
     *
     *        While DDDParam is abstract, all of its concrete implementations should follow this guideline:
     *              DDD<Type>(const string &name, unsigned int level, const string &description, <type> def, <extra-args>)
     *        Where:
     *        - <Type> is the type name you are implementing
     *        - name is the reference name
     *        - level is the severity level
     *        - description is the object's description
     *        - def is the default value and the first value stored right after construction.
     *
     *        You may then include extra arguments as you wish, required or optional.
     */
    class DDDParam : virtual public DDParam {
    public:

        /**
         * @brief gets the default of this parameter.
         * @return the default-value stored in this param.
         */
        virtual Value getDefault() const = 0;

        /**
         * @brief sets the default of this parameter as this one.
         * @param val the value to use
         */
        virtual void setDefault(Value val) = 0;

        /**
         * @brief sets the default of this parameter as this one.
         * @param lvl the new level of the parameter.
         */
        virtual void setLevel(unsigned int lvl) = 0;

        /**
         * @brief whether or not this object is ordered:
         *        if it is considered that each value can either be bigger than, less than,
         *        or equal to any other object in the domain.
         * @note all objects which are ordered should inherit the DDDOrdered object unless there is a good reason why not.
         * @return true is the object is ordered, false otherwise.
         */
        virtual bool isOrdered();

        /**
         * @brief makes a copy of this param.
         * @return a pointer to a new param with the same properties as this one.
         */
        virtual DDPtr copy() = 0;
    };
}
#endif //DDDYNAMIC_RECONFIGURE_DDD_PARAM_H
