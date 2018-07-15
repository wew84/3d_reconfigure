//
// Created by Noam Dori on 5/07/18.
//

#ifndef DDDYNAMIC_RECONFIGURE_DDD_ORDERED_PARAM_H
#define DDDYNAMIC_RECONFIGURE_DDD_ORDERED_PARAM_H

#include <dddynamic_reconfigure/ddd_param.h>

using namespace ddynamic_reconfigure;
using namespace std;
namespace dddynamic_reconfigure {

    /**
     * @brief this class extends the base class with additional features regarding order.
     *        In general, all parameters that fit the order axiom should implement this class.
     */
    class DDDOrdered : public DDDParam {
    public:

        /**
         * @brief checks if the value of this param resides in the allowed range, from max's point of view.
         * @return true if the current value is bigger than the max allowed, false otherwise.
         */
        virtual bool outOfMin() const = 0;

        /**
         * @brief gets the min allowed value of this parameter.
         * @return the min-value stored in this param.
         */
        virtual Value getMin() const = 0;

        /**
         * @brief sets the min of this parameter as this one.
         * @param val the value to use
         */
        virtual void setMin(Value val) = 0;

        /**
         * @brief checks if the value of this param resides in the allowed range, from max's point of view.
         * @return true if the current value is bigger than the max allowed, false otherwise.
         */
        virtual bool outOfMax() const = 0;

        /**
         * @brief gets the max allowed value of this parameter.
         * @return the max-value stored in this param.
         */
        virtual Value getMax() const = 0;

        /**
         * @brief sets the max of this parameter as this one.
         * @param val the value to use
         */
        virtual void setMax(Value val) = 0;

        bool isOrdered();
    };
}
#endif //DDDYNAMIC_RECONFIGURE_DDD_PARAM_H
