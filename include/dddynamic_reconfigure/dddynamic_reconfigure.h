//
// Created by Noam Dori on 5/07/18.
//

#ifndef DDDYNAMIC_RECONFIGURE_DDDYNAMIC_RECONFIGURE_H
#define DDDYNAMIC_RECONFIGURE_DDDYNAMIC_RECONFIGURE_H

//include space, written in C++03
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "ddd_ordered_param.h"
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
using namespace std;
using namespace boost;
using namespace dynamic_reconfigure;
using namespace ddynamic_reconfigure;
namespace dddynamic_reconfigure {
    // this is the pointer to any type of 3-Dynamic parameter.
    typedef shared_ptr<DDDParam> DDDPtr;
    // this is the pointer to any type of 3-Dynamic ordered parameter.
    typedef shared_ptr<DDDOrdered> DDDOrderedPtr;

    /**
     * @brief The DDDynamicReconfigure class is the main class responsible for keeping track of parameters basic properties,
     *        values, descriptions, etc.
     *
     *        It is also responsible of handling callbacks, config change requests, description setup and config setup,
     *        and the ROS publishers and services.
     *
     *        To operate a DDDynamic instance, you must go through the following procedure:
     *
     *        1. Construct a DDDynamicReconfigure instance with proper handling.
     *        2. Add parameters to the instance as needed with any of the "add" methods.
     *        3. Start the ROS services with any of the "start" methods.
     *        4. If you need to change the callback after startup you may do so using "setCallback".
     *        5. When you need to get any of the stored parameters, call either "get" or "at" on this instance,
     *           rather than through the callback.
     */
    class DDDynamicReconfigure : public DDynamicReconfigure {
    public:

        /**
         * @brief creates the most basic instance of a 3d-conf object.
         * @param nh the node handler of the node this is placed at.
         */
        explicit DDDynamicReconfigure(ros::NodeHandle &nh);

        /**
         * @brief adds a parameter to the list, allowing it to be generated.
         * @param param the pointer to the 2d-param to add to the list.
         */
        void add(DDPtr param);

        /**
         * @brief adds a parameter to the list, allowing it to be generated.
         * @param param the pointer to the 2d-param to add to the list.
         */
        void add(DDParam *param);

        /**
         * @brief adds a parameter to the list, allowing it to be generated.
         * @param param the pointer to the 2d-param to remove from the list.
         */
        void remove(DDPtr param);

        /**
         * @brief adds a parameter to the list, allowing it to be generated.
         * @param param the pointer to the 2d-param to remove from the list.
         */
        void remove(DDParam *param);

        /**
         * @brief adds a parameter to the list, allowing it to be generated.
         * @param param the pointer to the 2d-param to remove from the list.
         */
        void remove(string param_name);

        /**
         * @note an extra implementation of the DDynamic start() but with a tweak of the started_ boolean.
         */
        void start();

        using DDynamicReconfigure::start; // allows use of all start methods provided in base class

        /**
         * @brief a tool people who use this API can use to find the param given within the param map,
         *        and attempt to convert it to 3D-param if possible
         * @param name the string to look for
         * @return the param with the given name if it exists and can be casted to 3D-param type, nullptr otherwise
         */
        DDDPtr at3(const char* name);

        /**
         * @brief a tool people who use this API can use to find the param given within the param map,
         *        and attempt to convert it to 3D-ordered if possible
         * @param name the string to look for
         * @return the param with the given name if it exists and can be casted to 3D-ordered type, nullptr otherwise
         */
        DDDOrderedPtr at3o(const char* name);

    protected:

        /**
         * @brief the enum holding all basic property types 3D-reconfigure allows to change.
         */
        enum Property {
            DEFAULT,
            LEVEL,
            MAX,
            MIN
        };

        /**
         * @brief calls the internal callback for the low-level service, not exposed to users.
         *        Used to update default values.
         * @param obj the object we are using for its callback.
         * @param req ----(ROS)
         * @param rsp ----(ROS)
         * @param prop the property to change.
         * @return -------(ROS)
         */
        static bool internalPropCallback(DDDynamicReconfigure *obj, Reconfigure::Request &req,
                                         Reconfigure::Response &rsp, Property prop);

        /**
         * @brief whether or not the start() method was called.
         */
        bool started_;

    private:

        /**
         * @brief reassigns a value to the internal map assuming it is registered.
         * @param map the map that is being edited
         * @param name the name of the parameter to test
         * @param value the value of the new parameter
         * @param property the enum assigned property:
         *        DEFAULT is the default property
         *        LEVEL is the level property
         *        MAX is the max property
         *        MIN is the min property
         * @return false if the value could not be reassigned, otherwise true.
         */
        static bool reassign(DDMap &map, const string &name, const Value &value, Property property);

        /**
         * @brief gets the updates and assigns them to DDMap
         *        If you are changing max or min, this will also make sure to change values that are now beyond the limit,
         *        and calls the DDynamic internal callback accordingly.
         * @param req the ROS request holding info about the new map
         * @param prop the enum assigned property:
         *        DEFAULT is the default property
         *        LEVEL is the level property
         *        MAX is the max property
         *        MIN is the min property
         */
        void update(const Reconfigure::Request &req, Property prop);

        /**
         * @brief edits a reconfigure msg to edit the param if needed.
         * @param reconfigure the ROS reconfigure msg to edit.
         * @param prop the enum assigned property:
         *        MAX is the max property
         *        MIN is the min property
         *        any other property is ignored.
         * @param param the param to look into
         * @return true if there was any change, false otherwise.
         */
        bool manageMaxMin(Reconfigure &reconfigure, Property prop, const DDPtr &param);

        #pragma clang diagnostic push
        #pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
        /**
         * @brief services that ask to update basic properties of the variables:
         *        (lvl_service_) updates severity level
         *        (def_service_) updates defaults
         *        (range_service_) updates max and min.
         */
        ros::ServiceServer lvl_service_, def_service_, max_service_, min_service_;
        #pragma clang diagnostic pop
    };

    /**
     * @brief a tool people who use this API can use to find the param given within the param map,
     *        and attempt to convert it to 3D-param if possible
     * @param name the string to look for
     * @param map the map to search
     * @return the param with the given name if it exists and can be casted to 3D-param type, nullptr otherwise
     */
    DDDPtr at3(const DDMap& map, const char* name);

    /**
     * @brief a tool people who use this API can use to find the param given within the param map,
     *        and attempt to convert it to 3D-ordered if possible
     * @param name the string to look for
     * @param map the map to search
     * @return the param with the given name if it exists and can be casted to 3D-ordered type, nullptr otherwise
     */
    DDDOrderedPtr at3o(const DDMap& map, const char* name);
}
#endif //DDDYNAMIC_RECONFIGURE_DDDYNAMIC_RECONFIGURE_H