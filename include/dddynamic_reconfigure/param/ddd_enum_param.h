//
// Created by Noam Dori on 5/07/18.
//

#ifndef DDDYNAMIC_RECONFIGURE_DDD_ENUM_PARAM_H
#define DDDYNAMIC_RECONFIGURE_DDD_ENUM_PARAM_H

#include "ddd_int_param.h"
#include <ddynamic_reconfigure/param/dd_enum_param.h>
#include <boost/foreach.hpp>

namespace dddynamic_reconfigure {
    typedef map<string,pair<int,string> > EnumMap;
    /**
     * @brief a dynamic integer enum implementation of the parameter.
     *        This is an extension to the int parameter,
     *        which allows creating string aliases for certain (if not all) numbers available.
     * @remark why doesn't this class inherit DDEnum?
     *         First, note that both DDDInt (which DDDEnum inherits) and DDEnum are both non-interfaces (moreover, they are both concrete).
     *         A quote from Oracle's explanation as to why Java does not implement multiple non-interfaces
     *         (https://docs.oracle.com/javase/tutorial/java/IandI/multipleinheritance.html):
     *
     *              "suppose that you are able to define a new class that extends multiple classes.
     *              When you create an object by instantiating that class, that object will inherit fields from all of the class's superclasses.
     *              What if methods or constructors from different superclasses instantiate the same field?
     *              Which method or constructor will take precedence?"
     *
     *         In other words, there are bound to be conflicts between methods with multi-inheritance.
     *         This can be avoided if you implement multiple interfaces instead of classes,
     *         since interfaces do not own any fields from their parent interfaces or by their own, since now:
     *              - you only inherit the fields from one superclass
     *              - only one method or constructor can instantiate the given fields (the ones from the class and not the interfaces)
     *              - interfaces can only implement methods using other methods within them, making them much safer.
     *
     *         This clears up why DDDEnum cannot implement both DDDInt and DDEnum.
     *         However, this raises another question: why doesn't DDDEnum implement DDEnum and DDDOrdered instead?
     *         After all, DDDOrdered is an interface.
     *         The answer for this comes from the following consideration: is a dynamic enum an extension to a dynamic integer,
     *         or an extension to an enum (with ordered dynamic capabilities)?
     *         When looking into the enum, we see it is an extension to an integer.
     *         Therefore, we expect the same from the dynamic enum, which is why dynamic enums extend dynamic integers.
     *         Another explanation is considering the use case of DDDEnum.
     *         suppose we remove 2d-reconfigure from the ordeal, and re-implement everything in their child classes within 3d-reconfigure.
     *         After all implementation, the only thing left is that now DDDEnum does not implement DDDInt.
     *         This leaves our enum without implementations for some of its methods.
     *         This means that in our scenario, DDDEnum must extend DDDInt.
     *         If in that scenario DDDEnum must extend DDDInt, why not here?
     */
    class DDDEnum : public DDDInt {
    public:

        bool sameType(Value val);

        bool sameValue(Value val);

        void setValue(Value val);

        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value in integer form
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         */
        DDDEnum(const string &name, unsigned int level, const string &description,
                int def, const map<string,int> &dictionary);

        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def an alias of the default value
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         */
        DDDEnum(const string &name, unsigned int level, const string &description,
                const string& def, const map<string,int> &dictionary);

        #pragma clang diagnostic push
        #pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value in integer form
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         * @note since ROS cannot send the enum and const descriptions, this method is useless.
         *       Please use the constructor which takes a map<string,int> instead.
         * @deprecated see note. This is not tested, so it may fail.
         */
        DDDEnum(const string &name, unsigned int level, const string &description,
               int def, const pair<map<string,pair<int,string> >,string> &dictionary);

        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def an alias of the default value
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         * @note since ROS cannot send the enum and const descriptions, this method is useless.
         *       Please use the constructor which takes a map<string,int> instead.
         * @deprecated see note. This is not tested, so it may fail.
         */
        DDDEnum(const string &name, unsigned int level, const string &description,
               const string& def, const pair<map<string,pair<int,string> >,string>  &dictionary);
        #pragma clang diagnostic pop

        bool hasDefinition(string name);

        int getDefinition(string name);

        void removeDefinition(string name);

        void addDefinition(string name, int definition);

        void addDefinition(string name, int definition, string description);

    protected:
        /** 
         * @brief A dictionary from the string aliases to their integer counterparts.
         * This method of storage allows integers to have multiple aliases.
         */
        EnumMap dict_;
    };
}

#endif //DDDYNAMIC_RECONFIGURE_DDD_ENUM_PARAM_H