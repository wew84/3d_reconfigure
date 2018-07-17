DDDynamic-Reconfigure
==================================================
The DDDynamic-Reconfigure package (or 3D-reconfig) is a **C++** based extension to DDynamic-Reconfigure (or 2D-reconfig) which allows C++ based nodes to dynamically reconfigure every aspect of their parameters.

## Dependencies
3D-reconfig depends on 2D-reconfig, as it is an extension to its classes.

## Configuration
Other than the installation of the package to your workspace, no other configuration is needed.
The package used is called ``dddynamic_reconfigure``,
and this is both the namespace and the include directory used to implement the program.

## Implementation
let us look into the following code, which implements 3D-Reconfig:
````cpp
#include <ros/ros.h>

#include <dddynamic_reconfigure/dddynamic_reconfigure.h>
#include <dddynamic_reconfigure/param/ddd_all_params.h>

using namespace dddynamic_reconfigure;

void callback(const DDDMap& map, int) {
    ROS_INFO("Reconfigure Request: %d %f %s %s %ld",
            get(map, "int_param").toInt(), get(map, "double_param").toDouble(),
            get(map, "str_param").toString().c_str(),
            get(map, "bool_param").toBool() ? "True" : "False",
            map.size());
}

int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "dddynamic_tutorials");
    ros::NodeHandle nh;

    // DDDynamic setup stage
    DDDynamicReconfigure ddd(nh);
    ddd.add(new DDDInt("int_param", 0, "An Integer parameter", 50, 0, 100));
    ddd.add(new DDDDouble("double_param", 0, "A double parameter", .5, 0, 1));
    ddd.add(new DDDString("str_param", 0, "A string parameter", "Hello World"));
    ddd.add(new DDDBool("bool_param", 0, "A Boolean parameter", true));
    std::map<std::string, int> dict; // An enum to set size
        dict["Small"] = 0;      // A small constant
        dict["Medium"] = 1;     // A medium constant
        dict["Large"] = 2;      // A large constant
        dict["ExtraLarge"] = 3; // An extra large constant
    ddd.add(new DDDEnum("enum_param", 0, "A size parameter which is edited via an enum", 1, dict));
    ddd.start(callback);

    // Actual Server Node code
    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
````
This segment of code is used for declaring the configuration file and for setting up the server in place of the node which uses the parameters.

### Breakdown

Let's break down the code line by line:
```cpp
#include <ros/ros.h>

#include <dddynamic_reconfigure/dddynamic_reconfigure.h>
#include <dddynamic_reconfigure/param/ddd_all_params.h>

using namespace dddynamic_reconfigure;
```
In here, we import all needed files:
* ``<ros/ros.h>`` provides basic ROS management.
* ``<dddynamic_reconfigure/dddynamic_reconfigure.h>`` provides the 2D-reconfigure API
* ``<dddynamic_reconfigure/param/ddd_all_params.h>`` allows you to use all default parameter types.

The non include line allows us to use classes and functions provided in the ``dddynamic_reconfigure`` namespace
without mentioning what package they are from.

These do not change.

```cpp
void callback(const DDDMap& map, int) {
    ROS_INFO("Reconfigure Request: %d %f %s %s %ld",
            get(map, "int_param").toInt(), get(map, "double_param").toDouble(),
            get(map, "str_param").toString().c_str(),
            get(map, "bool_param").toBool() ? "True" : "False",
            map.size());
}
```

This is the callback used when 3D-reconfig receives a parameter change request.
It takes two parameters: the first is a map of the new configuration mapping from the name of the parameter to the actual parameter object,
and the second is the level, which is the highest level of severity caused by the parameter change.
This is calculated by applying the OR operator on all levels of the parameters that changed.

For those who used 2D-reconfigure, DDDMap is equivalent to DDMap

In this callback the level is not used, but we do print out the new configuration.

```cpp
int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "dddynamic_tutorials");
    ros::NodeHandle nh;
```

All this section do is initialise our ROS node and its handler.
This is default stuff you do anyways.

```cpp
    // DDDynamic setup stage
    DDDynamicReconfigure ddd(nh);
    ddd.add(new DDDInt("int_param", 0, "An Integer parameter", 50, 0, 100));
    ddd.add(new DDDDouble("double_param", 0, "A double parameter", .5, 0, 1));
    ddd.add(new DDDString("str_param", 0, "A string parameter", "Hello World"));
    ddd.add(new DDDBool("bool_param", 0, "A Boolean parameter", true));
```

This is we start using 3D-reconfig. First, we initialise our 3D-reconfig object.
Then, we start adding parameters to it. In 3D-reconfig, adding parameters is not just a simple function,
but you have to add a parameter object (an instance of the ``DDParam`` interface,
and more specifically, an instance of the ``DDDParam`` interface).
Let's look into the param objects above to see some common factors:
* Even though 3D-reconfig is compatible with 2D-params, we here will use 3D-params instead

* The type of the parameter is declared first by specifying ``new DDDType()``.
  For example, adding a new int parameter is done by doing ``ddd.add(new DDDInt(...))``

* Within the param constructor, the first argument is the name of the parameter.
  For example, in our int parameter, the name is set to ``"int_param"``.

* The second argument is the level of the parameter, that is,
  what needs to be reset or redone in the software/hardware in order to reapply this parameter?
  Usually, the higher the level, the more drastic measures you need to take to re-implement the parameter.

* The third parameter is the description of the parameter. This is great for documentation and for commandline tools.

* The fourth parameter is the default value. Depending on the type of parameter, each may treat this argument differently.

* ``DDDInt`` and ``DDDDouble`` have a fifth and sixth optional parameters: minimum and maximum allowed values.
  While the server side does not care about these values, the client may want to know these.

* It is important to note that the first 4 arguments are standardised for all param types,
  but from there onwards each param type may choose what to place there.

```cpp
    std::map<std::string, int> dict; // An enum to set size
        dict["Small"] = 0;      // A small constant
        dict["Medium"] = 1;     // A medium constant
        dict["Large"] = 2;      // A large constant
        dict["ExtraLarge"] = 3; // An extra large constant
    ddd.add(new DDDEnum("enum_param", 0, "A size parameter which is edited via an enum", 1, dict));
```

Here we add an int-enum parameter to our 2D-reconfig. ``DDDEnum`` is an int like parameter that also contains a dictionary
to remap predefined strings to usable integers. This param type has a required 5th argument (in contract to ``DDDInt`` having 5th and 6th optional)
which is a ``std::map<std::string,int>`` object mapping string values to integers.

In the code above we can see how to create a dictionary of our liking:

* we first initiate a map and name it with ``std::map<std::string,int> dict``.
* we then populate it with the format ``dict[<key>] = <value>`` where ``<key>`` is the string alias for the value,
  and ``<value>`` is the value you want to give an alias to.

This dictionary is then added into the enum as the 5th argument.

```cpp
    ddd.start(callback);

    // Actual Server Node code
    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
```

This section of code actually allows 3D-reconfigure to start working. Let's look into the two sections:
* ``ddd.start(callback)`` sets the callback of 3D-reconfigure to be the method ``callback`` and jump starts 3D-reconfigure.
* ``ros::spin()`` allows 3D-reconfigure to listen to parameter-change requests.
  Although the node now requires a spin, this does not mean you cannot add your own service-servers and subscribers to this node.
  ``ros::spin()`` can take care of multiple subscribers/service-servers in the same spinners (although in the same thread).
  If you want 2D-reconfig and your actual node to work on separate threads, consider using ``ros::MultiThreadedSpinner``.
  3D-reconfigure only uses 1 service-server and no subscribers, so 1 thread for it is more than enough.

### How does this compare with Dynamic-Reconfigure?
the basic implementation of 3D-reconfig and 2D-reconfig are completely identical (with consideration to the namesakes)

The difference between 2D-reconfigure and 3D-reconfigure is the ability to change every basic attribute within each parameter:
defaults, maximum, minimum, severity level. You also get the ability to add and remove parameters after the server started,
and the ability to add and remove definitions from enums after init (though this is less supported).
In the simplified API below you will see what is available.

One special property of 3D-reconfigure is that it can be used interchangeably with 2D-reconfigure,
 although anything you use with the 2D prefix will not be able to use all the features provided in 3D-reconfigure.

### Simplified API

#### Value

Although Value (or DDValue) belongs to the 2D-reconfig package, it is important to explain here as well.

The Value class is used to wrap all basic data-types (bool,int,double,string) in something generic.
The value object always stores an explicit basic data-type.
This has three main uses:

1. Values can represent all basic data-types. This means that arguments that need something relatively similar from all basic data-types can now just use the value in its argument.
   This also goes for when you need to return something that is of different data-types from different classes (one can only return integer, other can only return strings).

2. Values can be explicitly converted to all basic data-types they wrap. 
   This means that converting an int to a string is far easier.

3. Values store the type they were instantiated with. This can be tested against to get the original piece of data the value stored.

##### Constructors

``Value(int val)``,``Value(double val)``,``Value(bool val)``,``Value(string val)``,``Value(const char* val)``
are all constructors that assign the value type to the type they are given (with the exception for ``const char*`` which returns string and is there for convenience),
then store the value itself in its basic form.

##### Getter

There is only one true getter: ``getType()``, which returns the string name of the type it stores.

##### Converters

Each basic data-type has its own converter: ``toInt()``,``toDouble()``,``toBool()``,``toString()``.
When one is called, the value will attempt to return a converted form of what it stores into the required data-type.
The value does not just use an implicit cast. It tries to convert the datatype according to common needs that are not answered with other one-liners.
For example, converting a string to an int, a Value will first attempt to scan the string fand see it fits a numeric format.
If it succeeds, it will convert and return that number. Otherwise, it will return the next best thing: a hash value of the string.

#### DDDParam

The DDDParam interface is the abstraction of all *dynamic* parameter types, and is the template for creating them.
This class extends the DDParam interface in a generic, dynamic manner.
At this point, not much is known about the parameter, but the following:

* the parameter has a name
* the parameter has a severity level, which can be queried and changed.
* the parameter has a description
* the parameter contains some value, though its type and contents are unknown.
* the parameter has a default value, which can be queried and changed.

Other than storing data, the parameter also has specialised methods to interact with DDynamicReconfigure in order to apply changes and send them.
These methods should not be touched by the user.

Since this class is abstract, the class has multiple implementations which are not directly exposed but are used,
so its worth checking out their descriptions.

While this class is abstract, it does have two implemented things:
* the stream operator (`<<`).
* whether or not this value is ordered or not (by default no).

##### Generic Constructor

While DDDParam is abstract, all of its concrete implementations should follow this guideline:
```cpp
DDD<Type>(const string &name, unsigned int level, const string &description, <some-type> def, <extra-args>)
```
Where:
* ``<Type>`` is the type name you are implementing
* ``name`` is the reference name
* ``level`` is the severity level
* ``description`` is the object's description
* ``def`` is the default value and the first value stored right after construction.

You may then include extra arguments as you wish, required or optional.

##### Getters

parameters have many well known getters:
* ``getName()`` gets the name of the parameter.
* ``getLevel()`` gets the severity level of the parameter.
* ``getDefault()`` gets the default value of the parameter.
* ``getValue()`` gets the value the parameter stores.

ordered parameters (those which implement the ``DDOrdered`` interface) also have the following getters:
* ``getMin()`` gets the minimum value the parameter allows.
* ``getMax()`` gets the maximum value the parameter allows.

Other getters, such as "getDesc()", may be added in the future.

the parameters also have a stream (``<<``) operator which can be used to convert said parameters into neat strings.

##### Setters

3D-params are dynamic in many aspects, so they have many setters:

* ``setValue(Value val)`` changes the value the parameter stores.
* ``setDefault(Value val)`` changes the default value of the parameter.
* ``setLevel(unsigned int lvl)`` changes the severity level of the parameter.

ordered parameters also have the following setters:
* ``setMin()`` changes the minimum value the parameter allows.
* ``setMax()`` changes the maximum value the parameter allows.

##### Testers

DDDParams are also required to have some out-of-the-box testing features:
* ``sameType(Value val)`` checks whether or not 
  the raw value stored in the value is compatible with the given parameter.
  Compatible is a very broad word in this scenario.
  It means that the value can be placed in the parameter regardless of other limitations.

* ``sameValue(Value val)`` checks whether or not the value stored in the value object,
  when converted to the type of the internal value, are equal. This acts regardless of type.

* ``isOrdered()`` indicates whether or not the object acts in an ordered fashion. By default this returns false,
  but objects can override this.

#### DDDynamicReconfigure

The DDDynamicReconfigure class is the main class responsible for keeping track of parameters basic properties,
values, descriptions, etc.

It is also responsible of config change requests, description setup and config setup,
dynamic changes of the parameters' properties, and the ROS services.

Some things are left to be handled by the parent class, the DDynamicReconfigure class.

To operate a DDDynamic instance, you must go through the following procedure:

1. Construct a DDDynamicReconfigure instance with proper handling.
2. Add parameters to the instance as needed with any of the ``add`` methods (and remove any you don't need with ``remove``).
3. Start the ROS services with any of the ``start`` methods.
4. If you need to change the callback after startup you may do so using ``setCallback``.
5. When you need to get any of the stored parameters, call either ``get`` or ``at`` on this instance,
   rather than through the callback.

##### Constructor

DDDynamicReconfigure has one sole constructor: ``DDDynamicReconfigure(NodeHandle &nh)`` which constructs the instance and
sets the handler to the one you are using.

##### Parameter Handling

All parameter handling is done through registration using an ``add`` function:

* ``add(DDPtr param)`` is the main function which uses boost's shared pointers to represent the data in a virtual manner (and allows polymorphism)
* ``add(DDParam *param)`` is a convenience function which converts ``param`` into a shared pointer and uses the other add function.
  Be careful with this one.

Both of these functions will add a generic ``DDParam`` object into the given instance and will index it for later searches.

you can also remove the parameter using the ``remove`` function:
* ``remove(string param_name)`` is the main function which removes the parameter with the given name.
* ``remove(DDPtr param)`` removes the parameter that has the same name as this one.
* ``remove(DDParam *param)`` removes the parameter that has the same name as this one.

Both of these methods (add and remove) can be used before and after the 3D-reconfig instance applied its ``start()`` method, and will publish things accordingly.

##### Callback Handling & Startup

Below are the two default functions that are used by the rest:

* ``start()`` initializes all publishers and services and releases the needed messages for the commandline and other clients.
* ``setCallback(DDFunc callback)`` sets the triggered callback to the one specified, and triggers nothing else.

There is also ``clearCallback()`` which resets the callback to do nothing when triggered.

Following are convenience function which utilize ``start()`` and ``setCallback()``:

* ``start(DDFunc callback)`` calls start(), then setCallback(callback)
* ``start(void(*callback)(const DDMap&, int))`` remaps the void pointer to a boost function (of type ``DDFunc``) then calls start(callback)
* ``template<class T> void start(void(T::*callback)(const DDMap&, int), T *obj)``
  binds the **member** function into a boost function (of type ``DDFunc``) then calls start(callback)

##### Parameter Fetching

There are multiple proper ways to get the values stored within the DDDynamicReconfigure instance:

* through ``at(string name)``: this will get you the pointer to the parameter with the name you specified.
  If no such parameter exists it will return you a null-pointer (be careful not to de-reference those!)
  You also have variants of the ``at`` method to get special objects tailored to 3D-reconfig's param objects:
  * ``at3`` retrieves 3D-specific parameters, with all of the proper API. Will return a null-ptr if the param is not 3D-capable.
  * ``at3o`` retrieves 3D-ordered parameters, with all of the proper API. Will return a null-ptr if the param is not 3D-capable, and is not ordered.
  * ``at`` retrieves all parameters without fail (unless name is wrong), but only provides 2D-param API.

* through ``get(string name)``: this will get you the value stored in the parameter with the name you specified.
  If no such parameter exists it will return you a value storing a NULL character.

* through the stream (``<<``) operator: this will convert the 2D-reconfig instance into a string and stream it into the
  given streamer.

both ``at`` (and its variants) and ``get`` have alternate static versions which apply directly on ``DDMap`` objects.

## Architecture

### Code Design

#### Include Structure:

![](http://www.plantuml.com/plantuml/png/5Smn3i8m44JHdbF01Rm558WgDucCFKaisKwqEojV7rDzwHTlCKQbjtfxb9wgPxZM-q5UzX7HEC9UUFZN3PhEJdI6T3OP6E-NFFEgc7ihkokaHIDKfYPRgLYDwedX-lCBUfxz0G00.png)

To operate 3D-reconfigure, you will need to include 2 file types:

* The ``dddynamic_reconfigure`` file, which gives you access to the ``DDDynamicReconfigure`` class,
  the ``DDDParam`` class (and in turn the ``DDParam`` class), the ``DDValue`` class, and the toolbox methods.
  This will allow you to operate on the top level API without caring about what type of parameters you will get.

* the file ``ddd_all_params`` or any of the ``DDDParam`` implementations. You will need the implementations to insert physical 
  (and not abstract) parameters into your ``DDDynamicReconfigure`` server.
  As a shortcut, ``ddd_all_params`` gives you all basic parameter types (int,double,bool,string,enum) in one include.

As a bonus, you also get four static class-less methods: ``get``, ``at``, ``at3``, and ``at3o``.

#### Class Structure:

![](http://www.plantuml.com/plantuml/png/5Smn3i8m34RXdLF01U811iJKcrJ1CMsanAtityhrIQSdlU5R7DaGlTqULRb5aR1stu7JFqIMHzegzlY6Y_6X44NYezEGyNxHiwmcS__g5YRKiqECfaRSillgi62ay_8NqJBx1m00.png)

Like the API section shows, there are only 3 major classes: ``DDValue``,``DDDParam``,``DDDynamicReconfigure``.

The DDDValue class is a concrete class which should not be inherited, since it wraps physical values. 
Each instance stores 5 values: one for each type is can handle, and one to store the type.
When a value is instantiated, the value is stored in its raw form according to the chosen type,
and the rest stay with default values. When the value is accessed only then is the value converted (but not saved!)

The DDDParam interface class is an abstract class which should be implemented. 
In fact, this class is an extension to the DDParam interface.
Its basic implementations (int,double,bool,string) have already been implemented in the standard package.
These basic forms can also be further extended. For example, DDDEnum **extends** DDDInt because it has all of the features DDDInt has.
This can be done to other DDDParam implementations, and you can also further extend the extended classes (for example, DDDInvertibleEnum).
An example is given at the Extension section if you want to look more into this.
When any DDDParam implementation is extended, the user has access to everything within the object so that he can do what he needs to.

The DDDynamicReconfigure class is the concrete class that does the work against ROS and interfaces with the user.
Unlike DDDValue, this class can be extended, and it has an internal API that can aid users who wish to extend this class.
In fact, this class already is an extension to the DDynamicReconfigure class.
In the Extension section below this is elaborated. Keep in mind that extending DDDynamicReconfigure is not required.
While DDDynamicReconfigure allows extension, it does not provide full access to everything,
since the base functions of DDDynamic (and DDynamic) should not be modified.

### ROS Design

![](http://www.plantuml.com/plantuml/png/3OmxZiCm4CLwdo9x0KsH544gtyHGf5bRW3x6cpVGzQEA80lknWXwgzjVwQbwLaTzNEHj7n746Sx2oxzkYZGwjJEaqMJgVB_acG_eol_bTAYqQ5J8kWc67KPuOhYNRxlslFW3.png)

Unlike 1D-reconfigure and 2D-reconfigure, 3D-reconfigure is built on two publishers and 5 services:
the publishers are inherited from the base 2D-reconfigure, and one service is closed off yb 2D-reconfigure, but the rest 3D-reconfigure makes:

* ``desc_pub_`` publishes to topic "/parameter_descriptions", and is responsible for updating the descriptions of the parameter for commandline.
* ``update_pub_`` publishes to "/parameter_descriptions", and is responsible for updating the configuration values for commandline and client.

* ``set_service`` publishes and listens to requests on "/set_parameters", and is used to trigger parameter updates.
  This is held by 2D-reconfigure. It also contains the new parameters sent from client or commandline.

* ``lvl_service`` publishes and listens to requests on "/set_parameter_levels", and is used to trigger param level updates.
  It also contains the new severity levels sent from client or commandline.

* ``def_service`` publishes and listens to requests on "/set_parameter_defaults", and is used to trigger param default updates.
  It also contains the new defaults sent from client or commandline.

* ``min_service`` publishes and listens to requests on "/set_parameter_minimums", and is used to trigger param minimum updates.
  It also contains the new allowed maximums sent from client or commandline.

* ``max_service`` publishes and listens to requests on "/set_parameter_maximums", and is used to trigger param maximum updates.
  It also contains the new allowed minimums sent from client or commandline.

Since the DDDynamicReconfigure object is held on the server side, so are these ROS entities.

## Extension

***In all of these extensions, make sure to add the proper includes!***

### Adding a new Parameter type

To add a new parameter type, you must either:
* Extend one of the existing classes
* Implement the base class, ``DDDParam``.

In some cases, you might want your class to extend multiple classes, for example ``DDDIntVector`` both implements ``DDDVector`` and extends ``DDDInt``.
(``DDDVector`` does not exist in the standard param library).

Let us look into an example implementation of the param type "DDDIntEnforcer", which will update other parameters to its value when it updates.

```cpp
#ifndef DDDYNAMIC_RECONFIGURE_DDD_INT_ENFORCER_PARAM_H
#define DDDYNAMIC_RECONFIGURE_DDD_INT_ENFORCER_PARAM_H

#include <dddynamic_reconfigure/param/ddd_int_param.h>
#include <list>

namespace my_ddd_reconfig {
    // class definition
    class DDDIntEnforcer : public DDDInt {
    public:

        void setValue(Value val);
        
        // adds a parameter to be enforced by this param.
        DDDIntEnforcer &addEnforced(DDPtr param);
        
        // removes a parameter from being enforced by this param.
        void removeEnforced(DDPtr param);

        /**
         * creates a new int enforcer param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value
         * @param description details about the parameter
         * @param max the maximum allowed value. Defaults to INT32_MAX
         * @param min the minimum allowed value. Defaults to INT32_MIN
         */
        inline DDDIntEnforcer(const string &name, unsigned int level, const string &description,
                int def, int max = INT32_MAX, int min = INT32_MIN) :
                DDInt(name,level,description,def) {};

    protected:
        list<DDPtr> enforced_params_;
    };
    
    DDDIntEnforcer::setValue(Value val) {
        val_ = val.toInt();
        for(list<DDPtr>::iterator it = enforced_params_.begin(); it != enforced_params_.end(); ++it) {
            if(!enforced_params_[it].sameValue(val)) {
                enforced_params_[it].setValue(val);
            }
        }
    };
    
    DDDIntEnforcer &DDDIntEnforcer::addEnforced(DDPtr param) {
        enforced_params_.push_back(param);
        return *this;
    };
    
    void DDDIntEnforcer::removeEnforced(DDPtr param) {
        enforced_params_.remove(param);
    };
}

#endif //DDDYNAMIC_RECONFIGURE_DDD_INT_ENFORCER_PARAM_H
```

Notice how nothing within this class is private. This allows further extension of this class.
Moreover, notice that in here we are also using variables inherited from ``DDDInt``, specifically ``val_``.

### Extending DDDynamic's functions

Extending DDDynamicReconfigure means that you need additional functionality from the parameter server which 2D-reconfigure does not provide.
If that is the case, extending a class from DDDynamic gives you access to make new methods as need for the extra functionality,
and access to the following to make work with DDDynamic a bit easier:
* ``nh_``: this is the node handler used to create all publishers and subscribers in the parent class.
* ``params_`` this is the current parameter map 2D-reconfig uses to update parameters and add new ones.
* ``desc_pub_``: As explained before, this is the publisher responsible of updating the descriptions for the parameters and other metadata for the client and commandline.
* ``update_pub_``: This is the publisher responsible for updating the configuration values for the client and commandline.
* ``makeDescription()``: This is a helper method that generates a new Description message to be published by ``desc_pub_``.
  The message can be modified.
* ``makeConfiguration()``: This is a helper method that generates a new Description message to be published by ``update_pub_``.
  The message can be modified.
* ``internalCallback()``: This is a helper method that allows you to call the base param change callback built into 2D-reconfigure.
* ``internalPropCallback()``: This is a helper method that allows you to call any basic param property change callback built into 3D-reconfigure.

From there, it's your choice what to do with these.