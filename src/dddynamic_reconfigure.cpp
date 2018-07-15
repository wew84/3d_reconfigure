#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCDFAInspection"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "modernize-loop-convert"
#pragma ide diagnostic ignored "modernize-use-auto"
//
// Created by Noam Dori on 5/07/18.
//
#include <dddynamic_reconfigure/dddynamic_reconfigure.h>
#include <dddynamic_reconfigure/ddd_ordered_param.h>
#include <boost/foreach.hpp>

using namespace boost;
namespace dddynamic_reconfigure {

    DDDynamicReconfigure::DDDynamicReconfigure(ros::NodeHandle &nh) : DDynamicReconfigure(nh), started_(false) {};

    void DDDynamicReconfigure::add(DDPtr param) {
        params_[param->getName()] = param;
        if(started_) {
            desc_pub_.publish(makeDescription());
            update_pub_.publish(makeConfig());
        }
    };

    void DDDynamicReconfigure::add(DDParam *param) {
        add(DDPtr(param));
    };

    void DDDynamicReconfigure::remove(DDPtr param) {
        remove(param->getName());
    };

    void DDDynamicReconfigure::remove(DDParam *param) {
        remove(param->getName());
    };

    void DDDynamicReconfigure::remove(string param_name) {
        params_.erase(param_name);
        if(started_) {
            desc_pub_.publish(makeDescription());
            update_pub_.publish(makeConfig());
        }
    };

    void DDDynamicReconfigure::start() {
        DDynamicReconfigure::start();

        function<bool(Reconfigure::Request& req, Reconfigure::Response& rsp)> def_callback =
                bind(&internalPropCallback,this,_1,_2,DEFAULT);
        def_service_ = nh_.advertiseService("set_parameter_defaults", def_callback); // this allows changes to the parameter defaults

        function<bool(Reconfigure::Request& req, Reconfigure::Response& rsp)> lvl_callback =
                bind(&internalPropCallback,this,_1,_2,LEVEL);
        lvl_service_ = nh_.advertiseService("set_parameter_levels", lvl_callback); // this allows changes to the parameter levels

        function<bool(Reconfigure::Request& req, Reconfigure::Response& rsp)> max_callback =
                bind(&internalPropCallback,this,_1,_2,MAX);
        max_service_ = nh_.advertiseService("set_parameter_maximums", max_callback); // this allows changes to the parameter maximums

        function<bool(Reconfigure::Request& req, Reconfigure::Response& rsp)> min_callback =
                bind(&internalPropCallback,this,_1,_2,MIN);
        min_service_ = nh_.advertiseService("set_parameter_minimums", min_callback); // this allows changes to the parameter minimums

        started_ = true;
    }

    bool DDDynamicReconfigure::internalPropCallback(DDDynamicReconfigure *obj, Reconfigure::Request &req,
                                                    Reconfigure::Response &rsp, Property prop) {
        // debug msg
        string prop_name;
        switch(prop) {
            default: {ROS_ERROR_STREAM("Asked to edit unknown property type. Ignored request."); return false;}
            case DEFAULT: {prop_name = "default"; break;}
            case LEVEL: {prop_name = "level"; break;}
            case MAX: {prop_name = "maximum"; break;}
            case MIN: {prop_name = "minimum"; break;}
        }
        ROS_DEBUG_STREAM("Changing [" << prop_name << "] properties of some dddynamic parameters");

        // actual work
        obj->update(req, prop);
        obj->desc_pub_.publish(obj->makeDescription()); // updates the descriptor
        return true;
    }

    void DDDynamicReconfigure::update(const Reconfigure::Request &req, Property prop) {
        // the ugly part of the code, since ROS does not provide a nice generic message. Oh well...
        Reconfigure max_min_reconfigure;
        bool reconfig_changed = false;
        BOOST_FOREACH(const IntParameter i,req.config.ints) {
            if(!reassign(params_, i.name, Value(i.value),prop)) {
                ROS_ERROR_STREAM("Variable [" << i.name << "] is not registered");
            }
            reconfig_changed |= manageMaxMin(max_min_reconfigure,prop,params_[i.name]);
        }
        BOOST_FOREACH(const DoubleParameter i,req.config.doubles) {
            if(!reassign(params_, i.name, Value(i.value),prop)) {
                ROS_ERROR_STREAM("Variable [" << i.name << "] is not registered");
            }
            reconfig_changed |= manageMaxMin(max_min_reconfigure,prop,params_[i.name]);
        }
        BOOST_FOREACH(const BoolParameter i,req.config.bools) {
            if(!reassign(params_, i.name, Value((bool)i.value),prop)) {
                ROS_ERROR_STREAM("Variable [" << i.name << "] is not registered");
            }
            reconfig_changed |= manageMaxMin(max_min_reconfigure,prop,params_[i.name]);
        }
        BOOST_FOREACH(const StrParameter i,req.config.strs) {
            if(!reassign(params_, i.name, Value(i.value),prop)) {
                ROS_ERROR_STREAM("Variable [" << i.name << "] is not registered");
            }
            reconfig_changed |= manageMaxMin(max_min_reconfigure,prop,params_[i.name]);
        }
        if(reconfig_changed) {
            if(DDynamicReconfigure::internalCallback(dynamic_cast<DDynamicReconfigure*>(this),
                    max_min_reconfigure.request,max_min_reconfigure.response)) {
                ROS_INFO_STREAM("Some variables were found to be beyond their allowed ranges, changed accordingly.");
            } else {
                ROS_ERROR_STREAM("Some variables were found to be beyond their allowed ranges, could not change them.");
            }
        }
    }

    bool DDDynamicReconfigure::reassign(DDMap &map, const string &name, const Value &value, Property property) {
        if(map.find(name) != map.end() && (property == LEVEL || map[name]->sameType(value))) { // if the param with the given name exists,
                                                                                               // and either you are modifying the level,
                                                                                               // or its same same type as the member param.
            if(DDDPtr old = dynamic_pointer_cast<DDDParam>(map[name])) {
                switch (property) {
                    default: {
                        ROS_WARN_STREAM("Asked to edit unknown property of parameter [" << name << "]. Ignored.");
                        return true;
                    }
                    case DEFAULT: {
                        old->setDefault(value);
                        return true;
                    }
                    case LEVEL: {
                        old->setLevel((unsigned int) value.toInt());
                        return true;
                    }
                    case MAX: {
                        if (old->isOrdered()) {
                            dynamic_pointer_cast<DDDOrdered>(old)->setMax(value);
                        } else {
                            ROS_WARN_STREAM("Asked to edit range property of parameter [" << name
                                                                                          << "], which does not have range properties. Ignored.");
                        }
                        return true;
                    }
                    case MIN: {
                        if (old->isOrdered()) {
                            dynamic_pointer_cast<DDDOrdered>(old)->setMin(value);
                        } else {
                            ROS_WARN_STREAM("Asked to edit range property of parameter [" << name
                                                                                          << "], which does not have range properties. Ignored.");
                        }
                        return true;
                    }
                }
            }
        } else {
            return false;
        }
    }

    bool DDDynamicReconfigure::manageMaxMin(Reconfigure& reconfigure, DDDynamicReconfigure::Property prop,
                                            const DDPtr &param) {
        if(shared_ptr<DDDOrdered> ordered = dynamic_pointer_cast<DDDOrdered>(dynamic_pointer_cast<DDDParam>(param)->copy())) {
            switch (prop) {
                default: { break; }
                case MAX: {
                    if (ordered->outOfMax()) {
                        ordered->setValue(ordered->getMax());
                        ordered->prepConfig(reconfigure.request.config);
                        return true;
                    }
                }
                case MIN: {
                    if (ordered->outOfMin()) {
                        ordered->setValue(ordered->getMin());
                        ordered->prepConfig(reconfigure.request.config);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    DDDPtr DDDynamicReconfigure::at3(const char *name) {
        return dddynamic_reconfigure::at3(params_,name);
    }

    DDDOrderedPtr DDDynamicReconfigure::at3o(const char *name) {
        return dddynamic_reconfigure::at3o(params_,name);
    }

    DDDPtr at3(const DDMap& map, const char *name) {
        if(DDDPtr ret = dynamic_pointer_cast<DDDParam>(at(map,name))) {return ret;} else {return DDDPtr();};
    }

    DDDOrderedPtr at3o(const DDMap& map, const char *name) {
        if(DDDOrderedPtr ret = dynamic_pointer_cast<DDDOrdered>(at(map,name))) {return ret;} else {return DDDOrderedPtr();};
    }
}
#pragma clang diagnostic pop