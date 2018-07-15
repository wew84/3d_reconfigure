//
// Created by Noam Dori on 5/07/18.
//
#include <ros/ros.h>
#include <dddynamic_reconfigure/dddynamic_reconfigure.h>
#include <dddynamic_reconfigure/param/ddd_all_params.h>
#include <ddynamic_reconfigure/TutorialParams.h>

/**
  Topics:
  * /dd_server/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
  * /dd_server/parameter_updates [dynamic_reconfigure/Config]

  Services:
  * /dd_server/set_parameters:  dynamic_reconfigure/Reconfigure
*/
using namespace dddynamic_reconfigure;
using namespace ros;
using namespace dddynamic_reconfigure;

bool paramDefaultService(TutorialParams::Request& req, TutorialParams::Response& res, DDDynamicReconfigure& ddd) {
    res.int_param = ddd.at3o("int_param")->getDefault().toInt();
    res.double_param = ddd.at3("double_param")->getDefault().toDouble();
    res.str_param = ddd.at3("str_param")->getDefault().toString();
    res.enum_param = ddd.at3("enum_param")->getDefault().toInt();
    return true;
}

bool paramService(TutorialParams::Request& req, TutorialParams::Response& res, DDDynamicReconfigure& ddd) {
    res.int_param = ddd.get("int_param").toInt();
    res.double_param = ddd.get("double_param").toDouble();
    res.str_param = ddd.get("str_param").toString();
    res.enum_param = ddd.get("enum_param").toInt();
    return true;
}

void callback(const DDDMap& map, int) {
    ROS_INFO("Reconfigure Request: %d %f %s %s %ld",
            get(map, "int_param").toInt(),
            get(map, "double_param").toDouble(),
            get(map, "str_param").toString().c_str(),
            get(map, "bool_param").toBool() ? "True" : "False",
            map.size());
}

int main(int argc, char **argv) {
    // ROS init stage
    init(argc, argv, "dd_server");
    NodeHandle nh;

    // DDDynamic setup stage
    DDDynamicReconfigure ddd(nh);
    ddd.add(new DDDInt("int_param", 0, "An Integer parameter", 0, 50, 100));
    ddd.add(new DDDDouble("double_param", 0, "A double parameter", .5, 0, 1));
    ddd.add(new DDDString("str_param", 0, "A string parameter", "Hello World"));
    ddd.add(new DDDBool("bool_param", 0, "A Boolean parameter", true));
    map<string, int> dict;
        dict["Small"] = 0;
        dict["Medium"] = 1;
        dict["Large"] = 2;
        dict["ExtraLarge"] = 3;
    ddd.add(new DDDEnum("enum_param", 0, "A size parameter which is edited via an enum", 1, dict));
    ddd.start(DDDFunc(callback));

    // Actual Server Node code
    ROS_INFO("Spinning node");
    function<bool(TutorialParams::Request &, TutorialParams::Response &)> f = bind(paramService, _1, _2, ddd);
    ServiceServer checkParam = nh.advertiseService("get_params", f);
    function<bool(TutorialParams::Request &, TutorialParams::Response &)> f2 = bind(paramDefaultService, _1, _2, ddd);
    ServiceServer checkDefaultParam = nh.advertiseService("get_param_defaults", f2);
    MultiThreadedSpinner spinner(3);
    spinner.spin();
    return 0;
}

