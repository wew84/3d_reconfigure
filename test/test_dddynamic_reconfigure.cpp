//
// Created by Noam Dori on 5/07/18.
//
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCDFAInspection"
#include <dddynamic_reconfigure/dddynamic_reconfigure.h>
#include <gtest/gtest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dddynamic_reconfigure/param/ddd_all_params.h>
#include <exception>
using namespace std;
namespace dddynamic_reconfigure {

    enum Property {
        VALUE,
        DEFAULT,
        MIN,
        MAX
    };

    Value getProp(DDDynamicReconfigure ddd, const char *name, Property prop) {
        shared_ptr<DDDParam> ptr = dynamic_pointer_cast<DDDParam>(ddd.at(name));
        switch (prop) { // the order these are placed matters, so be wary!
            case MIN: {if(ptr->isOrdered()){return dynamic_pointer_cast<DDDOrdered>(ddd.at(name))->getMin();}}
            case MAX: {if(ptr->isOrdered()){return dynamic_pointer_cast<DDDOrdered>(ddd.at(name))->getMax();}}
            default:
            case VALUE: {return ptr->getValue();}
            case DEFAULT: {return ptr->getDefault();}
        }
    }

    void callService(ros::NodeHandle &nh, const char* topic, dynamic_reconfigure::Reconfigure srv) {
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + topic, srv));
    }

    void runTestsOf(void(*test_case)(const char*,Property), bool is_ordered) {
        test_case("/set_parameters",VALUE);
        test_case("/set_parameter_defaults",DEFAULT);
        if(is_ordered) {
            test_case("/set_parameter_maximums", MAX);
            test_case("/set_parameter_minimums", MIN);
        }
    }

    TEST(DDDynamicReconfigureTest, mapTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDDynamicReconfigure ddd(nh); // gets our main class running

        DDParam* ptr = new DDDBool("exists",0,"",true);
        DDPtr dd_ptr = DDPtr(ptr);

        ddd.add(dd_ptr);
        ASSERT_NE(DDPtr(),ddd.at("exists"));

        ddd.remove(ptr);
        ASSERT_EQ(DDPtr(),ddd.at("exists"));

        ddd.remove(dd_ptr);
        ASSERT_EQ(DDPtr(),ddd.at("exists"));
    }

    void basicCallback(const DDMap& map, int, bool *flag) {
        *flag = true;
    }

    /**
     * @brief preliminary test which makes sure we can use callbacks
     */
    TEST(DDDynamicReconfigureTest, basicCallbackTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDDynamicReconfigure ddd(nh); // gets our main class running

        bool flag = false;
        DDFunc callback = bind(&basicCallback,_1,_2,&flag);
        ddd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        callService(nh,"/set_parameters",srv);
        ASSERT_TRUE(flag);
    }

    void intCallback(const DDMap& map, int) {
        ASSERT_EQ("int",at(map,"int_param")->getValue().getType());
    }

    void runIntTest(const char* topic, Property prop) {
        ros::NodeHandle nh("~");
        DDFunc callback = &intCallback;

        DDDynamicReconfigure ddd(nh);
        ddd.add(new DDDInt("int_param", 0,"int_param", 0));
        ddd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::IntParameter int_param;
        int_param.name = "int_param";

        int_param.value = (int)random();
        srv.request.config.ints.push_back(int_param);
        callService(nh,topic,srv);
        ASSERT_EQ(int_param.value, getProp(ddd,"int_param",prop).toInt());

        int_param.value = INT32_MAX;
        srv.request.config.ints.push_back(int_param);
        callService(nh,topic,srv);
        ASSERT_EQ(int_param.value, getProp(ddd,"int_param",prop).toInt());

        int_param.value = INT32_MIN;
        srv.request.config.ints.push_back(int_param);
        callService(nh,topic,srv);
        ASSERT_EQ(int_param.value, getProp(ddd,"int_param",prop).toInt());
    }

    /**
     * @brief tests that int parameters are registered properly
     */
    TEST(DDDynamicReconfigureTest, intTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        runTestsOf(&runIntTest,true);
    }

    void doubleCallback(const DDMap& map, int) {
        ASSERT_EQ("double",at(map,"double_param")->getValue().getType());
    }

    void runDoubleTest(const char* topic, Property prop) {
        ros::NodeHandle nh("~");
        DDFunc callback = &doubleCallback;

        DDDynamicReconfigure ddd(nh);
        ddd.add(new DDDDouble("double_param", 0,"double_param", 0));
        ddd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name = "double_param";

        double_param.value = (double)random();
        srv.request.config.doubles.push_back(double_param);
        callService(nh,topic,srv);
        ASSERT_EQ(double_param.value, getProp(ddd,"double_param",prop).toDouble());

        double_param.value = DBL_MAX;
        srv.request.config.doubles.push_back(double_param);
        callService(nh,topic,srv);
        ASSERT_EQ(double_param.value, getProp(ddd,"double_param",prop).toDouble());

        double_param.value = DBL_MIN;
        srv.request.config.doubles.push_back(double_param);
        callService(nh,topic,srv);
        ASSERT_EQ(double_param.value, getProp(ddd,"double_param",prop).toDouble());

        double_param.value = -DBL_MAX;
        srv.request.config.doubles.push_back(double_param);
        callService(nh,topic,srv);
        ASSERT_EQ(double_param.value, getProp(ddd,"double_param",prop).toDouble());

        double_param.value = -DBL_MIN;
        srv.request.config.doubles.push_back(double_param);
        callService(nh,topic,srv);
        ASSERT_EQ(double_param.value, getProp(ddd,"double_param",prop).toDouble());
    }

    /**
     * @brief tests that double parameters are registered properly
     */
    TEST(DDDynamicReconfigureTest, doubleTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        runTestsOf(&runDoubleTest,true);
    }

    void boolCallback(const DDMap& map, int) {
        ASSERT_EQ("bool",at(map,"bool_param")->getValue().getType());
    }

    void runBoolTest(const char* topic, Property prop) {
        ros::NodeHandle nh("~");
        DDFunc callback = &boolCallback;

        DDDynamicReconfigure ddd(nh);
        ddd.add(new DDDBool("bool_param", 0,"bool_param", false));
        ddd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::BoolParameter bool_param;
        bool_param.name = "bool_param";

        bool_param.value = (unsigned char)false;
        srv.request.config.bools.push_back(bool_param);
        callService(nh,topic,srv);
        ASSERT_EQ((bool)bool_param.value, getProp(ddd,"bool_param",prop).toBool());

        bool_param.value = (unsigned char)true;
        srv.request.config.bools.push_back(bool_param);
        callService(nh,topic,srv);
        ASSERT_EQ((bool)bool_param.value, getProp(ddd,"bool_param",prop).toBool());
    }

    /**
     * @brief tests that boolean parameters are registered properly
     */
    TEST(DDDynamicReconfigureTest, boolTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        runTestsOf(&runBoolTest,false);
    }

    void strCallback(const DDMap& map, int) {
        ASSERT_EQ("string",at(map,"string_param")->getValue().getType());
    }

    void runStringTest(const char* topic, Property prop) {
        ros::NodeHandle nh("~");
        DDFunc callback = &strCallback;

        DDDynamicReconfigure ddd(nh);
        ddd.add(new DDDString("string_param", 0,"string_param", ""));
        ddd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::StrParameter string_param;
        string_param.name = "string_param";

        string_param.value = string("\000"); // NOLINT(bugprone-string-literal-with-embedded-nul)
        srv.request.config.strs.push_back(string_param);
        callService(nh,topic,srv);
        ASSERT_EQ(string_param.value, getProp(ddd,"string_param",prop).toString());

        string_param.value = "";
        srv.request.config.strs.push_back(string_param);
        callService(nh,topic,srv);
        ASSERT_EQ(string_param.value, getProp(ddd,"string_param",prop).toString());

        string_param.value = "Hello World";
        srv.request.config.strs.push_back(string_param);
        callService(nh,topic,srv);
        ASSERT_EQ(string_param.value, getProp(ddd,"string_param",prop).toString());
    }

    /**
     * @brief tests that string parameters are registered properly
     */
    TEST(DDDynamicReconfigureTest, stringTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        runTestsOf(&runStringTest,false);
    }

    void enumCallback(const DDMap& map, int) {
        ASSERT_EQ("int",at(map,"enum_param")->getValue().getType());
    }

    void runEnumTest(const char* topic, Property prop) {
        ros::NodeHandle nh("~");
        DDFunc callback = &enumCallback;

        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;

        DDDynamicReconfigure ddd(nh);
        ddd.add(new DDDEnum("enum_param", 0,"enum_param", "ONE", dict));
        ddd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        dynamic_reconfigure::IntParameter int_enum;
        int_enum.name = "enum_param";

        int_enum.value = 1;
        srv.request.config.ints.push_back(int_enum);
        callService(nh,topic,srv);
        ASSERT_EQ(int_enum.value, getProp(ddd,"enum_param",prop).toInt());

        int_enum.value = 10;
        srv.request.config.ints.push_back(int_enum);
        callService(nh,topic,srv);
        ASSERT_EQ(int_enum.value, getProp(ddd,"enum_param",prop).toInt());

        int_enum.value = -1;
        srv.request.config.ints.push_back(int_enum);
        callService(nh,topic,srv);
        ASSERT_EQ(int_enum.value, getProp(ddd,"enum_param",prop).toInt());

        srv.request.config.ints.clear();
        dynamic_reconfigure::StrParameter str_enum;
        str_enum.name = "enum_param";

        str_enum.value = "ONE";
        srv.request.config.strs.push_back(str_enum);
        callService(nh,topic,srv);
        ASSERT_EQ(dict[str_enum.value], getProp(ddd,"enum_param",prop).toInt());

        str_enum.value = "TEN";
        srv.request.config.strs.push_back(str_enum);
        callService(nh,topic,srv);
        ASSERT_EQ(dict[str_enum.value], getProp(ddd,"enum_param",prop).toInt());

        str_enum.value = "NEG-ONE";
        srv.request.config.strs.push_back(str_enum);
        callService(nh,topic,srv);
        ASSERT_EQ(dict[str_enum.value], getProp(ddd,"enum_param",prop).toInt());
    }

    /**
     * @brief tests that int-enum parameters are registered properly
     */
    TEST(DDDynamicReconfigureTest, enumTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        runTestsOf(&runEnumTest,true);
    }

    void complexCallback(const DDMap& map, int level) {
        ASSERT_EQ(0, level);
        ASSERT_EQ(1, at(map,"int_param")->getValue().toInt());
        ASSERT_EQ(0.6, at(map,"double_param")->getValue().toDouble());
        ASSERT_EQ("Goodbye Home", at(map,"str_param")->getValue().toString());
        ASSERT_EQ(false, at(map,"bool_param")->getValue().toBool());
        ASSERT_EQ(3, at(map,"enum_param")->getValue().toInt());
    }

    void runComplexTest(const char* topic, Property) {
        ros::NodeHandle nh("~");
        DDDynamicReconfigure ddd(nh); // gets our main class running
        ddd.add(new DDDInt("int_param", 0, "An Integer parameter", 0, 50, 100));
        ddd.add(new DDDDouble("double_param", 0, "A double parameter", .5, 0, 1));
        ddd.add(new DDDString("str_param", 0, "A string parameter", "Hello World"));
        ddd.add(new DDDBool("bool_param", 0, "A Boolean parameter", true));
        map<string, int> dict; {
            dict["Small"] = 0;
            dict["Medium"] = 1;
            dict["Large"] = 2;
            dict["ExtraLarge"] = 3;
        }
        ddd.add(new DDDEnum("enum_param", 0, "A size parameter which is edited via an enum", 0, dict));
        ddd.start(complexCallback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        dynamic_reconfigure::IntParameter int_param;
        int_param.name = "int_param";
        int_param.value = 1;
        srv.request.config.ints.push_back(int_param);

        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name = "double_param";
        double_param.value = 0.6;
        srv.request.config.doubles.push_back(double_param);

        dynamic_reconfigure::BoolParameter bool_param;
        bool_param.name = "bool_param";
        bool_param.value = (unsigned char) false;
        srv.request.config.bools.push_back(bool_param);

        dynamic_reconfigure::StrParameter string_param;
        string_param.name = "str_param";
        string_param.value = "Goodbye Home";
        srv.request.config.strs.push_back(string_param);

        dynamic_reconfigure::StrParameter enum_param;
        enum_param.name = "enum_param";
        enum_param.value = "ExtraLarge";
        srv.request.config.strs.push_back(enum_param);

        callService(nh,topic,srv);

        bool flag = false;
        DDFunc callback = bind(&basicCallback,_1,_2,&flag);
        ddd.setCallback(callback);
        callService(nh,"/set_parameters",srv);
        ASSERT_TRUE(flag);

        flag = false;
        ddd.clearCallback();
        callService(nh,"/set_parameters",srv);
        ASSERT_FALSE(flag);
    }

    /**
     * @brief tests that dddynamic can handle complex callbacks and param lists.
     */
    TEST(DDDynamicReconfigureTest, callbackTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        runTestsOf(&runComplexTest,false);
    }

    class InternalClass {
    public:
        inline void internalCallback(const DDMap& map, int level) {}
    };

    /**
     * @brief tests that dddynamic can take member methods as callbacks
     */
    TEST(DDDynamicReconfigureTest, memberCallbackTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDDynamicReconfigure ddd(nh); // gets our main class running

        ddd.start(&InternalClass::internalCallback,new InternalClass);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        callService(nh,"/set_parameters",srv);
    }

    void levelCallback(const DDMap&, int level, int *flag) {
        *flag = level;
    }

    /**
     * @brief tests that dddynamic properly handles param change levels
     */
    TEST(DDDynamicReconfigureTest, levelTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        int flag = 0;
        DDFunc callback = bind(&levelCallback, _1, _2, &flag);

        DDDynamicReconfigure ddd(nh);
        int top = (int) random() % 5 + 5;
        unsigned int or_sum = 0, next;
        for (int i = 1; i < top; i++) {
            next = (unsigned int) random();
            or_sum |= next;
            ddd.add(new DDDInt((format("param_%d") % i).str(), next,"level_param", 0));
        }
        ddd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::IntParameter int_param;
        for (int i = 1; i < top; i++) {
            int_param.name = (format("param_%d") % i).str();
            int_param.value = 1;
            srv.request.config.ints.push_back(int_param);
        }

        callService(nh,"/set_parameters",srv);
        ASSERT_EQ(or_sum, flag);

        ddd.add(new DDInt("unchanged_param", 1,"unchanged_param", 0)); //u-int max means everything is 1, so the result must also be that.
        dynamic_reconfigure::IntParameter unchanged_param;
        unchanged_param.name = "unchanged_param";
        unchanged_param.value = 1;
        srv.request.config.ints.push_back(unchanged_param);

        callService(nh,"/set_parameters",srv);
        ASSERT_EQ(1, flag);

        callService(nh,"/set_parameters",srv);
        ASSERT_EQ(0, flag);
    }

    void badCallback(const DDMap&, int) {
        std::exception e;
        throw e; // NOLINT(cert-err09-cpp,cert-err61-cpp,misc-throw-by-value-catch-by-reference)
    }

    /**
     * @brief tests that dddynamic can properly handle exceptions
     */
    TEST(DDDynamicReconfigureTest, badCallbackTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDDynamicReconfigure ddd(nh); // gets our main class running
        ddd.start(badCallback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        callService(nh,"/set_parameters",srv);
        // this is the best way to see exceptions doesn't make the whole thing tumble
    }

    void missingCallback(const DDMap& map, int) {
        ASSERT_EQ(map.end(),map.find("int_param"));
        ASSERT_EQ(map.end(),map.find("double_param"));
        ASSERT_EQ(map.end(),map.find("bool_param"));
        ASSERT_EQ(map.end(),map.find("str_param"));
        ASSERT_EQ(map.end(),map.find("enum_param"));
    }

    /**
     * @brief tests that dddynamic can properly handle missing/unregistered parameters
     */
    TEST(DDDynamicReconfigureTest, unknownParamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDDynamicReconfigure ddd(nh); // gets our main class running
        ddd.start(missingCallback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        dynamic_reconfigure::IntParameter int_param;
        int_param.name = "int_param";
        int_param.value = 1;
        srv.request.config.ints.push_back(int_param);

        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name = "double_param";
        double_param.value = 0.6;
        srv.request.config.doubles.push_back(double_param);

        dynamic_reconfigure::BoolParameter bool_param;
        bool_param.name = "bool_param";
        bool_param.value = (unsigned char) false;
        srv.request.config.bools.push_back(bool_param);

        dynamic_reconfigure::StrParameter string_param;
        string_param.name = "str_param";
        string_param.value = "Goodbye Home";
        srv.request.config.strs.push_back(string_param);

        dynamic_reconfigure::StrParameter enum_param;
        enum_param.name = "enum_param";
        enum_param.value = "ExtraLarge";
        srv.request.config.strs.push_back(enum_param);

        callService(nh,"/set_parameters",srv);
    }

    /**
     * @brief tests that dddynamic's stream operator properly works
     */
    TEST(DDDynamicReconfigureTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDDynamicReconfigure ddd(nh); // gets our main class running
        DDDInt ddd_int("int_param", 0, "An Integer parameter", 0, 50, 100);
        DDDDouble ddd_double("double_param", 0, "A double parameter", .5, 0, 1);
        DDDString ddd_string("str_param", 0, "A string parameter", "Hello World");
        DDDBool ddd_bool("bool_param", 0, "A Boolean parameter", true);
        ddd.add(new DDDInt(ddd_int)); // note that using the address operator causes a segmentation fault when destructing objects!
        ddd.add(new DDDDouble(ddd_double));
        ddd.add(new DDDString(ddd_string));
        ddd.add(new DDDBool(ddd_bool));
        map<string, int> dict; {
            dict["Small"] = 0;
            dict["Medium"] = 1;
            dict["Large"] = 2;
            dict["ExtraLarge"] = 3;
        }
        DDDEnum ddd_enum("enum_param", 0, "A size parameter which is edited via an enum", 0, dict);
        ddd.add(new DDDEnum(ddd_enum));

        stringstream stream, explicit_stream;
        stream << ddd;

        explicit_stream << "{" << ddd_bool << "," << ddd_double << "," << ddd_enum << "," << ddd_int << "," << ddd_string << "}";
        ASSERT_EQ(explicit_stream.str(),stream.str());
    }
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "dddynamic_reconfigure_test");

    srand((unsigned int)random());

    return RUN_ALL_TESTS();
}
#pragma clang diagnostic pop