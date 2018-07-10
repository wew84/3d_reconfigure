//
// Created by Noam Dori on 5/07/18.
//
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dddynamic_reconfigure/param/ddd_string_param.h>

namespace dddynamic_reconfigure {

    /**
     * @brief preliminary test which makes sure we can use the object.
     */
    TEST(DDDStringTest, constructorTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDString param1("param1",0,"param1","Hello World");
        DDDString param2("",0,"","");
        DDDString param3("\000",0,"\000","\000"); // NOLINT(bugprone-string-literal-with-embedded-nul)
    }

    /**
     * @brief a test making sure we can handle all API for handling the values of the param
     */
    TEST(DDDStringTest, valueTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDString param("ddd_param",0,"param1","1");
        // we won't do any tests on getLevel or getName, as those are implicit.
        Value v("1");
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(1);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value("2");
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value(2);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        param.setValue(v);
        v = Value("3");
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getValue().getType() == "string");
        ASSERT_TRUE(param.sameValue(Value("2")));
    }

    /**
     * @brief a test making sure we can handle all API for handling the default values of the param
     */
    TEST(DDDStringTest, defaultTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // setLevel won't be tested since what it does is implicit.
        DDDString param("ddd_param",0,"param1","1");
        // we won't do any tests on getLevel or getName, as those are implicit.
        Value v("1");

        param.setDefault(v);
        v = Value("3");
        ASSERT_NE(v.toString(), param.getDefault().toString()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getDefault().getType() == "string");
        ASSERT_EQ("1",param.getDefault().toString());
    }

    TEST(DDDStringTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDString param1("param1",0,"param1","Hello World");
        stringstream stream;
        stream << param1;
        ASSERT_EQ(param1.getName() + ":" + param1.getValue().toString(),stream.str());
    }
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    srand((unsigned int)random());

    return RUN_ALL_TESTS();
}