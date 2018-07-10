//
// Created by Noam Dori on 5/07/18.
//
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dddynamic_reconfigure/param/ddd_bool_param.h>

namespace dddynamic_reconfigure {

    /**
     * @brief preliminary test which makes sure we can use the object.
     */
    TEST(DDDBoolTest, constructorTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDBool param1("param1",0,"param1",true);
        DDDBool param2("",0,"",false);
        DDDBool param3("\000",0,"\000",false); // NOLINT(bugprone-string-literal-with-embedded-nul)
    }

    /**
     * @brief a test making sure we can handle all API for handling the values of the param
     */
    TEST(DDDBoolTest, valueTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDBool param("ddd_param",0,"ddd_param",true);
        // we won't do any tests on getLevel or getName, as those are implicit.
        Value v(true);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(1);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(false);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value(0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        param.setValue(v);
        v = Value(true);
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getValue().getType() == "bool");
        ASSERT_TRUE(param.sameValue(Value(false)));
    }

    /**
     * @brief a test making sure we can handle all API for handling the default values of the param
     */
    TEST(DDDBoolTest, defaultTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // setLevel won't be tested since what it does is implicit.
        DDDBool param("ddd_param",0,"ddd_param",true);
        Value v(false);

        param.setDefault(v);
        v = Value(true);
        ASSERT_NE(v.toBool(), param.getDefault().toBool()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getDefault().getType() == "bool");
        ASSERT_EQ(false,param.getDefault().toBool());
    }

    TEST(DDDBoolTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDBool param1("param1",0,"param1",true);
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