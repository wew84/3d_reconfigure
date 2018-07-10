//
// Created by Noam Dori on 5/07/18.
//
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dddynamic_reconfigure/param/ddd_int_param.h>

namespace dddynamic_reconfigure {

    /**
     * @brief preliminary test which makes sure we can use the object.
     */
    TEST(DDDIntTest, constructorTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDInt param1("param1",0,"param1",1);
        DDDInt param2("",0,"",1,100);
        DDDInt param3("\000",(unsigned int)-1,"param1", 1, -100, -10); // NOLINT(bugprone-string-literal-with-embedded-nul)
    }

    /**
     * @brief a test making sure we can handle all API for handling the values of the param
     */
    TEST(DDDIntTest, valueTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDInt param("ddd_param",0,"param1",1);
        // we won't do any tests on getLevel or getName, as those are implicit.
        Value v(1);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(1.0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(2);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value(2.0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        param.setValue(v);
        v = Value(3);
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getValue().getType() == "int");
        ASSERT_TRUE(param.sameValue(Value(2)));
    }

    /**
     * @brief a test making sure we can handle all API for handling the default values of the param
     */
    TEST(DDDIntTest, defaultTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // setLevel won't be tested since what it does is implicit.
        DDDInt param("ddd_param",0,"param1",1);
        Value v("1");

        param.setDefault(v);
        v = Value("3");
        ASSERT_NE(v.toInt(), param.getDefault().toInt()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getDefault().getType() == "int");
        ASSERT_EQ(1,param.getDefault().toInt());
    }

    /**
     * @brief a test making sure we can handle all API for handling the max & min values of the param
     */
    TEST(DDDIntTest, rangeTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // setLevel won't be tested since what it does is implicit.
        DDDInt param("ddd_param",0,"param1",1,2,0);
        ASSERT_FALSE(param.outOfMax());
        ASSERT_FALSE(param.outOfMin());

        // tests for max
        Value v("1");

        param.setMax(v);
        v = Value("3");
        ASSERT_NE(v.toInt(), param.getDefault().toInt()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getMax().getType() == "int");
        ASSERT_EQ(1,param.getDefault().toInt());

        ASSERT_FALSE(param.outOfMax());

        v = Value(0);
        param.setMax(v);
        ASSERT_TRUE(param.outOfMax());

        // tests for min
        v = Value("1");

        param.setMin(v);
        v = Value("3");
        ASSERT_NE(v.toInt(), param.getDefault().toInt()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getMin().getType() == "int");
        ASSERT_EQ(1,param.getDefault().toInt());

        ASSERT_FALSE(param.outOfMin());

        v = Value(2);
        param.setMin(v);
        ASSERT_TRUE(param.outOfMin());
    }

    TEST(DDDIntTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDInt param1("param1",0,"param1",1);
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