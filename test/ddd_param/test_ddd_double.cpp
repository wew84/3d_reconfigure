//
// Created by Noam Dori on 5/07/18.
//
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dddynamic_reconfigure/param/ddd_double_param.h>

namespace dddynamic_reconfigure {

    /**
     * @brief preliminary test which makes sure we can use the object.
     */
    TEST(DDDDoubleTest, constructorTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDDouble param1("param1",0,"param1",0.5);
        DDDDouble param2("",0,"",1,10);
        DDDDouble param3("\000",0,"\000", -0, -3.4e100, 43.5e20); // NOLINT(bugprone-string-literal-with-embedded-nul)
    }

    /**
     * @brief a test making sure we can handle all API for handling the values of the param
     */
    TEST(DDDDoubleTest, valueTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDDouble param("ddd_param",0,"ddd_param",1);
        // we won't do any tests on getLevel or getName, as those are implicit.
        Value v(1.0);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(1);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(2.0);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value(2);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        param.setValue(v);
        v = Value(3);
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getValue().getType() == "double");
        ASSERT_TRUE(param.sameValue(Value(2)));
    }

    /**
     * @brief a test making sure we can handle all API for handling the default values of the param
     */
    TEST(DDDDoubleTest, defaultTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // setLevel won't be tested since what it does is implicit.
        DDDDouble param("ddd_param",0,"param1",1);
        Value v("1");

        param.setDefault(v);
        v = Value("3");
        ASSERT_NE(v.toDouble(), param.getDefault().toDouble()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getDefault().getType() == "double");
        ASSERT_EQ(1.0,param.getDefault().toDouble());
    }

    /**
     * @brief a test making sure we can handle all API for handling the max & min values of the param
     */
    TEST(DDDDoubleTest, rangeTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // setLevel won't be tested since what it does is implicit.
        DDDDouble param("ddd_param",0,"param1",1,1.5,0.5);
        ASSERT_FALSE(param.outOfMax());
        ASSERT_FALSE(param.outOfMin());

        // tests for max
        Value v("1");

        param.setMax(v);
        v = Value("3");
        ASSERT_NE(v.toDouble(), param.getDefault().toDouble()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getMax().getType() == "double");
        ASSERT_EQ(1.0,param.getDefault().toDouble());

        ASSERT_FALSE(param.outOfMax());

        v = Value(0.9);
        param.setMax(v);
        ASSERT_TRUE(param.outOfMax());

        // tests for min
        v = Value("1");

        param.setMin(v);
        v = Value("3");
        ASSERT_NE(v.toDouble(), param.getDefault().toDouble()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getMin().getType() == "double");
        ASSERT_EQ(1.0,param.getDefault().toDouble());

        ASSERT_FALSE(param.outOfMin());

        v = Value(1.1);
        param.setMin(v);
        ASSERT_TRUE(param.outOfMin());
    }

    TEST(DDDDoubleTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDDDouble param1("param1",0,"param1",1.0);
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