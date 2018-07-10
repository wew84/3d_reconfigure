//
// Created by Noam Dori on 5/07/18.
//
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dddynamic_reconfigure/param/ddd_enum_param.h>

namespace dddynamic_reconfigure {

    /**
     * @brief preliminary test which makes sure we can use the object.
     */
    TEST(DDDEnumTest, constructorTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;
        DDDEnum param1("param1",0,"param1",0,dict);
        DDDEnum param2("",0,"","ONE",dict);
        DDDEnum param3("\000",0,"\000", 0, dict); // NOLINT(bugprone-string-literal-with-embedded-nul)
    }

    /**
     * @brief a test making sure we can handle all API for handling the values of the param
     */
    TEST(DDDEnumTest, valueTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;
        DDDEnum param("ddd_param",0,"ddd_param",1,dict);
        // we won't do any tests on getLevel or getName, as those are implicit.
        Value v(1);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value("ONE");
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(1.0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(10);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value("TEN");
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value(10.0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        param.setValue(v);
        v = Value(-1);
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens regarding int setValue

        ASSERT_TRUE(param.getValue().getType() == "int");
        ASSERT_TRUE(param.sameValue(Value(10)));

        param.setValue(v);
        param.setValue(Value("TEN"));
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens regarding string setValue

        ASSERT_TRUE(param.getValue().getType() == "int");
        ASSERT_TRUE(param.sameValue(Value(10)));

        // make sure setValue and sameValue can handle int-string values
        v = Value("10");
        ASSERT_TRUE(param.sameValue(v));
        param.setValue(v);

        ASSERT_TRUE(param.getValue().getType() == "int");
        ASSERT_TRUE(param.sameValue(Value(10)));

        // make sure setValue and sameValue can handle non-number non-dictionary strings
        v = Value("TWO");
        // 'two' is not in our dictionary, so we will attempt to place it in there using a hash conversion
        ASSERT_FALSE(param.sameValue(v));
        param.setValue(v);

        ASSERT_TRUE(param.getValue().getType() == "int");
        int hash = (int)boost::hash<string>()("TWO");
        ASSERT_TRUE(param.sameValue(Value(hash)));
    }

    /**
     * @brief a test making sure we can handle all API for handling the default values of the param
     */
    TEST(DDDEnumTest, defaultTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // setLevel won't be tested since what it does is implicit.
        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;
        DDDEnum param("ddd_param",0,"ddd_param",1,dict);

        // int-wise test
        Value v("1");

        param.setDefault(v);
        v = Value("3");
        ASSERT_NE(v.toInt(), param.getDefault().toInt()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getDefault().getType() == "int");
        ASSERT_EQ(1,param.getDefault().toInt());

        // string-wise test
        v = Value("ONE");

        param.setDefault(v);
        v = Value("3");
        ASSERT_NE(v.toInt(), param.getDefault().toInt()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getDefault().getType() == "int");
        ASSERT_EQ(1,param.getDefault().toInt());
    }

    /**
     * @brief a test making sure we can handle all API for handling the max & min values of the param
     */
    TEST(DDDEnumTest, rangeTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // setLevel won't be tested since what it does is implicit.
        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;
        DDDEnum param("ddd_param",0,"ddd_param",1,dict);
        ASSERT_FALSE(param.outOfMax());
        ASSERT_FALSE(param.outOfMin());

        // tests for max

        // int-wise test
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

        // string-wise test
        v = Value("ONE");

        param.setMax(v);
        v = Value("3");
        ASSERT_NE(v.toInt(), param.getDefault().toInt()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getMax().getType() == "int");
        ASSERT_EQ(1,param.getDefault().toInt());

        ASSERT_FALSE(param.outOfMax());

        v = Value("NEG-ONE");
        param.setMax(v);
        ASSERT_TRUE(param.outOfMax());

        // tests for min

        // int-wise test
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

        // string-wise test
        v = Value("ONE");

        param.setMin(v);
        v = Value("3");
        ASSERT_NE(v.toInt(), param.getDefault().toInt()); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getMin().getType() == "int");
        ASSERT_EQ(1,param.getDefault().toInt());

        ASSERT_FALSE(param.outOfMin());

        v = Value("TEN");
        param.setMin(v);
        ASSERT_TRUE(param.outOfMin());
    }

    /**
     * @brief tests that the dictionary API is working.
     */
    TEST(DDDEnumTest, dictionaryTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;
        DDDEnum param("ddd_param",0,"ddd_param",1,dict);

        ASSERT_TRUE(param.hasDefinition("ONE"));
        ASSERT_FALSE(param.hasDefinition("TWO"));

        dict["TWO"] = 2;
        ASSERT_FALSE(param.hasDefinition(dict.find("TWO")->first));

        param.addDefinition("TWO",2);
        ASSERT_TRUE(param.hasDefinition("TWO"));

        param.addDefinition("SIX",6,"\000"); // NOLINT(bugprone-string-literal-with-embedded-nul)
        ASSERT_TRUE(param.hasDefinition("SIX"));
        ASSERT_EQ(dict.end(),dict.find("SIX")); // for non-aliasing tests

        param.removeDefinition("TWO");
        ASSERT_FALSE(param.hasDefinition("TWO"));

        ASSERT_EQ(6,param.getDefinition("SIX"));

        param.removeDefinition("SIX");
        ASSERT_FALSE(param.hasDefinition("SIX"));

        ASSERT_EQ(1,param.getDefinition("SIX"));
    }

    TEST(DDDEnumTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;
        DDDEnum param1("param1",0,"param1",1,dict);
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