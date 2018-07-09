cmake_minimum_required(VERSION 2.8.3)
project(dddynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation dynamic_reconfigure ddynamic_reconfigure)

set(CMAKE_CXX_STANDARD 98)

############
## Catkin ##
############

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS)

#############
## Library ##
#############

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/dddynamic_reconfigure.cpp
        include/dddynamic_reconfigure/ddd_param.h
        src/param/ddd_int_param.cpp
        src/param/ddd_double_param.cpp
        src/param/ddd_bool_param.cpp
        src/param/ddd_string_param.cpp
        src/param/ddd_enum_param.cpp
        src/ddd_param.cpp
        src/ddd_ordered_param.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    ## DDDynamic tester
    add_rostest_gtest(dddynamic_reconfigure-test test/dddynamic_reconfigure.test test/test_dddynamic_reconfigure.cpp)
    target_link_libraries(dddynamic_reconfigure-test ${PROJECT_NAME})

    ## DDDParam tester
    foreach (param_type int double bool string enum)
        add_rostest_gtest(ddd_${param_type}-test test/ddd_param/ddd_${param_type}.test test/ddd_param/test_ddd_${param_type}.cpp)
        target_link_libraries(ddd_${param_type}-test ${PROJECT_NAME})
    endforeach ()

    ## Full scale tester
    add_executable(ddd_server test/ddd_full_scale/ddd_server.cpp)
    target_link_libraries(ddd_server ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(ddd_full_scale-test test/ddd_full_scale/ddd_full_scale.test test/ddd_full_scale/ddd_client.cpp)
    target_link_libraries(ddd_full_scale-test ${PROJECT_NAME})
endif(CATKIN_ENABLE_TESTING)