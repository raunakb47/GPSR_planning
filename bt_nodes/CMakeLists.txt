cmake_minimum_required(VERSION 3.8)
project(bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp_action REQUIRED)


set(dependencies
rclcpp
behaviortree_cpp_v3)

include_directories(include)

add_library(follow_person_bt_node SHARED src/FollowPerson.cpp)
list(APPEND plugin_libs follow_person_bt_node)

add_library(guide_person_bt_node SHARED src/GuidePerson.cpp)
list(APPEND plugin_libs guide_person_bt_node)

add_library(count_bt_node SHARED src/Count.cpp)
list(APPEND plugin_libs count_bt_node)

add_library(describe_object_bt_node SHARED src/DescribeObject.cpp)
list(APPEND plugin_libs describe_object_bt_node)

add_library(describe_person_bt_node SHARED src/DescribePerson.cpp)
list(APPEND plugin_libs describe_person_bt_node)

add_library(find_object_bt_node SHARED src/FindObject.cpp)
list(APPEND plugin_libs find_object_bt_node)

add_library(find_person_bt_node SHARED src/FindPerson.cpp)
list(APPEND plugin_libs find_person_bt_node)

add_library(listen_bt_node SHARED src/Listen.cpp)
list(APPEND plugin_libs listen_bt_node)

add_library(look_around_bt_node SHARED src/LookAround.cpp)
list(APPEND plugin_libs look_around_bt_node)

add_library(look_to_bt_node SHARED src/LookTo.cpp)
list(APPEND plugin_libs look_to_bt_node)

add_library(move_to_bt_node SHARED src/MoveTo.cpp)
list(APPEND plugin_libs move_to_bt_node)

add_library(offer_bt_node SHARED src/Offer.cpp)
list(APPEND plugin_libs offer_bt_node)

add_library(pick_object_bt_node SHARED src/PickObject.cpp)
list(APPEND plugin_libs pick_object_bt_node)

add_library(place_object_bt_node SHARED src/PlaceObject.cpp)
list(APPEND plugin_libs place_object_bt_node)

add_library(query_bt_node SHARED src/Query.cpp)
list(APPEND plugin_libs query_bt_node)

add_library(recognize_person_bt_node SHARED src/RecognizePerson.cpp)
list(APPEND plugin_libs recognize_person_bt_node)

add_library(speak_bt_node SHARED src/Speak.cpp)
list(APPEND plugin_libs speak_bt_node)



foreach(bt_plugin ${plugin_libs})
ament_target_dependencies(${bt_plugin} ${dependencies})
target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS
${plugin_libs}
EXPORT ${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
