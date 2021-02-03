cmake_minimum_required(VERSION 3.9)
project(rigid2d)

# create the rigid2d library and add it's include directories
add_library(rigid2d src/rigid2d.cpp)
target_include_directories(rigid2d PUBLIC include/)
target_compile_features(rigid2d PUBLIC cxx_std_17) # enable C++ 17
target_compile_options(rigid2d PUBLIC -Wall -Wextra)

# create the executable  and link it with the rigid2d library
add_executable(rigid2d_main src/main.cpp)
target_link_libraries(rigid2d_main rigid2d)

enable_testing()
add_executable(rigid_test tests/tests.cpp)
target_link_libraries(rigid_test rigid2d)
add_test(NAME Test_of_Rigid2d COMMAND rigid_test)

find_package(Doxygen)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
doxygen_add_docs(doxygen include/ src/ README.md ALL)
