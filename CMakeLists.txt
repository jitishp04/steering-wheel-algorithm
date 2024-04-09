cmake_minimum_required(VERSION 3.2)
project(helloworld)
set(CMAKE_CXX_STANDARD 11)
add_executable(helloworld helloworld.cpp PrimeChecker.cpp)

enable_testing()
add_executable(helloworld-Runner TestPrimeChecker.cpp PrimeChecker.cpp)
add_test(NAME helloworld-Runner COMMAND helloworld-Runner)