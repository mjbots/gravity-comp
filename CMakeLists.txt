include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        25c847a7b7b739aa00d4f9a1a30fb041f2ce7a7b
)

FetchContent_MakeAvailable(moteus)

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/openrobots")

find_package(pinocchio)

add_executable(gravity-comp1 gravity-comp1.cc)
target_link_libraries(gravity-comp1 moteus::cpp)

add_executable(gravity-comp2 gravity-comp2.cc)
target_link_libraries(gravity-comp2 moteus::cpp)
target_link_libraries(gravity-comp2 pinocchio::pinocchio)
