cmake_minimum_required(VERSION 3.5)
project(TrajOptKP)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -DNDEBUG") #-march-native flag causes seg fault when used with OSQP??
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(MUJOCO_DIR    "$ENV{MJ_HOME}")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    message(STATUS "OPENMP has been found")
endif()

set(Eigen3_FOUND True)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
if (TARGET Eigen3::Eigen)
    # Use the imported target
    message(STATUS "Eigen library found")
else()
    message(STATUS "could not find eigen library")
endif (TARGET Eigen3::Eigen)

set(Mujoco_INCLUDE_DIRS ${MUJOCO_DIR}/include/mujoco)
message(STATUS "Mujoco directory: " ${MUJOCO_DIR})
file(GLOB LIB_MUJOCO "${MUJOCO_DIR}/lib/libmujoco.so.*")
message(STATUS "parent directory: " ${PARENT_DIR})
message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})
message(STATUS "Mujoco include directory: " ${Mujoco_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
if (TARGET yaml-cpp::yaml-cpp)
    # Use the imported target
    message(STATUS "yaml library found")
else()
    message(STATUS "could not find yaml library")
endif (TARGET yaml-cpp::yaml-cpp)

set(PROJECT_INCLUDE_DIR ${PARENT_DIR}/${PROJECT_NAME}/include)
message(STATUS "project include directory: " ${PROJECT_INCLUDE_DIR})

# PNG writer
find_package(PNGwriter 0.7.0)

if(PNGwriter_FOUND)
    message(STATUS "PNG writer library found")
endif(PNGwriter_FOUND)

# OSQP
find_package(osqp REQUIRED)
if(osqp_FOUND)
    message(STATUS "OSQP library found")
else()
    message(STATUS "could not find OSQP library")
endif(osqp_FOUND)

find_package(OsqpEigen REQUIRED)
if(OsqpEigen_FOUND)
    message(STATUS "OSQP Eigen library found")
else()
    message(STATUS "could not find OSQP library")
endif(OsqpEigen_FOUND)

# Create core library for my project
add_library(core_lib STATIC
        src/Differentiator/Differentiator.cpp
        src/ModelTranslator/ModelTranslator.cpp
        src/PhysicsSimulators/MuJoCoHelper.cpp
        src/KeyPointGenerator/KeyPointGenerator.cpp
        src/StdInclude/StdInclude.cpp
        src/FileHandler/FileHandler.cpp
        )

target_include_directories(core_lib PUBLIC
        ${Mujoco_INCLUDE_DIRS}
        ${YAML_INCLUDE_DIRS}
        ${PROJECT_INCLUDE_DIR}
        )

target_link_libraries(core_lib
        Eigen3::Eigen
        ${LIB_MUJOCO}
        -lglfw
        GL
        ${YAML_CPP_LIBRARIES}
        )

# TODO - make some other libraries to speed up overall computation (optimiser library for instance)

# TrajOptKP executable
add_executable(${PROJECT_NAME} src/main.cpp
            src/Visualiser/Visualiser.cpp
            src/ModelTranslator/Reaching.cpp
            src/ModelTranslator/TwoDPushing.cpp
            src/Optimiser/Optimiser.cpp
            src/Optimiser/iLQR.cpp
#            src/Optimiser/PredictiveSampling.cpp
            src/ModelTranslator/Walker.cpp
#            src/Optimiser/GradDescent.cpp
            src/ModelTranslator/BoxSweep.cpp
            src/ModelTranslator/Acrobot.cpp
            src/ModelTranslator/Humanoid.cpp
            src/ModelTranslator/ThreeDPushing.cpp
            src/ModelTranslator/PushBaseClass.cpp
            src/ModelTranslator/Pentabot.cpp
            src/ModelTranslator/PistonBlock.cpp
            src/Optimiser/iLQR_SVR.cpp
            src/ModelTranslator/PushSoft.cpp
            src/ModelTranslator/SweepMultiple.cpp
            src/ModelTranslator/PlaceObject.cpp
            src/ModelTranslator/FloatingCube.cpp
            src/GAOptimalKeypoints.cpp
            src/ModelTranslator/ArticulatedContact.cpp
            src/ModelTranslator/anyMal.cpp
            src/ModelTranslator/ImpactLargeBox.cpp
            src/ModelTranslator/BimanualPickup.cpp
            src/Optimiser/SCVX.cpp
            src/ModelTranslator/Piston.cpp
            src/ModelTranslator/LWR_reaching.cpp)

add_executable(gen_testing_data src/GenTestingData.cpp
        src/Visualiser/Visualiser.cpp
        src/ModelTranslator/Reaching.cpp
        src/ModelTranslator/TwoDPushing.cpp
        src/Optimiser/Optimiser.cpp
        src/Optimiser/iLQR.cpp
        src/Optimiser/SCVX.cpp
        src/ModelTranslator/Walker.cpp
        src/ModelTranslator/BoxSweep.cpp
        src/ModelTranslator/Acrobot.cpp
        src/ModelTranslator/Humanoid.cpp
        src/ModelTranslator/ThreeDPushing.cpp
        src/ModelTranslator/PushBaseClass.cpp
        src/ModelTranslator/Pentabot.cpp
        src/ModelTranslator/PistonBlock.cpp
        src/ModelTranslator/PlaceObject.cpp
        src/ModelTranslator/ArticulatedContact.cpp
        src/ModelTranslator/ImpactLargeBox.cpp
        src/ModelTranslator/anyMal.cpp
        src/ModelTranslator/BimanualPickup.cpp)

# Benchmark computing dynamics derivatives code
add_executable(benchmark_derivatives src/benchmark_derivatives.cpp
        src/Visualiser/Visualiser.cpp
        src/ModelTranslator/TwoDPushing.cpp
        src/Optimiser/Optimiser.cpp
        src/Optimiser/iLQR.cpp
        src/ModelTranslator/PushBaseClass.cpp)

# Toy contact problem analysis script
add_executable(toy_contact_analysis src/toy_contact_analysis.cpp
        src/Visualiser/Visualiser.cpp
        src/Optimiser/Optimiser.cpp
        src/Optimiser/iLQR.cpp
        src/ModelTranslator/PistonBlock.cpp
        src/ModelTranslator/Acrobot.cpp
        src/ModelTranslator/ArticulatedContact.cpp
        src/ModelTranslator/BoxSweep.cpp
        src/ModelTranslator/PushBaseClass.cpp)

# Derivative Error Analysis Code
add_executable(derivative_error_analysis
        src/derivative_error_analysis.cpp
        src/Visualiser/Visualiser.cpp
        src/Optimiser/Optimiser.cpp
        src/Optimiser/iLQR.cpp
        src/ModelTranslator/Acrobot.cpp
        src/ModelTranslator/ArticulatedContact.cpp
        src/ModelTranslator/BoxSweep.cpp
        src/ModelTranslator/ImpactLargeBox.cpp
        src/ModelTranslator/PushBaseClass.cpp
        src/ModelTranslator/TwoDPushing.cpp
        src/ModelTranslator/Walker.cpp
        src/ModelTranslator/PlaceObject.cpp
        src/ModelTranslator/PistonBlock.cpp
        )

# Contact stiffness test code
add_executable(contact_stiffness_test
        src/contact_stiffness_test.cpp
        src/Visualiser/Visualiser.cpp
        src/Optimiser/Optimiser.cpp
        src/Optimiser/iLQR.cpp
        src/ModelTranslator/BoxSweep.cpp
        src/ModelTranslator/TwoDPushing.cpp
        src/ModelTranslator/PushBaseClass.cpp
        src/ModelTranslator/ImpactLargeBox.cpp
        )

# Figure 1 data generator script
add_executable(figure1_data_generator
        src/figure_1_data_generator.cpp
        src/Visualiser/Visualiser.cpp
        src/Optimiser/Optimiser.cpp
        src/Optimiser/iLQR.cpp
        src/ModelTranslator/Walker.cpp
        src/ModelTranslator/BoxSweep.cpp
        src/ModelTranslator/ImpactLargeBox.cpp
        src/ModelTranslator/TwoDPushing.cpp
        src/ModelTranslator/PushBaseClass.cpp)

include_directories(
        include
        ${MUJOCO_DIR}/include
)

target_include_directories(${PROJECT_NAME} PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} PNGwriter::PNGwriter OsqpEigen::OsqpEigen) # or osqp::osqp or osqp::osqpstatic

target_include_directories(gen_testing_data PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})
target_link_libraries(gen_testing_data core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} PNGwriter::PNGwriter OsqpEigen::OsqpEigen)

target_include_directories(benchmark_derivatives PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})
target_link_libraries(benchmark_derivatives core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} PNGwriter::PNGwriter)

target_include_directories(toy_contact_analysis PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})
target_link_libraries(toy_contact_analysis core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} PNGwriter::PNGwriter)

target_include_directories(derivative_error_analysis PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})
target_link_libraries(derivative_error_analysis core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} PNGwriter::PNGwriter)

target_include_directories(contact_stiffness_test PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})
target_link_libraries(contact_stiffness_test core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} PNGwriter::PNGwriter)

target_include_directories(figure1_data_generator PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})
target_link_libraries(figure1_data_generator core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} PNGwriter::PNGwriter)


# ------------------------------------------   TESTS   -------------------------------------------------------------
enable_testing()
# ------- Computing derivatives test -------------
add_executable(test_derivs src/tests/Derivs_Test.cpp
        src/tests/test_acrobot.h
        src/tests/3D_test_class.h
        src/tests/test_humanoid.h
        src/ModelTranslator/anyMal.cpp)

target_include_directories(test_derivs PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})

target_link_libraries(test_derivs core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} gtest)

add_test(NAME Derivs COMMAND test_derivs)

# ----------- Keypoint generation tests ------------
#add_executable(test_keypoints src/tests/Keypoints_Test.cpp
#        src/tests/test_acrobot.h
#        src/tests/3D_test_class.h)
#
#target_include_directories(test_keypoints PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})
#
#target_link_libraries(test_keypoints core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} gtest)
#
#add_test(NAME Keypoints COMMAND test_keypoints)

# ---------- Model translator tests ------------
add_executable(test_model_translator src/tests/ModelTranslator_Test.cpp
        src/tests/test_acrobot.h
        src/tests/3D_test_class.h
        src/ModelTranslator/anyMal.cpp
        src/tests/test_bimanual_pickup.h)

target_include_directories(test_model_translator PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})

target_link_libraries(test_model_translator core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} gtest)

add_test(NAME ModelTranslatorState COMMAND test_model_translator)

# ---------- Kinematic chain tests -------------
add_executable(test_kinematic_chain src/tests/kinematic_chain_Test.cpp
        src/tests/test_humanoid.h
        src/tests/3D_test_class.h
        src/tests/test_walker.h)

target_include_directories(test_kinematic_chain PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})

target_link_libraries(test_kinematic_chain core_lib Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} gtest)

add_test(NAME ModelTranslatorKin COMMAND test_kinematic_chain)