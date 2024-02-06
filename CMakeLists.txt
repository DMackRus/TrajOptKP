cmake_minimum_required(VERSION 3.0.2)
project(TrajOptKP)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O3")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
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
#file(GLOB LIB_MUJOCO ${MUJOCO_DIR}/lib/libmujoco.so.2.3.2)
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

add_executable(${PROJECT_NAME} src/main.cpp
            src/PhysicsSimulators/PhysicsSimulator.cpp
            src/StdInclude/StdInclude.cpp
            src/PhysicsSimulators/MuJoCoHelper.cpp
            src/ModelTranslator/ModelTranslator.cpp
            src/Visualiser/Visualiser.cpp
            src/ModelTranslator/DoublePendulum.cpp
            src/ModelTranslator/Reaching.cpp
            src/ModelTranslator/TwoDPushing.cpp
            src/Differentiator/Differentiator.cpp
            src/Optimiser/Optimiser.cpp
            src/Optimiser/iLQR.cpp
            src/Optimiser/PredictiveSampling.cpp
            src/ModelTranslator/BoxFlick.cpp
            src/ModelTranslator/Walker.cpp
            src/FileHandler/FileHandler.cpp
            src/Optimiser/GradDescent.cpp
            src/ModelTranslator/BoxSweep.cpp
            src/ModelTranslator/Acrobot.cpp
            src/ModelTranslator/Hopper.cpp
            src/Testing.cpp
            src/KeyPointGenerator/KeypointGenerator.cpp)

include_directories(
        include
        ${MUJOCO_DIR}/include
)

target_include_directories(${PROJECT_NAME} PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})

target_compile_definitions(${PROJECT_NAME} PUBLIC ${BULLET_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES})
