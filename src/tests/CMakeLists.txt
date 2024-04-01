# ------------------------------------------   TESTS   -------------------------------------------------------------
add_executable(test_derivs ../../src/tests/Derivs_Test.cpp
        ../../src/Differentiator/Differentiator.cpp
        ../../src/ModelTranslator/ModelTranslator.cpp
        ../../src/ModelTranslator/Acrobot.cpp
        ../../src/tests/3D_test_class.h
        ../../src/PhysicsSimulators/MuJoCoHelper.cpp
        ../../src/StdInclude/StdInclude.cpp
        ../../src/FileHandler/FileHandler.cpp)

target_include_directories(test_derivs PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})

target_link_libraries(test_derivs Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} gtest)

add_test(Derivs test_derivs)

add_executable(test_keypoints ../../src/tests/Keypoints_Test.cpp
        ../../src/Differentiator/Differentiator.cpp
        ../../src/ModelTranslator/ModelTranslator.cpp
        ../../src/ModelTranslator/Acrobot.cpp
        ../../src/tests/3D_test_class.h
        ../../src/PhysicsSimulators/MuJoCoHelper.cpp
        ../../src/StdInclude/StdInclude.cpp
        ../../src/FileHandler/FileHandler.cpp
        ../../src/KeyPointGenerator/KeyPointGenerator.cpp)

target_include_directories(test_keypoints PUBLIC ${Mujoco_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS} ${PROJECT_INCLUDE_DIR})

target_link_libraries(test_keypoints Eigen3::Eigen ${LIB_MUJOCO} -lglfw libGL.so GL ${YAML_CPP_LIBRARIES} gtest)

add_test(Keypoints test_keypoints)