mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

./build/gen_testing_data Generate_openloop_data acrobot
./build/gen_testing_data Generate_openloop_data pushing_no_clutter
./build/gen_testing_data Generate_openloop_data pushing_low_clutter
./build/gen_testing_data Generate_openloop_data pushing_moderate_clutter
./build/gen_testing_data Generate_openloop_data box_sweep
./build/gen_testing_data Generate_openloop_data impact_large_box
./build/gen_testing_data Generate_openloop_data walker_run