mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

#./build/contact_stiffness_test box_sweep
#./build/contact_stiffness_test pushing_no_clutter

#./build/gen_testing_data Generate_openloop_data acrobot
#./build/gen_testing_data Generate_openloop_data pushing_no_clutter
#./build/gen_testing_data Generate_openloop_data box_sweep
#./build/gen_testing_data Generate_openloop_data pushing_low_clutter
#./build/gen_testing_data Generate_openloop_data walker_run
#./build/gen_testing_data Generate_openloop_data pushing_moderate_clutter
#./build/gen_testing_data Generate_openloop_data anyMal

#./build/gen_testing_data Generate_openloop_data impact_large_box

#./build/gen_testing_data Generate_asynchronus_mpc_data bimanual_pickup
#./build/gen_testing_data Generate_asynchronus_mpc_data anyMal
#./build/gen_testing_data Generate_asynchronus_mpc_data walker_run
./build/gen_testing_data Generate_asynchronus_mpc_data pushing_moderate_clutter


#./build/contact_stiffness_test impact_large_box