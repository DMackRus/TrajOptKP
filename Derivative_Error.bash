mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

#./build/derivative_error_analysis acrobot 100 2000 3
#./build/derivative_error_analysis walker_run 100 200 3
#./build/derivative_error_analysis box_sweep 100 1500 3
#./build/derivative_error_analysis impact_large_box 100 2000 8
#./build/derivative_error_analysis impact_large_box 100 2000 4
#./build/derivative_error_analysis pushing_no_clutter 100 1000 3

#./build/derivative_error_analysis pushing_low_clutter 100 1000 3
#./build/derivative_error_analysis pushing_moderate_clutter 100 1000 3

./build/derivative_error_analysis piston_block 100 1000 3