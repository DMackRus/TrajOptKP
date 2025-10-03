mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

#./build/derivative_error_analysis acrobot 100 1000 5
./build/derivative_error_analysis impact_large_box 100 2000 8
