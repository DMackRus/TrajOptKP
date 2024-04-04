#sleep 1h 30m
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

./build/TrajOptKP gen_testing_data 50 2000
./build/TrajOptKP gen_testing_data 100 2000
./build/TrajOptKP gen_testing_data 150 2000

