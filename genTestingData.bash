#sleep 1h 30m
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

./build/TrajOptKP gen_testing_data 2000 0 1
./build/TrajOptKP gen_testing_data 2000 0 10
./build/TrajOptKP gen_testing_data 2000 0 100
./build/TrajOptKP gen_testing_data 2000 0 1000

./build/TrajOptKP gen_testing_data 2000 1 10
./build/TrajOptKP gen_testing_data 2000 1 100
./build/TrajOptKP gen_testing_data 2000 1 1000

./build/TrajOptKP gen_testing_data 2000 2 1
./build/TrajOptKP gen_testing_data 2000 2 10
./build/TrajOptKP gen_testing_data 2000 2 100
./build/TrajOptKP gen_testing_data 2000 2 1000