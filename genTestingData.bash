#sleep 1h 30m
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

./build/TrajOptKP gen_testing_data 60 2000 0 0.01
./build/TrajOptKP gen_testing_data 60 2000 0 0.1
./build/TrajOptKP gen_testing_data 60 2000 0 1
./build/TrajOptKP gen_testing_data 60 2000 0 10

./build/TrajOptKP gen_testing_data 60 2000 1 0.01
./build/TrajOptKP gen_testing_data 60 2000 1 0.1
./build/TrajOptKP gen_testing_data 60 2000 1 1
./build/TrajOptKP gen_testing_data 60 2000 1 10

./build/TrajOptKP gen_testing_data 60 2000 2 0.01
./build/TrajOptKP gen_testing_data 60 2000 2 0.1
./build/TrajOptKP gen_testing_data 60 2000 2 1
./build/TrajOptKP gen_testing_data 60 2000 2 10

./build/TrajOptKP gen_testing_data 60 2000 4 0.01
./build/TrajOptKP gen_testing_data 60 2000 4 0.1
./build/TrajOptKP gen_testing_data 60 2000 4 1
./build/TrajOptKP gen_testing_data 60 2000 4 10