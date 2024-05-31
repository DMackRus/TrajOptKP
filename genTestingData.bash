#sleep 1h 30m
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

# -------------------- Asyncronus MPC ------------------------------
# ---------------- Push soft into rigid ----------------------------
#./build/TrajOptKP gen_testing_data 20 2000 4 0.5
#./build/TrajOptKP gen_testing_data 20 2000 4 0.2
#./build/TrajOptKP gen_testing_data 20 2000 4 0.1

#./build/TrajOptKP gen_testing_data 20 2000 2 0.5
#./build/TrajOptKP gen_testing_data 20 2000 2 0.2
#./build/TrajOptKP gen_testing_data 20 2000 2 0.1
#
#./build/TrajOptKP gen_testing_data 20 2000 1 0.5
#./build/TrajOptKP gen_testing_data 20 2000 1 0.2
#./build/TrajOptKP gen_testing_data 20 2000 1 0.1

# -------------------- Push soft ----------------------------------
./build/TrajOptKP gen_testing_data 20 2000 4 0.2
./build/TrajOptKP gen_testing_data 20 2000 4 0.5

./build/TrajOptKP gen_testing_data 20 2000 4 0.1

./build/TrajOptKP gen_testing_data 20 2000 2 0.5
./build/TrajOptKP gen_testing_data 20 2000 2 0.2
./build/TrajOptKP gen_testing_data 20 2000 2 0.1

./build/TrajOptKP gen_testing_data 20 2000 1 0.5
./build/TrajOptKP gen_testing_data 20 2000 1 0.2
./build/TrajOptKP gen_testing_data 20 2000 1 0.1

# -------------------- Push moderate clutter ---------------------
#./build/TrajOptKP gen_testing_data 60 2000 4 0.5
#./build/TrajOptKP gen_testing_data 60 2000 4 0.2
#./build/TrajOptKP gen_testing_data 60 2000 4 0.1

#./build/TrajOptKP gen_testing_data 60 2000 2 0.5
#./build/TrajOptKP gen_testing_data 60 2000 2 0.2
#./build/TrajOptKP gen_testing_data 60 2000 2 0.1
#
#./build/TrajOptKP gen_testing_data 60 2000 1 0.5
#./build/TrajOptKP gen_testing_data 60 2000 1 0.2
#./build/TrajOptKP gen_testing_data 60 2000 1 0.1

# -------------------- Open loop optimisation ------------------------------


#./build/TrajOptKP gen_testing_data 20 2000 4 0.5
#./build/TrajOptKP gen_testing_data 60 2000 4 0.05
#./build/TrajOptKP gen_testing_data 60 2000 4 0.1
#./build/TrajOptKP gen_testing_data 60 2000 4 0.5
#./build/TrajOptKP gen_testing_data 60 2000 4 1
#./build/TrajOptKP gen_testing_data 60 2000 4 1.5
#./build/TrajOptKP gen_testing_data 60 2000 4 2
#./build/TrajOptKP gen_testing_data 60 2000 4 2.5
#./build/TrajOptKP gen_testing_data 60 2000 4 3
#./build/TrajOptKP gen_testing_data 60 2000 4 5

#./build/TrajOptKP gen_testing_data 60 2000 0 1
#./build/TrajOptKP gen_testing_data 60 2000 1 1
#./build/TrajOptKP gen_testing_data 60 2000 2 1
#./build/TrajOptKP gen_testing_data 60 2000 3 1
#./build/TrajOptKP gen_testing_data 60 2000 4 1
#./build/TrajOptKP gen_testing_data 60 2000 5 1
#./build/TrajOptKP gen_testing_data 60 2000 6 1
#./build/TrajOptKP gen_testing_data 60 2000 7 1
#./build/TrajOptKP gen_testing_data 60 2000 8 1
#./build/TrajOptKP gen_testing_data 60 2000 9 1
#./build/TrajOptKP gen_testing_data 60 2000 10 1
#./build/TrajOptKP gen_testing_data 60 2000 11 1
#./build/TrajOptKP gen_testing_data 60 2000 12 1

# Openloop optimisation
#./build/TrajOptKP gen_testing_data 2000 1 0.01
#./build/TrajOptKP gen_testing_data 2000 1 0.1
#./build/TrajOptKP gen_testing_data 2000 1 1
#./build/TrajOptKP gen_testing_data 2000 1 2
#
#./build/TrajOptKP gen_testing_data 2000 2 0.01
#./build/TrajOptKP gen_testing_data 2000 2 0.1
#./build/TrajOptKP gen_testing_data 2000 2 1
#./build/TrajOptKP gen_testing_data 2000 2 2
#
#./build/TrajOptKP gen_testing_data 2000 4 0.01
#./build/TrajOptKP gen_testing_data 2000 4 0.1
#./build/TrajOptKP gen_testing_data 2000 4 1
#./build/TrajOptKP gen_testing_data 2000 4 2