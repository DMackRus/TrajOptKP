#!/bin/bash

# Function to run your program and check its exit status
run_program() {
    local dof=$1
    local K=$2

    while true; do
        # Run your program with the argument
        ./build/TrajOptKP gen_testing_data 80 2000 $dof $K

        # Check the exit status
        if [[ $? -eq 0 ]]; then
            echo "Program exited correctly with status 1. Proceeding to the next argument."
            break
        else
            echo "Program did not exit correctly. Retrying..."
            # Sleep for a short duration before retrying (optional)
            sleep 1
        fi
    done
}

cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

array=(0 0.1 0.5 1 5 10 20 50 100 500 1000000)

run_program 10 1
run_program 10 500
run_program 10 100
run_program 5 1
run_program 5 1000000000000000

#for i in "${array[@]}"
#do
#   run_program 5 $i
#done
#
#for i in "${array[@]}"
#do
#   run_program 10 $i
#done
#
#for i in "${array[@]}"
#do
#   run_program 0 $i
#done

#run_program 3 50
#run_program 3 50
#run_program 3 50
#run_program 3 50
#run_program 3 50

#for i in {0..15}; do
#  run_program $i 10
#done
#
##run_program 0 20
#
#for i in {0..15}; do
#  run_program $i 20
#done
#
#for i in {0..15}; do
#  run_program $i 50
#done

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
#./build/TrajOptKP gen_testing_data 200 10000 2 0.5
#./build/TrajOptKP gen_testing_data 200 10000 2 0.2
#./build/TrajOptKP gen_testing_data 200 10000 2 0.1
#
#./build/TrajOptKP gen_testing_data 200 10000 1 0.5
#./build/TrajOptKP gen_testing_data 200 10000 1 0.2
#./build/TrajOptKP gen_testing_data 200 10000 1 0.1

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

