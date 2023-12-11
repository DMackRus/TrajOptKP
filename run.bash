mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ..

if [ $# -eq 0 ]; then
    echo "Usage: $0 <task_config>"
    exit 1
fi

# Access the first argument
task=$1
./build/TrajOptKP $task
