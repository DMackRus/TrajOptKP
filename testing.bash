cd build
cmake --build .
cd ..
./build/devel/lib/automaticTOTaskSpecification/automaticTOTaskSpecification baseline
./build/devel/lib/automaticTOTaskSpecification/automaticTOTaskSpecification SI5
./build/devel/lib/automaticTOTaskSpecification/automaticTOTaskSpecification adaptive_jerk
./build/devel/lib/automaticTOTaskSpecification/automaticTOTaskSpecification SI10
./build/devel/lib/automaticTOTaskSpecification/automaticTOTaskSpecification SI20
./build/devel/lib/automaticTOTaskSpecification/automaticTOTaskSpecification magvel_change
./build/devel/lib/automaticTOTaskSpecification/automaticTOTaskSpecification iterative_error
