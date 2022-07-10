benchmark?=output/benchmark.csv
package?=grid_map_geo

format:
	Tools/fix_code_style.sh .

config:
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

build:
	catkin build ${package}

build-test:
	catkin build ${package} --no-deps -i --catkin-make-args tests

test: build-test
	Tools/run_tests.sh .
