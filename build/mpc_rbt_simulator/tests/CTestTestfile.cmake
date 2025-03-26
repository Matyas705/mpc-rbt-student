# CMake generated Testfile for 
# Source directory: /home/student/238362_tm/src/mpc-rbt-simulator/tests
# Build directory: /home/student/238362_tm/build/mpc_rbt_simulator/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(tiagoDriverTest "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/student/238362_tm/build/mpc_rbt_simulator/test_results/mpc_rbt_simulator/tiagoDriverTest.gtest.xml" "--package-name" "mpc_rbt_simulator" "--output-file" "/home/student/238362_tm/build/mpc_rbt_simulator/ament_cmake_gtest/tiagoDriverTest.txt" "--append-env" "LD_LIBRARY_PATH=mpc_rbt_simulator" "--command" "/home/student/238362_tm/build/mpc_rbt_simulator/tests/tiagoDriverTest" "--gtest_output=xml:/home/student/238362_tm/build/mpc_rbt_simulator/test_results/mpc_rbt_simulator/tiagoDriverTest.gtest.xml")
set_tests_properties(tiagoDriverTest PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/student/238362_tm/build/mpc_rbt_simulator/tests/tiagoDriverTest" TIMEOUT "900" WORKING_DIRECTORY "/home/student/238362_tm/src/mpc-rbt-simulator/tests" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/student/238362_tm/src/mpc-rbt-simulator/tests/CMakeLists.txt;1;ament_add_gtest;/home/student/238362_tm/src/mpc-rbt-simulator/tests/CMakeLists.txt;0;")
subdirs("../gtest")
