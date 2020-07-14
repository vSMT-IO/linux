#!/bin/bash


#./perf stat -x, -o test.csv --append -e cycles,inst_retired.any,cpu_clk_unhalted.thread,resource_stalls.any,cycle_activity.stalls_total,int_misc.recovery_cycles_any --cpu=5 -p $1

echo "process id profiling-------------->" >> test.csv
./perf stat -x, -o test.csv --append -e cycles,inst_retired.any,cpu_clk_unhalted.thread,resource_stalls.any,cycle_activity.stalls_total,int_misc.recovery_cycles_any -p $1 sleep 1

echo "on logical core 5-------------->" >> test.csv
./perf stat -x, -o test.csv --append -e cycles,inst_retired.any,cpu_clk_unhalted.thread,resource_stalls.any,cycle_activity.stalls_total,int_misc.recovery_cycles_any --cpu=5 sleep 1


echo "on logical core 21-------------->" >> test.csv
./perf stat -x, -o test.csv --append -e cycles,inst_retired.any,cpu_clk_unhalted.thread,resource_stalls.any,cycle_activity.stalls_total,int_misc.recovery_cycles_any --cpu=21 sleep 1


echo "on logical core 12-------------->" >> test.csv
./perf stat -x, -o test.csv --append -e cycles,inst_retired.any,cpu_clk_unhalted.thread,resource_stalls.any,cycle_activity.stalls_total,int_misc.recovery_cycles_any --cpu=12 sleep 1
