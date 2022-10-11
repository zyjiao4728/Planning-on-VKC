#!/bin/bash
cd /home/jiao/BIGAI/vkc_ws/Planning-on-VKC
source devel/setup.zsh

for iter in {1..100}
do  
    # echo "Round $iter, Baseline: 0, Env: 1"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false"
    # roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false

    # echo "Round $iter, Baseline: 0, Env: 2"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false"
    # roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false

    echo "Round $iter, Baseline: 0, Env: 1"
    echo "roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false ompl:=true"
    roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false ompl:=true

    echo "Round $iter, Baseline: 0, Env: 2"
    echo "roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false ompl:=true"
    roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false ompl:=true

    # echo "Round $iter, Baseline: 0, Env: 1"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false longhorizon:=true"
    # roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false longhorizon:=true

    # echo "Round $iter, Baseline: 0, Env: 2"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false longhorizon:=true"
    # roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false longhorizon:=true

    # echo "Round $iter, Baseline: 1, Env: 1"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=1 rviz:=false"
    # roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=1 rviz:=false

    # echo "Round $iter, Baseline: 2, Env: 1"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=2 rviz:=false"
    # roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=2 rviz:=false

    # echo "Round $iter, Baseline: 1, Env: 2"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=1 rviz:=false"
    # roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=1 rviz:=false

    # echo "Round $iter, Baseline: 2, Env: 2"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=2 rviz:=false"
    # roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=2 rviz:=false
done