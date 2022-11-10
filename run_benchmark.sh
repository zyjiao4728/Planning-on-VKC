#!/bin/bash
cd /home/jiao/BIGAI/vkc_ws/Planning-on-VKC
source devel/setup.zsh

for iter in {1..100}
do  
    # echo "Round $iter, Household Env"
    # echo "roslaunch vkc_example household_env.launch taskid:=0 nruns:=5 longhorizon:=false"
    # roslaunch vkc_example household_env.launch taskid:=0 nruns:=5 longhorizon:=false steps:=40 rviz:=false

    echo "Round $iter, Household Env"
    echo "roslaunch vkc_example household_env.launch taskid:=0 nruns:=5 longhorizon:=true"
    roslaunch vkc_example household_env.launch taskid:=0 nruns:=5 longhorizon:=true steps:=40 rviz:=false

    # echo "Round $iter, Baseline: 0, Env: 1"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false longhorizon:=true"
    # roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false longhorizon:=true

    # echo "Round $iter, Baseline: 0, Env: 2"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false longhorizon:=true"
    # roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false longhorizon:=true

    # echo "Round $iter, Baseline: 0, Env: 1"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false"
    # roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false

    # echo "Round $iter, Baseline: 0, Env: 2"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false"
    # roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false

    # echo "Round $iter, Baseline: 0, Env: 1"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false ompl:=true"
    # roslaunch vkc_example benchmark_env.launch envid:=1 baseline:=0 rviz:=false ompl:=true

    # echo "Round $iter, Baseline: 0, Env: 2"
    # echo "roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false ompl:=true"
    # roslaunch vkc_example benchmark_env.launch envid:=2 baseline:=0 rviz:=false ompl:=true

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

    # for taskid in {1..6}
    # do
    #     echo "Round $iter, Baseline: 0, taskid: $taskid"
    #     echo "roslaunch vkc_example urdf_scene_env.launch taskid:=$taskid"
    #     roslaunch vkc_example urdf_scene_env.launch rviz:=false taskid:=$taskid baseline:=0

    #     echo "Round $iter, Baseline: 1, taskid: $taskid"
    #     echo "roslaunch vkc_example urdf_scene_env.launch taskid:=$taskid"
    #     roslaunch vkc_example urdf_scene_env.launch rviz:=false taskid:=$taskid baseline:=1

    #     echo "Round $iter, Baseline: 2, taskid: $taskid"
    #     echo "roslaunch vkc_example urdf_scene_env.launch taskid:=$taskid"
    #     roslaunch vkc_example urdf_scene_env.launch rviz:=false taskid:=$taskid baseline:=2
    # done
done