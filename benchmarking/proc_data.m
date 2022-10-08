clear all; close all; clc;
N = 100;

% open_door_push_vkc_long = csvread("open_door_push_vkc_long.csv");
% sum(open_door_push_vkc_long(:,4)>0) / N * 100
% open_drawer_pull_vkc_long = csvread("open_drawer_pull_vkc_long.csv");
% sum(open_drawer_pull_vkc_long(:,4)>0) / N * 100
open_door_pull_bl1 = csvread("open_door_pull_bl1.csv");
sum(open_door_pull_bl1(:,7)) / N * 100
open_door_pull_bl2 = csvread("open_door_pull_bl2.csv");
sum(open_door_pull_bl2(:,7)) / N * 100
open_door_push_vkc = csvread("open_door_push_vkc.csv");
sum(open_door_push_vkc(:,4)>0) / N * 100
open_drawer_pull_bl1 = csvread("open_drawer_pull_bl1.csv");
sum(open_drawer_pull_bl1(:,7)) / N * 100
open_drawer_pull_bl2 = csvread("open_drawer_pull_bl2.csv");
sum(open_drawer_pull_bl2(:,7)) / N * 100
open_drawer_pull_vkc = csvread("open_drawer_pull_vkc.csv");
sum(open_drawer_pull_vkc(:,4)>0) / N * 100