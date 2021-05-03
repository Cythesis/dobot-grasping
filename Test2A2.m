close all; clear all; clc; clf;

workspace = Workspace();
ROSCom = RosPublish();
controller = Controller(workspace, ROSCom);

controller.StoreContainer('Salt', 2)
% controller.StoreContainer('Salt', 1)
% controller.StoreContainer('Salt', 4)
% controller.StoreContainer('Salt', 4)