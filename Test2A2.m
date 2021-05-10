close all; clear all; clc; clf;

workspaceSize = [-1, 1, -1, 1, -0.012, 1];
dobotBaseTransform = transl(0,0,0);
simulationToggle = 1;
realRobotToggle = 0;

workspace = Workspace(realRobotToggle, simulationToggle, dobotBaseTransform, workspaceSize);
ROSCom = RosPublish(realRobotToggle);
controller = Controller(workspace, ROSCom);

% controller.StoreContainer('Salt', 4)
% controller.StoreContainer('Salt2', 4)
% controller.RetrieveContainer('Salt')
% controller.StoreContainer('Chilli', 4)
% controller.StoreContainer('Basil', 2)
% controller.StoreContainer('Basil2', 3)
% controller.StoreContainer('Basil3', 2)
% controller.RetrieveContainer('Basil')
% controller.StoreContainer('Chilli2', 2)
% controller.StoreContainer('Chilli3', 4)
% controller.RetrieveContainer('Basil')

