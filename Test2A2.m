close all; clear all; clc; clf;

workspaceSize = [-1, 1, -1, 1, -0.012, 1];
dobotBaseTransform = transl(0,0,0);
simulationToggle = 1;
realRobotToggle = 0;

workspace = Workspace(realRobotToggle, simulationToggle, dobotBaseTransform, workspaceSize);
ROSCom = RosPublish();
controller = Controller(workspace, ROSCom);

controller.StoreContainer('Salt', 2)
controller.StoreContainer('Salt2', 3)
controller.StoreContainer('Chilli', 1)
controller.RetrieveContainer('Salt')
controller.StoreContainer('Basil', 2)
controller.StoreContainer('Basil2', 4)
controller.StoreContainer('Basil3', 2)
controller.RetrieveContainer('Basil2')
controller.StoreContainer('Chilli2', 1)
controller.StoreContainer('Chilli3', 4)
controller.RetrieveContainer('Basil')
controller.StoreContainer('Chilli4', 1)
controller.StoreContainer('Basil4', 2)
