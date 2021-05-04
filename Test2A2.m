close all; clear all; clc; clf;

workspace = Workspace();
ROSCom = RosPublish();
controller = Controller(workspace, ROSCom);

controller.StoreContainer('Salt', 4)
controller.StoreContainer('Pepper', 3)
controller.StoreContainer('Chilli', 1)
controller.StoreContainer('Basil', 2)
controller.StoreContainer('Basil2', 2)
controller.StoreContainer('Basil3', 2)
controller.StoreContainer('Chilli2', 1)
controller.StoreContainer('Chilli3', 1)
controller.StoreContainer('Chilli4', 1)
controller.StoreContainer('Basil4', 2)
