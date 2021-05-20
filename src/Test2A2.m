close all; clear all; clc; clf;

% Setup

workspaceSize = [-1, 1, -1, 1, -0.012, 1];
dobotBaseTransform = transl(0,0,0);
simulationToggle = 1;
realRobotToggle = 0;
kinectToggle = 0;

workspace = Workspace(realRobotToggle, simulationToggle, kinectToggle, dobotBaseTransform, workspaceSize);
kinect = Kinect(kinectToggle);
ROSCom = RosPublish(realRobotToggle);
controller = Controller(workspace, ROSCom, kinect);

% Change the view

viewSteps = 15;
view(-30, 30)
[az, el] = view;

pause(3)
for i = 1:viewSteps
    incrementZoom = linspace(1, 1.1, viewSteps);
    incrementAz = linspace(az, 30, viewSteps);
    incrementEl = linspace(el, 10, viewSteps);
    camzoom(incrementZoom(i));
    view(incrementAz(i), incrementEl(i));
    pause(0.05)
end
[az, el] = view;

%% Use DS4 for remote control simulation

% controller.StartRemoteControl(0,0);

%% Use DS4 for safety retreat simulation

% controller.SafetySymbolSimulation();

%% Use DS4 for collision avoidance simulation

% controller.CollisionAvoidanceSimulation();

%% Store and retrieval tests (uncomment to see some random storage and retrievals)



% initangle = rad2deg(tr2rpy(controller.kinect1.GetTargetRaw(0)));
% while(1)
%     angles = rad2deg(tr2rpy(controller.kinect1.GetTargetRaw(0)));
%     tr = controller.kinect1.GetTargetRaw(0);
%     trplot(tr)
%     pause()
% end


% controller.StoreContainer('Salt', 4)
% 
% for i = 1:viewSteps
%     incrementZoom = linspace(1, 1, viewSteps);
%     incrementAz = linspace(az, 150, viewSteps);
%     incrementEl = linspace(el, 10, viewSteps);
%     camzoom(incrementZoom(i));
%     view(incrementAz(i), incrementEl(i));
%     pause(0.05)
% end
% [az, el] = view;
% 
% controller.StoreContainer('Salt2', 4)
% 
% for i = 1:viewSteps
%     incrementZoom = linspace(1, 1, viewSteps);
%     incrementAz = linspace(az, -30, viewSteps);
%     incrementEl = linspace(el, 5, viewSteps);
%     camzoom(incrementZoom(i));
%     view(incrementAz(i), incrementEl(i));
%     pause(0.05)
% end
% [az, el] = view;
% 
% controller.RetrieveContainer('Salt')
% 
% for i = 1:viewSteps
%     incrementZoom = linspace(1, 1, viewSteps);
%     incrementAz = linspace(az, 30, viewSteps);
%     incrementEl = linspace(el, 10, viewSteps);
%     camzoom(incrementZoom(i));
%     view(incrementAz(i), incrementEl(i));
%     pause(0.05)
% end
% [az, el] = view;
% 
% controller.StoreContainer('Chilli', 4)
% controller.StoreContainer('Basil', 2)
% controller.StoreContainer('Basil2', 3)
% 
% for i = 1:viewSteps
%     incrementZoom = linspace(1, 1, viewSteps);
%     incrementAz = linspace(az, -30, viewSteps);
%     incrementEl = linspace(el, 5, viewSteps);
%     camzoom(incrementZoom(i));
%     view(incrementAz(i), incrementEl(i));
%     pause(0.05)
% end
% [az, el] = view;
% 
% controller.RetrieveContainer('Salt2')
% 
% for i = 1:viewSteps
%     incrementZoom = linspace(1, 1, viewSteps);
%     incrementAz = linspace(az, 30, viewSteps);
%     incrementEl = linspace(el, 10, viewSteps);
%     camzoom(incrementZoom(i));
%     view(incrementAz(i), incrementEl(i));
%     pause(0.05)
% end
% [az, el] = view;
% 
% controller.StoreContainer('Basil3', 2)
% 
% for i = 1:viewSteps
%     incrementZoom = linspace(1, 1, viewSteps);
%     incrementAz = linspace(az, -30, viewSteps);
%     incrementEl = linspace(el, 5, viewSteps);
%     camzoom(incrementZoom(i));
%     view(incrementAz(i), incrementEl(i));
%     pause(0.05)
% end
% [az, el] = view;
% 
% controller.RetrieveContainer('Basil')
% 
% for i = 1:viewSteps
%     incrementZoom = linspace(1, 1, viewSteps);
%     incrementAz = linspace(az, 30, viewSteps);
%     incrementEl = linspace(el, 10, viewSteps);
%     camzoom(incrementZoom(i));
%     view(incrementAz(i), incrementEl(i));
%     pause(0.05)
% end
% [az, el] = view;
% 
% controller.StoreContainer('Chilli2', 2)
% controller.StoreContainer('Chilli3', 4)
% 
% for i = 1:viewSteps
%     incrementZoom = linspace(1, 1, viewSteps);
%     incrementAz = linspace(az, -30, viewSteps);
%     incrementEl = linspace(el, 5, viewSteps);
%     camzoom(incrementZoom(i));
%     view(incrementAz(i), incrementEl(i));
%     pause(0.05)
% end
% [az, el] = view;
% 
% controller.RetrieveContainer('Basil3')
% controller.RetrieveContainer('Chilli2')
