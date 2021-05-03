%% Construct a simulated workspace to model the existing Dobot workspace

classdef Workspace < handle
    
    properties
        % workspace parameters
        workspaceSize;
        defaultWorkspaceSize = [-1, 1, -1, 1, -0.012, 1];
        defaultDobotBaseTransform = transl(0,0,0);
        Dobot1;
        initialJointAngles;
        simulationToggle = 1;
        % Physical dimensions which can be called upon:
        % Platform
        platformOffset = [0, 0, 0];
        platformStepHeight = 0.052;
        % Conveyor:
        conveyorOffset = [0.35, 0.217, 0.052];
        conveyorHeight = 0.057;
        conveyorFullLength = 0.696;
        % Frame
        frameOffset = [0, 0.042, -0.012];
        shelf1Height = 0.075;
        shelf1YOffset = 0.1924;
        shelf2Height = 0.205;
        shelf2YOffset = 0.225;
        shelfXOffset = 0.02375;
        % Containers
        maxNumContainers = 16;
        containerStorage = [];
        containerHeights = [0.056, 0.084, 0.085, 0.045];
        currentNumContainers = 0;
        shelf1NumStored = 0;
        shelf2NumStored = 0;
        maxContainerDia = 0.11875;
    end

    methods  
       %% Constructor function
       function self = Workspace(baseTransformInput, workspaceInput, simulationToggle)
           switch nargin
                case 3
                    self.workspaceSize = workspaceInput;
                    dobotBaseTransform = baseTransformInput;
                    self.simulationToggle = simulationToggle;
                case 2
                    if isempty(workspaceInput)
                        self.workspaceSize = self.defaultWorkspaceSize;
                        dobotBaseTransform = baseTransformInput;
                    elseif isempty(baseTransformInput)
                        self.workspaceSize = workspaceInput;
                        dobotBaseTransform = self.defaultDobotBaseTransform;
                    end
                case 1
                    if isempty(workspaceInput)
                        self.workspaceSize = self.defaultWorkspaceSize;
                        dobotBaseTransform = baseTransformInput;
                    elseif isempty(baseTransformInput)
                        self.workspaceSize = workspaceInput;
                        dobotBaseTransform = self.defaultDobotBaseTransform;
                    end
                case 0
                    self.workspaceSize = self.defaultWorkspaceSize;
                    dobotBaseTransform = self.defaultDobotBaseTransform;
            end
           
           self.GetWorkspace(dobotBaseTransform);
           
       end
       
       %% Workspace plotting function
       function GetWorkspace(self, dobotBaseTransform)
           
           axis(self.workspaceSize, 'normal');
           
           hold on
           PlaceObject('Frame.ply', self.frameOffset);
           PlaceObject('Platform.ply', self.platformOffset);
           PlaceObject('ConveyorBelt.ply', self.conveyorOffset);
           
           self.Dobot1 = Dobot(dobotBaseTransform, self.workspaceSize);
           self.initialJointAngles = self.Dobot1.model.getpos;
           self.initialJointAngles(1) = -0.5;
           self.Dobot1.model.animate(self.initialJointAngles);
           
           camlight(1, 30);
           view(45, 30);
       end
       %% Test function
      
       %% Function to run when adding container; returns the new container's storage transform
       function [storageLocation, containerIndex] = AddContainer(self, containerLabel, containerType)
           if (containerType == 1) || (containerType == 2)
                if (self.shelf2NumStored >= (self.maxNumContainers/2))
                   disp("Storage has reached capacity for this container type. ")
                   return
                end
            else
                if (self.shelf1NumStored >= (self.maxNumContainers/2))
                   disp("Storage has reached capacity for this container type. ")
                   return
                end
            end
            containerIndex = (self.currentNumContainers + 1);
            containerLink = Link('alpha',0,'a',0.001,'d',0,'offset',0);
            self.containerStorage(containerIndex).type = containerType;
            self.containerStorage(containerIndex).label = containerLabel;
            self.containerStorage(containerIndex).model = SerialLink(containerLink, 'name', containerLabel);
            self.containerStorage(containerIndex).model.base = transl(self.conveyorOffset(1) + 0.3 ...
                        , self.conveyorOffset(2), self.conveyorOffset(3) + self.conveyorHeight ...
                        + self.containerHeights(containerType));
            storageLocation = self.GetNewStorageLocation(containerIndex);
            if (self.simulationToggle == 0)
                return
            end
            [faceData, vertexData, plyData] = plyread(['Container',num2str(containerType),'.ply'],'tri');
            self.containerStorage(containerIndex).model.faces = {faceData, []};
            self.containerStorage(containerIndex).model.points = {vertexData, []};
            plot3d(self.containerStorage(containerIndex).model, 0,'noarrow','workspace',self.workspaceSize,'delay',0);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end 
            handles = findobj('Tag', self.containerStorage(containerIndex).model.name);
            h = get(handles,'UserData');
            try
                h.link(1).Children.FaceVertexCData = [plyData.vertex.red ...
                                                      , plyData.vertex.green ...
                                                      , plyData.vertex.blue]/255;
                h.link(1).Children.FaceColor = 'interp';
            catch
                disp("No colour in ply file")
            end
            self.ConfirmContainerStored(containerIndex)
       end
       %% Function to calculate new storage position; only called by AddContainer() function
       function storageLocation = GetNewStorageLocation(self, storageIndex)
            containerType = self.containerStorage(storageIndex).type;
            if (containerType == 1) || (containerType == 4) % Use shelf 2
                storeHeight = self.shelf2Height + self.containerHeights(containerType);
                storeYOffset = self.shelf2YOffset + (self.maxContainerDia/2);
                storeXOffset = -self.shelfXOffset - (self.maxContainerDia/2) - (self.shelf2NumStored * self.maxContainerDia);
            else % Use shelf 1
                storeHeight = self.shelf1Height + self.containerHeights(containerType);
                storeYOffset = self.shelf1YOffset + (self.maxContainerDia/2);
                storeXOffset = -self.shelfXOffset - (self.maxContainerDia/2) - (self.shelf1NumStored * self.maxContainerDia);
            end
            storageLocation = transl(storeXOffset, storeYOffset, storeHeight);
       end
       %% Function to update container storage variables, only called by AddContainer() function
       function ConfirmContainerStored(self, storageIndex)
            containerType = self.containerStorage(storageIndex).type;
            self.currentNumContainers = self.currentNumContainers + 1;
            if (containerType == 1) || (containerType == 4) % Use shelf 2
                self.shelf2NumStored = (self.shelf2NumStored + 1);
            else % Use shelf 1
                self.shelf1NumStored = (self.shelf1NumStored + 1);
            end
       end
       %% Function for container-only animation
       function AnimateContainer(self, containerIndex, startTransform, endTransform, steps)
            xPath = linspace(startTransform(1,4), endTransform(1,4), steps);
            for i = 1:steps
                self.containerStorage(containerIndex).model.base = startTransform;
                self.containerStorage(containerIndex).model.base(1,4) = xPath(i);
                self.containerStorage(containerIndex).model.animate(0);
                drawnow();
                pause(0.001);
            end
       end
       %% Function for Dobot-only animation
       function AnimateDobot(self, currentJointAngles, finalJointAngles, steps)
            jointPath = self.Dobot1.GetJointPathQQ(currentJointAngles, finalJointAngles, steps);
            for i = 1:steps
                self.Dobot1.model.animate(jointPath(i, :))
                drawnow();
                pause(0.001);
            end
       end
       %% Function for Container-Dobot simultaneous animation
       function AnimateSimulatenously(self, containerIndex, currentJointAngles, finalJointAngles, steps)
            jointPath = self.Dobot1.GetJointPathQQ(currentJointAngles, finalJointAngles, steps);
            for i = 1:steps
                self.Dobot1.model.animate(jointPath(i, :))
                self.containerStorage(containerIndex).model.base = self.Dobot1.model.fkine(jointPath(i, :));
                self.containerStorage(containerIndex).model.animate(0);
                drawnow();
                pause(0.001);
            end
       end
   end
end