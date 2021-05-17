%%              Environment Simulation Class - Chef's Third Arm
%
% This class handles all of the environmental features for simulation and
% running the 'Chef's Third Arm' automated pantry.
%
% CALLABLE CLASS FUNCTIONS: 
%
% Constructor function:
% self = Workspace(realRobotToggleInput, simulationToggleInput, kinectToggleInput, baseTransformInput, workspaceInput)
%           - Input a toggle for running the real robot, running the
%           simulation and using the real kinect. Toggles are 0 (off) or 1
%           (on). Optional arguments: baseTransformInput (transform of the
%           Dobot, typically transl(0,0,0), must be 4x4 homog. matrix.
%           workspaceInput: size for the workspace - matrix of [x, -x, y,
%           -y, z, -z]. 
%
% Function to animate a motion of a container only
% AnimateContainer(containerIndex, startTransform, endTransform, steps)
%           - Inputs: index of the container to be animated, start
%           transform (4x4 homog.), end transform (4x4 homog.), number of
%           steps for the animation
%
% Function to animate the motion of the Dobot only
% AnimateDobot(currentJointAngles, finalJointAngles, steps)
%           - Inputs: 1x6 joint matrix of starting position, 1x6 joint
%           matrix of final position, number of steps for the animation
%
% Function to simultaneously animate the motion of the Dobot and a given container
% AnimateSimulatenously(containerIndex, currentJointAngles, finalJointAngles, steps)
%           - Inputs: index of the container to be animated, 1x6 joint 
%           matrix of starting position, 1x6 joint matrix of final 
%           position, number of steps for the animation
%

classdef Workspace < handle
    
    properties
        % workspace parameters
        workspaceSize;
        defaultWorkspaceSize = [-1, 1, -1, 1, -0.012, 1];
        defaultDobotBaseTransform = transl(0,0,0);
        Dobot1;
        initialJointAngles;
        simulationToggle = 1;
        realRobotToggle = 0;
        kinectToggle = 0;
        % Physical dimensions which can be called upon:
        % Platform
        platformOffset = [0, 0, 0];
        platformStepHeight = 0.052;
        % Conveyor:
        conveyorOffset = [0.35, 0.217, 0.052];
        conveyorHeight = 0.06;
        conveyorFullLength = 0.696;
        % Frame
        frameOffset = [0, 0.042, -0.012];
        shelf1Height = 0.0786;
        shelf1YOffset = 0.0924;
        shelf2Height = 0.2125;
        shelf2YOffset = 0.23;
        shelfXOffset = 0.06125;
        % Kinect
        kinectOffset = [-0.2, 0.217, 0.88275];
        % Containers
        maxNumContainers = 10;
        containerStorage = [];
        containerHeights = [0.056, 0.084, 0.085, 0.045];
        currentNumContainers = 0;
        shelf1NumStored = 0;
        shelf2NumStored = 0;
        maxContainerDia = 0.11875;
        shelfLocations = [];
        shelfFilledIndex = zeros(2, 5);
    end

    methods  
       %% Constructor function
       function self = Workspace(realRobotToggleInput, simulationToggleInput, kinectToggleInput, baseTransformInput, workspaceInput)
            % Make last two arguments as default, but three toggles must be specified
            if (nargin < 5)
                self.workspaceSize = self.defaultWorkspaceSize;
                dobotBaseTransform = self.defaultDobotBaseTransform;
            elseif (nargin < 3)
                disp("You must input a toggle for whether you are using a real robot, kinect and another " ...
                        + " toggle to determine if you wish to run a simulation");
                return
            else
                self.workspaceSize = workspaceInput;
                dobotBaseTransform = baseTransformInput;
            end
            % Ensure that at least either the real robot is being used or the
            % simulation is being used
            if (realRobotToggleInput == 0) && (simulationToggleInput == 0)
                disp("Either a simulation must be run or the real robot must be used." )
                return
            end
            % Set toggles to class properties
            self.simulationToggle = simulationToggleInput;
            self.realRobotToggle = realRobotToggleInput;
            self.kinectToggle = kinectToggleInput;
            % Run get workspace function to create simulated environment
            self.GetWorkspace(dobotBaseTransform);
            % Set up shelf locations
            for i = 1:(self.maxNumContainers/2)
               self.shelfLocations(i).shelf1 = [(-self.shelfXOffset - (self.maxContainerDia/2) - ((i-1) * self.maxContainerDia)) ...
                                                    , self.shelf1YOffset + (self.maxContainerDia/2), self.shelf1Height];
               self.shelfLocations(i).shelf2 = [(-self.shelfXOffset - (self.maxContainerDia/2) - ((i-1) * self.maxContainerDia)) ...
                                                    , self.shelf2YOffset + (self.maxContainerDia/2), self.shelf2Height];
            end
       end
       
       %% Workspace plotting function
       function GetWorkspace(self, dobotBaseTransform)
           % Determine environment dimensions
           axis(self.workspaceSize, 'normal');
           % Place static ply objects
           hold on
           PlaceObject('Frame.ply', self.frameOffset);
           PlaceObject('Platform.ply', self.platformOffset);
           PlaceObject('ConveyorBelt.ply', self.conveyorOffset);
           PlaceObject('Kinect.ply', self.kinectOffset);
           % Set up Dobot object from Dobot.m class
           self.Dobot1 = Dobot(dobotBaseTransform, self.workspaceSize);
           self.initialJointAngles = self.Dobot1.model.getpos;
           self.initialJointAngles(1) = -0.5;
           self.Dobot1.model.animate(self.initialJointAngles);
           % Set up environment perspective
           camlight(1, 30);
           view(45, 30);
       end
       %% Function to run when adding container; returns the new container's storage transform
       function [storageLocation, containerIndex] = AddContainer(self, containerLabel, containerType)
            % Check that there is space for new container for its respective
            % shelf, depending on container type
            if (containerType == 1) || (containerType == 4)
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
            % Set container index based on the current number of containers
            containerIndex = (self.currentNumContainers + 1);
            % Set up new container as a single link for SerialLink object
            containerLink = Link('alpha',0,'a',0.001,'d',0,'offset',0);
            % Create empty tag field to be filled by kinect later
            self.containerStorage(containerIndex).tag = [];
            % Set container type field and lable field, and assign the
            % SerialLink; set initial base transform
            self.containerStorage(containerIndex).type = containerType;
            self.containerStorage(containerIndex).label = containerLabel;
            self.containerStorage(containerIndex).model = SerialLink(containerLink, 'name', containerLabel);
            self.containerStorage(containerIndex).model.base = transl(self.conveyorOffset(1) + 0.3 ...
                        , self.conveyorOffset(2), self.conveyorOffset(3) + self.conveyorHeight ...
                        + self.containerHeights(containerType));
            % Obtain storage location for the given container index
            storageLocation = self.GetNewStorageLocation(containerIndex);
            % Run confirmation function to increment shelf storage values
            self.ConfirmContainerStored(containerIndex)
            % End the function if simulation is not toggled on
            if (self.simulationToggle == 0)
                return
            end
            % Set up ply data for the new container
            [faceData, vertexData, plyData] = plyread(['Container',num2str(containerType),'.ply'],'tri');
            self.containerStorage(containerIndex).model.faces = {faceData, []};
            self.containerStorage(containerIndex).model.points = {vertexData, []};
            [viewAz, viewEl] = view;
            plot3d(self.containerStorage(containerIndex).model, 0,'noarrow','workspace',self.workspaceSize,'delay',0,'view', [viewAz, viewEl]);
            hold on
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
            
       end
       %% Function to calculate new storage position; only called by AddContainer() function
       function storageLocation = GetNewStorageLocation(self, storageIndex)
           % Check the container type, and find a new location on the
           % respective shelf which is empty, and set it to filled. Store
           % the new position index.
           containerType = self.containerStorage(storageIndex).type;
            if (containerType == 1) || (containerType == 4) % Use shelf 2
                shelfIndex = 0;
                for i = 1:(self.maxNumContainers/2)
                    if (self.shelfFilledIndex(2, i) == 0)
                        shelfIndex = i;
                        self.shelfFilledIndex(2, i) = 1;
                        break
                    end
                end
                % Handle case where the shelf is already full
                if shelfIndex == 0
                    disp("Error: shelf was full when finding shelf location." )
                    return
                end
                % Store shelf position x,y,z location
                storeHeight = self.shelfLocations(shelfIndex).shelf2(3) + self.containerHeights(containerType);
                storeYOffset = self.shelfLocations(shelfIndex).shelf2(2);
                storeXOffset = self.shelfLocations(shelfIndex).shelf2(1);
            else % Use shelf 1
                shelfIndex = 0;
                for i = 1:(self.maxNumContainers/2)
                    if (self.shelfFilledIndex(1, i) == 0)
                        shelfIndex = i;
                        self.shelfFilledIndex(1, i) = 1;
                        break
                    end
                end
                % Handle case where the shelf is already full
                if shelfIndex == 0
                    disp("Error: shelf was full when finding shelf location." )
                    return
                end
                % Store shelf position x,y,z location
                storeHeight = self.shelfLocations(shelfIndex).shelf1(3) + self.containerHeights(containerType);
                storeYOffset = self.shelfLocations(shelfIndex).shelf1(2);
                storeXOffset = self.shelfLocations(shelfIndex).shelf1(1);
            end
            % Return the new shelf position as 4x4 homog. transform
            self.containerStorage(storageIndex).shelfIndex = shelfIndex;
            storageLocation = transl(storeXOffset, storeYOffset, storeHeight);
       end
       %% Function to update container storage variables, only called by AddContainer() function
       function ConfirmContainerStored(self, storageIndex)
            % Check container type
            containerType = self.containerStorage(storageIndex).type;
            % Increment total number of containers stored
            self.currentNumContainers = self.currentNumContainers + 1;
            % Increment number of containers on each shelf depending on
            % container type
            if (containerType == 1) || (containerType == 4) % Use shelf 2
                self.shelf2NumStored = (self.shelf2NumStored + 1);
            else % Use shelf 1
                self.shelf1NumStored = (self.shelf1NumStored + 1);
            end
       end
       %% Function for container-only animation
       function AnimateContainer(self, containerIndex, startTransform, endTransform, steps)
            % Generate a path along X, because this is only used for 
            % animating along the conveyor belt
            xPath = linspace(startTransform(1,4), endTransform(1,4), steps);
            % Animate selected container along each step of the path
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
            % Get a joint path from inital and final joint matrices
            jointPath = self.Dobot1.GetJointPathQQ(currentJointAngles, finalJointAngles, steps);
            % Animate each step
            for i = 1:steps
                self.Dobot1.model.animate(jointPath(i, :))
                drawnow();
                pause(0.001);
            end
       end
       %% Function for Container-Dobot simultaneous animation
       function AnimateSimulatenously(self, containerIndex, currentJointAngles, finalJointAngles, steps)
            % Get a joint path from inital and final joint matrices 
            jointPath = self.Dobot1.GetJointPathQQ(currentJointAngles, finalJointAngles, steps);
            % Animate each step and update given container base transform
            % to animate it simultaneously
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