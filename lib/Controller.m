%%              Controller Class - Chef's Third Arm
%
% This class comprises the core functionalities and features of the 'Chef's
% Third Arm' system. These functions are the higest level functions which
% utilize all other classes in order to operate the system. Once a
% 'controller' object has been created, all functions for the system should
% be called through it.
%
% CALLABLE CLASS FUNCTIONS:
%
% Constructor function:
% self = Controller(workspaceInput, ROSCommunicator, kinectInput)
%           - INPUTS: workspace object (of workspace.m class), ROS
%           communicator object (of RosPublish.m class), kinect object (of
%           kinect.m class)
%
% Container Storage function:
% StoreContainer(containerLabel, containerType)
%           - INPUTS: A character vector of the container label (eg.
%           'Salt'), a container type number (between 1 and 4)
%           - Outputs: The storage location based on currently stored
%           containers (4x4 homog. transform), and index for the new
%           container.
% 
% Container Retrieval function:
% RetrieveContainer(containerLabel)
%           - INPUTS: A character vector of the container label (eg.
%           'Salt') which has already been stored in the workspace
%
% Emergency Stop Function (For GUI input):
% EmergencyStop()
%
% Clear Emergency Stop Function (For GUI input):
% ClearEmergencyStop()
%
% Dobot Cartesian Position Jogging Function:
% currentJointAngles = JogPosition(jogAmount, jogParameter)
%           - INPUTS: amount to jog (in metres), and what direction to jog;
%           ie. 0: Linear rail, 1: X,  2: Y,  3: Z,  4: Roll,  5: Pitch    
%           6: Yaw
%           - OUPUTS: the resulting joint angles to reach position
%
% Dobot Joint Position Jogging Function:
% JogJoint(jogAmount, jogParameter)
%           - INPUTS: amount to jog (in metres for linear rail, in degrees)
%           for revolute joints, and what joint to jog;
%           ie. 0: Linear rail, 1: base joint,  2: forearm joint,  
%           3: reararm joint,  4: end-effector servo
%
% Function to commence remote controlling of Dobot. Includes joint and 
% cartesian control. Requires DS4 controller connection.
% StartRemoteControl(jointToggle, collisionsToggle)
%           - INPUTS: toggle to determine whether joints are controller or
%           cartesian (ie. 0: cartesian control, 1: joint control), toggle
%           to turn on or off collision checking for all containters
%
% Function to turn off remote control (For GUI)
% EndRemoteControl()
%
% Function to either simulate or control for real a linear rail command for Dobot:
% newJointAngles = LinearRailCommand(self, currentJointAngles, positionInput, steps)
%           - INPUTS:
%
% Function to either simulate or control for real a linear rail command with container for Dobot:
% newJointAngles = LinearRailCommandSimultaneous(self, containerIndex, currentJointAngles, positionInput, steps)
%           - INPUTS:
%
% Function to either simulate or control for real a joint command for the Dobot
% newJointAngles = JointCommand(self, currentJointAngles, finalJointAngles, steps)
%           - INPUTS:
%
% Function to either simulate or control for real a joint command with a container for the Dobot
% newJointAngles = JointCommandSimultaneous(self, containerIndex, currentJointAngles, finalJointAngles, steps)
%           -INPUTS:
%
% Function to set a calibration for the real kinect camera (determine relative
% pose to the Dobot), using the tag zero calibration piece 
% CameraCalibrationRun()
%
% Function to commence a simulated situation for the Dobot retreating from
% a safety symbol. The safety symbol is to be controlled with a DS4
% controller. Currently, the simulation cannot recover after this function
% ends.
% SafetySymbolSimulation()
% 

classdef Controller < handle
    
    properties
        
        % Input objects
        workspace1; 
        ROSCom1;
        kinect1;
        % Default control constants and stepping
        springStroke = 0.008;
        conveyorSteps = 100;
        dobotShortSteps = 15;
        dobotShortestSteps = 5;
        dobotLongSteps = 100;
        dobotMidSteps = 50;
        % Set up flags and toggles
        emergencyStopCheck = 0;
        remoteControlCheck = 0;
        safetySymbolSimulationCheck = 0;
        collisionSimulationCheck = 0;
        cartesianToJointToggle = 0;
        jointSetToggle = 0;
        
    end
    
    methods
        
        %% CONSTRUCTOR FUNCTION
        function self = Controller(workspaceInput, ROSCommunicator, kinectInput)
            % Set up class properties for workspace, ROS and kinect objects
            self.workspace1 = workspaceInput;
            self.ROSCom1 = ROSCommunicator;
            self.kinect1 = kinectInput;
        end
        
        %% CONTAINER STORAGE FUNCTION
        function StoreContainer(self, containerLabel, containerType)
            
            % Ensure that there are no identically named ingredients in the workspace
            for i = 1:size(self.workspace1.containerStorage,2)
                if strcmp(containerLabel,self.workspace1.containerStorage(i).label)
                    disp("Ingredient label already exists! All labels must be unique. ")
                    disp("Container will not be added. ")
                    return
                end
            end
            
            % Get the target storage transform and the index of the container
            [targetStorageTransform, containerIndex] = self.workspace1.AddContainer(containerLabel, containerType);
            
            % Get the initial pose
            if (self.workspace1.simulationToggle == 1)
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            if (self.workspace1.realRobotToggle == 1)
                realJointAngles = self.ROSCom1.GetJoint();
                realLinRailPos = self.ROSCom1.GetRail();
                [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(realJointAngles, realLinRailPos);
                currentJointAngles = [modelLinRailPos, modelJointAngles];
            end
            
            % Move to 'up' position
            currentJointAngles = self.JointCommand(currentJointAngles, self.workspace1.Dobot1.jointStateUp, self.dobotShortSteps);
            
            % Move to initial 'ready' linear rail position
            linRailPos = -0.775;
            currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortSteps);
            
            % Move container along the conveyor
            if (self.workspace1.simulationToggle == 1)
                containerConveyStart = transl(self.workspace1.conveyorOffset(1) + 0.3 ...
                        , self.workspace1.conveyorOffset(2) ...
                        , self.workspace1.conveyorOffset(3) + self.workspace1.conveyorHeight ...
                        + self.workspace1.containerHeights(containerType));
                containerConveyEnd = transl(self.workspace1.conveyorOffset(1) - 0.3 ...
                        , self.workspace1.conveyorOffset(2) ...
                        , self.workspace1.conveyorOffset(3) + self.workspace1.conveyorHeight ...
                        + self.workspace1.containerHeights(containerType));
                    trplot(containerConveyEnd)
                self.workspace1.AnimateContainer(containerIndex, containerConveyStart, containerConveyEnd, self.conveyorSteps);
            end
            if (self.workspace1.realRobotToggle == 1)
                self.ROSCom1.MoveBelt(1,17500);
                pause(5)
                self.ROSCom1.MoveBelt(0,0);
            end
            
            % Move end-effector just above the container
            if (self.workspace1.kinectToggle == 1)
                storedTags = [self.workspace1.containerStorage(:).tag];
                try [kinectTransformRead, newContainerTag] = self.kinect1.StoreFood(storedTags);
                    containerConveyEnd = transl(0,0,0);
                    containerConveyEnd(1:3,4) = kinectTransformRead(1:3,4);
                    self.workspace1.containerStorage(containerIndex).tag = newContainerTag;
                    trplot(containerConveyEnd)
                catch
                    disp("There was an error when finding the Kinect transform. ")
                end
            end
            [linRailPos, ~] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, containerConveyEnd);
            currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortestSteps);
            containerConveyEnd(3,4) = (containerConveyEnd(3,4) + 0.015);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, containerConveyEnd);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortSteps);
           
            % Attach to container, use the end-effector spring force and suction cup vacuum
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0,0,(-self.springStroke - 0.00));
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            if (self.workspace1.realRobotToggle == 1)
                self.ROSCom1.MoveTool(1);
            end
            
            % Retract the end-effector spring back to level position
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0,0,self.springStroke);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);

            % Lift container off the conveyor belt a bit
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0.00, -0.045, 0.015);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
         
            % Lift the container into a position above the respective shelf to clear any containers stored
            if (containerType == 1) || (containerType == 4)
%                 targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0,0.15,0);
%                 targetTransform(3,4) = targetStorageTransform(3,4) + 0.05;
                currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, self.workspace1.Dobot1.jointStateUp, self.dobotShortSteps);
            else
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                targetTransform(3,4) = targetStorageTransform(3,4) + 0.1;
                [linRailPos, ~] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, targetTransform);
                currentJointAngles = self.LinearRailCommandSimultaneous(containerIndex, currentJointAngles, linRailPos, self.dobotShortSteps);
                [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
            end
            

            % Move the container into a position ready to be moved into storage without fouling other containers
            if (containerType == 1) || (containerType == 4)
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                targetTransform(1,4) = targetStorageTransform(1,4);
                linRailPos = (targetTransform(1,4) - self.workspace1.Dobot1.model.base(1,4)) * -1;
                currentJointAngles = self.LinearRailCommandSimultaneous(containerIndex, currentJointAngles, linRailPos, self.dobotShortSteps);
            else
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                targetTransform(1,4) = targetStorageTransform(1,4);
                targetTransform(2,4) = targetStorageTransform(2,4);
                [linRailPos, ~] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, targetTransform);
                currentJointAngles = self.LinearRailCommandSimultaneous(containerIndex, currentJointAngles, linRailPos, self.dobotShortSteps);
                [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
            end
            
           
            % Move the container into the final position
            targetTransform = targetStorageTransform;
            [linRailPos, ~] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, targetTransform);
            currentJointAngles = self.LinearRailCommandSimultaneous(containerIndex, currentJointAngles, linRailPos, self.dobotShortSteps);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
            
            % Adjust servo angle to return angle of container to zero
            containerRotTransform = self.workspace1.containerStorage(containerIndex).model.base;
            containerRotTransform(1,4) = 0; containerRotTransform(2,4) = 0; containerRotTransform(3,4) = 0;
            targetJointAngles(5) = currentJointAngles(6) - atan(containerRotTransform(2,1) / containerRotTransform(1,1));
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            
            % Turn off suction cup vacuum and move the end-effector away from the container
            if (self.workspace1.realRobotToggle == 1)
                self.ROSCom1.MoveTool(0);
            end
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0, -0.045, 0.03);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortSteps);
            
        end
        
        %% Container Retrieval Function
        function RetrieveContainer(self, containerLabel)
            
            % Initialise an index for selected container
            containerIndex = 0;
            
            % Search for the ingredient in the workspace
            for i = 1:size(self.workspace1.containerStorage,2)
                if strcmp(containerLabel,self.workspace1.containerStorage(i).label)
                    disp("Ingredient " + "'" + containerLabel + "'" + " found. Retrieving... ")
                    containerIndex = i;
                    break
                end
                if i == size(self.workspace1.containerStorage,2) && (containerIndex == 0)
                    disp("Ingredient " + "'" + containerLabel + "'" + " does not exist in the pantry. Please reconsider your selection. ")
                    return
                end
            end
            
            % Get the initial pose, container type and get the target location of the desired container
            containerType = self.workspace1.containerStorage(containerIndex).type;
            if (self.workspace1.simulationToggle == 1)
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
                storageLocation = self.workspace1.containerStorage(containerIndex).model.base;
            end
            if (self.workspace1.realRobotToggle == 1)
                realJointAngles = self.ROSCom1.GetJoint();
                realLinRailPos = self.ROSCom1.GetRail();
                [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(realJointAngles, realLinRailPos);
                currentJointAngles = [modelLinRailPos, modelJointAngles];
                if (self.workspace1.kinectToggle == 1)
                    if (~isempty(self.workspace1.containerStorage(containerIndex).tag))
                        try kinectReading = self.kinect1.GetTargetRaw(self.workspace1.containerStorage(containerIndex).tag);
                            storageLocation = transl(0,0,0);
                            storageLocation(1:3,4) = kinectReading(1:3,4);
                        catch
                            disp("There was an error when retrieving the stored container transform from the kinect. ")
                        end
                    end
                end
            end
            
            % If shelf 1: move to default position, if shelf 2: move to 'up' position
            if (containerType == 1) || (containerType == 4)
                currentJointAngles = self.JointCommand(currentJointAngles, self.workspace1.Dobot1.jointStateUp, self.dobotShortSteps);
            else
                currentJointAngles = self.JointCommand(currentJointAngles, self.workspace1.Dobot1.jointStateUp, self.dobotShortSteps);
                linRailPos = -0.75;
                currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortSteps);
            end
            % If shelf 1: move to lin rail approach position, if shelf 2: move to lin rail x = target storage location X
            if (containerType == 1) || (containerType == 4)
                linRailPos = (storageLocation(1,4) - self.workspace1.Dobot1.model.base(1,4)) * -1;
                currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortSteps);
            else
                currentJointAngles = self.JointCommand(currentJointAngles, self.workspace1.Dobot1.jointStateDefault2, self.dobotShortSteps);
            end
            
            % Move end effector / lin rail to final position + a bit above container
            [linRailPos, ~] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, storageLocation);
            currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortestSteps);
            targetTransform = storageLocation;
            targetTransform(3,4) = (storageLocation(3,4) + 0.00);
            targetTransform(2,4) = (storageLocation(2,4) - 0.01);
            try [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            catch
                if (containerType == 2) || (containerType == 3)
                    targetTransform(3,4) = (storageLocation(3,4) + 0.01);
                    [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                else
                    disp("Error with top shelf container range. ")
                end
            end
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortSteps);
            
            % Drop end effector down with spring and turn on suction
            targetTransform = storageLocation;
            targetTransform(3,4) = (storageLocation(3,4) - self.springStroke);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            if (self.workspace1.realRobotToggle == 1)
                self.ROSCom1.MoveTool(1);
            end
            
            targetTransform = storageLocation;
            targetTransform(2,4) = (storageLocation(2,4) - 0.01);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            
            % Lift end effector + container back up plus a little
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0, 0, 0.005);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            
            % If shelf 1: move Z upwards to clear containers, if shelf 2: move back Y axis to clear containers
            if (containerType == 1) || (containerType == 4)
%                 targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0, -0.075, 0.045);
%                 [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, self.workspace1.Dobot1.jointStateUp, self.dobotShortSteps);
            else
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0, 0, 0.1);
                [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
            end
            
            % Move linear rail back to conveyor drop location
            linRailPos = -0.65;
            currentJointAngles = self.LinearRailCommandSimultaneous(containerIndex, currentJointAngles, linRailPos, self.dobotShortSteps);
            
            % Move end effector above conveyor and release suction.
            dropTransform = transl(self.workspace1.conveyorOffset(1) - 0.25 ...
                        , self.workspace1.conveyorOffset(2) ...
                        , self.workspace1.conveyorOffset(3) + self.workspace1.conveyorHeight ...
                        + self.workspace1.containerHeights(containerType));
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, dropTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
            if (self.workspace1.realRobotToggle == 1)
                self.ROSCom1.MoveTool(0);
            end
            
            % Return end effector to top position and run item off conveyor belt
            currentJointAngles = self.JointCommand(currentJointAngles, self.workspace1.Dobot1.jointStateUp, self.dobotShortSteps);
            if (self.workspace1.realRobotToggle == 1)
                self.ROSCom1.MoveBelt(1,-17500);
                pause(6.25)
                self.ROSCom1.MoveBelt(0,0);
            end
            if (self.workspace1.simulationToggle == 1)
                containerConveyStart = self.workspace1.containerStorage(containerIndex).model.base;
                containerConveyEnd = transl(self.workspace1.conveyorOffset(1) + 0.35 ...
                        , self.workspace1.conveyorOffset(2) ...
                        , self.workspace1.conveyorOffset(3) + self.workspace1.conveyorHeight ...
                        + self.workspace1.containerHeights(containerType));
                self.workspace1.AnimateContainer(containerIndex, containerConveyStart, containerConveyEnd, self.conveyorSteps);
                % Completely remove the container from simulated workspace
                self.workspace1.containerStorage(containerIndex).model.base = transl(5,0,0);
                self.workspace1.containerStorage(containerIndex).model.animate(0);
            end
            % Reset the shelf-filled index corresponding to the container removed
            if (containerType == 1) || (containerType == 4)
                self.workspace1.shelfFilledIndex(2, self.workspace1.containerStorage(containerIndex).shelfIndex) = 0;
                self.workspace1.shelf2NumStored = self.workspace1.shelf2NumStored - 1;
            else
                self.workspace1.shelfFilledIndex(1, self.workspace1.containerStorage(containerIndex).shelfIndex) = 0;
                self.workspace1.shelf1NumStored = self.workspace1.shelf1NumStored - 1;
            end
            self.workspace1.containerStorage(containerIndex) = [];
        end
        
        %% Emergency stop function (for GUI callback)
        function EmergencyStop(self)
            % Set flag to 1
            self.emergencyStopCheck = 1;
            % Iterate with endless loop until the flag is cleared
            while(self.emergencyStopCheck == 1)
                pause(0.001)
            end
        end
        
        %% Clear emergency stop function (for GUI callback)
        function ClearEmergencyStop(self)
            % Clear the emergency stop flag to end indefinite loop
            self.emergencyStopCheck = 0;
        end
        
        %% Check the shelf capacity function (for GUI callback)
        function [shelf1Capacity, shelf2Capacity] =  GetCapacityStatus(self)
            % Return current capacity of each shelf as a ratio of max
            % capacity
            shelf1Capacity = self.workspace1.shelf1NumStored / (self.workspace1.maxNumContainers/2);
            shelf2Capacity = self.workspace1.shelf2NumStored / (self.workspace1.maxNumContainers/2);
        end
        
        %% Function to jog the Dobot end effector via x,y,z or r,p,y (GUI callback)
        function currentJointAngles = JogPosition(self, jogAmount, jogParameter)
            % Initialise parameters to zero
            moveLinRail = 0;
            moveJoints = 0;
            changeTransform = transl(0,0,0);
            % Iterate through jogParameter (multiple axes can be jogged)
            for i = 1:(size(jogParameter, 2))
                % Check which axis will be jogged and set the new transl by
                % the jog amount
                switch jogParameter(i)
                    case 0 % LINEAR RAIL
                        moveLinRail = 1;
                    case 1 % X AXIS
                        moveJoints = 1;
                        changeTransform = changeTransform * transl(0,jogAmount(i),0);
                    case 2 % Y AXIS
                        moveJoints = 1;
                        changeTransform = changeTransform * transl(-jogAmount(i),0,0);
                    case 3 % Z AXIS
                        moveJoints = 1;
                        changeTransform = changeTransform * transl(0,0,jogAmount(i));
                    case 4 % ROLL
                        disp("Dobot cannot rotate the end effector about X. ")
                        return
                    case 5 % PITCH
                        disp("Dobot cannot rotate the end effector about Y. ")
                        return
                    case 6 % YAW
                        moveJoints = 1;
                        changeTransform = changeTransform * rpy2tr(0,0,deg2rad(jogAmount(i)));
                    otherwise
                        disp("Invalid jog parameter input. ")
                        return
                end
            end
            % Get current joint angles of the robot, depending on
            % simulation or real robot toggles
            if (self.workspace1.simulationToggle == 1)
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            if (self.workspace1.realRobotToggle == 1)
                realJointAngles = self.ROSCom1.GetJoint();
                realLinRailPos = self.ROSCom1.GetRail();
                [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(realJointAngles, realLinRailPos);
                currentJointAngles = [modelLinRailPos, modelJointAngles];
            end
            % Obtain the current transform
            currentTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
            % Determine the new transform and try to create a new pose for
            % the robot to reach it, and animate / move real robot to get there
            if (moveJoints == 1)
                targetTransform = currentTransform * changeTransform;
                try [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                    currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, 3);
                catch
                end
            end
            % Determine the new transform for the linear rail if selected, 
            % and try to create a new pose for the robot to reach it, and 
            % animate / move real robot to get there
            if (moveLinRail == 1)
                linRailPos = currentJointAngles(1) + jogAmount(jogParameter == 0);
                currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, 3);
            end
        end
        
        %% Function to jog the Dobot end effector via joint angles (GUI callback)
        function JogJoint(self, jogAmount, jogParameter)
            moveLinRail = 0;
            % Check which joint is being jogged
            switch jogParameter
                case 0 % LINEAR RAIL
                    moveLinRail = 1;
                case 1 % Base joint
                    joint = 1;
                case 2 % Forearm
                    joint = 2;
                case 3 % Rear arm
                    joint = 3;
                case 4 % Servo
                    joint = 5;
                otherwise
                    disp("Invalid jog parameter input. ")
                    return
            end
            % Check the current joint angles of simulation or real robot
            if (self.workspace1.simulationToggle == 1)
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            if (self.workspace1.realRobotToggle == 1)
                realJointAngles = self.ROSCom1.GetJoint();
                realLinRailPos = self.ROSCom1.GetRail();
                [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(realJointAngles, realLinRailPos);
                currentJointAngles = [modelLinRailPos, modelJointAngles];
            end
            % Set the new joint angles by the specified jog amount and
            % animate it / move real robot
            if (moveLinRail == 0)
                jogAmount = jogAmount;
                targetJointAngles = currentJointAngles(2:end);
                targetJointAngles(joint) = jogAmount;
                targetJointAngles(4) = pi/2 - targetJointAngles(3) - targetJointAngles(2);
                currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            else
                linRailPos = jogAmount;
                currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortestSteps);
            end
        end
        
        %% Function for remote control input with DS4 controller
        % Some of this code was derived from Robotics 41013 Lab 11.
        function StartRemoteControl(self, jointToggle, collisionsToggle)
            % Make arguments as optional, set toggles to zero as default
            if (nargin < 1)
                self.cartesianToJointToggle = 0;
                collisionsToggle = 0;
            else
                self.cartesianToJointToggle = jointToggle;
            end
            % Toggle the remote control check so that an indefinite loop
            % can be used for remote control
            self.remoteControlCheck = 1;
            % Controller ID: May need to change if there are any errors
            controllerID = 1;
            % Try to create a joystick object, may fail if user does not
            % have the right toolbox. End function if so.
            try DS4Controller = vrjoystick(controllerID);
            catch
                disp("Simulink 3D Animation toolbox may not be installed, or controller ID may need to change. " )
                self.remoteControlCheck = 0;
                return
            end
            % Obtain initial joint angles; for simulation or for real robot
            if (self.workspace1.simulationToggle == 1)
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            if (self.workspace1.realRobotToggle == 1)
                actualRealJointAngles = self.ROSCom1.GetJoint();
                actualLinRailPos = self.ROSCom1.GetRail();
                [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(actualRealJointAngles, actualLinRailPos);
                currentJointAngles = [modelLinRailPos, modelJointAngles];
            end
            % Initialise OS dependent controller readings
            leftJoystickHorizontal = 1;
            leftJoystickVertical = 2;
            if (ispc)
                rightJoystickHorizontal = 3;
                rightJoystickVertical = 6;
            elseif (isunix)
                rightJoystickHorizontal = 4;
                rightJoystickVertical = 5;
            else
                disp("If you are using MacOS, please reconsider. ")
                return
            end
            % Initialise RMRC parameters. Can be tuned for smoother operation
            dT = 0.15;      % Time step
            counter = 0;    % Loop counter
            axisGain = 0.03; % Amount for distance per tick gain on robot joints
            railGain = 0.05; % Amount of distance per tick gain on linear rail
            jointGain = 0.15; % Amount to move joints per tick gain
            lambda = 0.1;   % Damping coefficient
            pickedUp = 0;   % Identifier for an object if picked up
            tic;            % Start time
            % Commence control loop, idle until there is user input.
            % Indefinite loop until remote control check is cleared by GUI.
            while (self.remoteControlCheck == 1)
                while (self.cartesianToJointToggle == 1)
                    % Iterate counter
                    counter = counter + 1;
                    % Obtain joystick data
                    [axes, buttons, ~] = read(DS4Controller);
                    % Initialise variables
                    baseVelocity = 0;
                    linRailVelocity = 0;
                    forearmVelocity = 0;
                    reararmVelocity = 0;
                    % Calculate joint velocities based on joint set toggle
                    if (self.jointSetToggle == 0) % joint set: linear rail + base joint
                        baseVelocity = jointGain * axes(leftJoystickHorizontal);
                        linRailVelocity = railGain * -axes(rightJoystickHorizontal);
                    else % ie. joint set toggle == 1: forearm joint + reararm joint
                        forearmVelocity = jointGain * axes(leftJoystickVertical);
                        reararmVelocity = jointGain * axes(rightJoystickVertical);
                    end
                    % Store previous joint angles before updating
                    prevJointAngles = currentJointAngles;
                    % Update to new joint angles based on velocities
                    currentJointAngles(1) = currentJointAngles(1) + (linRailVelocity * dT);
                    currentJointAngles(2) = currentJointAngles(2) + (baseVelocity * dT);
                    currentJointAngles(3) = currentJointAngles(3) + (forearmVelocity * dT);
                    currentJointAngles(4) = currentJointAngles(4) + (reararmVelocity * dT);
                    currentJointAngles(5) = pi/2 - currentJointAngles(4) - currentJointAngles(3);
                    % Check joint limits, set to previous values if
                    % breaching joint limits
                    inLimits = self.workspace1.Dobot1.CheckJointLimits(currentJointAngles);
                    if ~all(inLimits)
                        jointLimitIndex = find(~inLimits);
                        currentJointAngles(jointLimitIndex) = prevJointAngles(jointLimitIndex);
                        currentJointAngles(5) = pi/2 - currentJointAngles(4) - currentJointAngles(3);
                    end
                    % Pick up container with button press if nearby
                    endEffTr = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                    if (buttons(1)) && (pickedUp == 0)
                        if ~(isempty(self.workspace1.containerStorage))
                            for i = 1:size(self.workspace1.containerStorage, 2)
                                containerTr = self.workspace1.containerStorage(i).model.base;
                                dist = sqrt((endEffTr(1,4) - containerTr(1,4))^2 + (endEffTr(2,4) - containerTr(2,4))^2 + (endEffTr(3,4) - containerTr(3,4))^2);
                                if (dist < 0.015)
                                    disp("Picking up container... ")
                                    pickedUp = i;
                                    break
                                end
                            end
                        end
                    end
                    % Drop container if currently holding a container and button 2 is pressed
                    if (pickedUp ~= 0)
                        self.workspace1.containerStorage(pickedUp).model.base(1:3, 4) = endEffTr(1:3, 4);
                        self.workspace1.containerStorage(pickedUp).model.animate(0);
                        if buttons(2)
                            disp("Dropping container... ")
                            pickedUp = 0;
                        end
                    end
                    % Animate new Dobot joint angles
                    self.workspace1.Dobot1.model.animate(currentJointAngles)
                    % If button 6 is pressed, change the set of joints to control
                    if buttons(6)
                        if (self.jointSetToggle == 0)
                            self.jointSetToggle = 1;
                        else
                            self.jointSetToggle = 0;
                        end
                    end
                    % If button 3 is pressed, change to cartesian control
                    if buttons(3)
                        self.cartesianToJointToggle = 0;
                        break
                    end
                    % If button 4 is pressed, end remote controlling
                    if buttons(4)
                        self.remoteControlCheck = 0;
                        break
                    end
                    if (self.remoteControlCheck == 0)
                        break
                    end
                    % Pause until dT has passed for this iteration
                    while (toc < dT * counter)
                    end
                end
                while (self.cartesianToJointToggle == 0)
                    % Iterate counter
                    counter = counter + 1;
                    % Obtain joystick data
                    [axes, buttons, ~] = read(DS4Controller);
                    % Convert joystick input to an axial / rail velocity
                    xVelocity = axisGain * axes(leftJoystickHorizontal);
                    yVelocty = axisGain * -axes(rightJoystickVertical);
                    zVelocity = axisGain * -axes(leftJoystickVertical);
                    linRailVelocity = railGain * -axes(rightJoystickHorizontal);
                    % Use jacobian with DLS to determine joint velocities; use
                    % 6 link model which includes linear rail for "supposedly
                    % actuated" jacobian 
                    jacobian = self.workspace1.Dobot1.model2.jacob0(currentJointAngles(2:end));
                    jacobian3X3 = jacobian(1:3, 1:3);
                    DLSinvJacob = (transpose(jacobian3X3) * (jacobian3X3 + lambda*eye(3))) \ transpose(jacobian3X3);
                    qDotX = (DLSinvJacob(:,1) * xVelocity)';
                    qDotY = (DLSinvJacob(:,2) * yVelocty)';
                    qDotZ = (DLSinvJacob(:,3) * zVelocity)';
                    % Store previous joint angles before updating
                    prevJointAngles = currentJointAngles;
                    % Apply joint velocities to increment joint angles
                    currentJointAngles(2:4) = currentJointAngles(2:4) + ((qDotX + qDotY + qDotZ) * dT);
                    currentJointAngles(5) = pi/2 - currentJointAngles(4) - currentJointAngles(3);
                    currentJointAngles(6) = -currentJointAngles(2);
                    currentJointAngles(1) = currentJointAngles(1) + (linRailVelocity * dT);
                    % Check joint limits
                    inLimits = self.workspace1.Dobot1.CheckJointLimits(currentJointAngles);
                    if ~all(inLimits)
                        jointLimitIndex = find(~inLimits);
                        currentJointAngles(jointLimitIndex) = prevJointAngles(jointLimitIndex);
                        currentJointAngles(5) = pi/2 - currentJointAngles(4) - currentJointAngles(3);
                    end
                    % Collision detection feature
                    if (collisionsToggle == 1)
                        for i = 1:size(self.workspace1.containerStorage, 2)
                            collisionCheck = self.workspace1.Dobot1.CheckCollision(currentJointAngles, self.workspace1.containerStorage(i), 0);
                            if (collisionCheck == 1)
                                self.workspace1.Dobot1.CheckCollision(currentJointAngles, self.workspace1.containerStorage(i), 1);
                                return
                            end
                        end
                    end
                    % Pick up container with button press if nearby
                    endEffTr = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                    if (buttons(1)) && (pickedUp == 0)
                        if ~(isempty(self.workspace1.containerStorage))
                            for i = 1:size(self.workspace1.containerStorage, 2)
                                try containerTr = self.workspace1.containerStorage(i).model.base;
                                catch
                                    disp("There was an error with indexing for container pickup. ")
                                    containerTr = transl(0,0,0);
                                end
                                dist = sqrt((endEffTr(1,4) - containerTr(1,4))^2 + (endEffTr(2,4) - containerTr(2,4))^2 + (endEffTr(3,4) - containerTr(3,4))^2);
                                if (dist < 0.015)
                                    disp("Picking up container... ")
                                    pickedUp = i;
                                    break
                                end
                            end
                        end
                    end
                    % Drop picked-up container with a button press
                    if (pickedUp ~= 0)
                        self.workspace1.containerStorage(pickedUp).model.base(1:3, 4) = endEffTr(1:3, 4);
                        self.workspace1.containerStorage(pickedUp).model.animate(0);
                        if buttons(2)
                            disp("Dropping container... ")
                            pickedUp = 0;
                        end
                    end
                    % Animate the new Dobot joint configuration
                    self.workspace1.Dobot1.model.animate(currentJointAngles)
                    % If button 3 is pressed, switch to joint control
                    if buttons(3)
                        self.cartesianToJointToggle = 1;
                        break
                    end
                    % If button 4 is pressed, end remote control
                    if buttons(4)
                        self.remoteControlCheck = 0;
                        break
                    end
                    if (self.remoteControlCheck == 0)
                        break
                    end
                    % Pause until dT has passed for this iteration
                    while (toc < dT * counter)
                    end
                end
            end
        end
        
        %% Function to terminate remote control 
        function EndRemoteControl(self)
            % Clear remote control flag to end indefinite while loop
            self.remoteControlCheck = 0;
        end
        
        %% Function for generic linear rail movement
        function newJointAngles = LinearRailCommand(self, currentJointAngles, positionInput, steps)
            % Set variables for command
            linRailPos = positionInput;
            updatedJointAngles = currentJointAngles;
            updatedJointAngles(1) = linRailPos;
            % Check if the linear rail position has changed
            if linRailPos ~= currentJointAngles(1)
                % If simulation toggle is on, animate the new position
                if (self.workspace1.simulationToggle == 1)
                    if (steps == 1)
                        self.workspace1.Dobot1.model.animate(updatedJointAngles);
                    else
                        self.workspace1.AnimateDobot(currentJointAngles, updatedJointAngles, steps);
                    end
                    newJointAngles = self.workspace1.Dobot1.model.getpos;
                end
                % If real robot is used, send a command to move it to new location
                if (self.workspace1.realRobotToggle == 1)
                    [~, realLinRailPos] = self.workspace1.Dobot1.GetRealJointAngles(updatedJointAngles(2:end), linRailPos);
                    self.ROSCom1.MoveRail(realLinRailPos);
                    newRailPos = self.ROSCom1.GetRail();
                    newJointAngles = currentJointAngles;
                    newJointAngles(1) = -newRailPos;
                end
            else
                % Return updated joint angles
                newJointAngles = currentJointAngles;
            end
        end
        
        %% Function for generic linear rail movement with container
        function newJointAngles = LinearRailCommandSimultaneous(self, containerIndex, currentJointAngles, positionInput, steps)
            % Set variables for command
            linRailPos = positionInput;
            updatedJointAngles = currentJointAngles;
            updatedJointAngles(1) = linRailPos;
            % Check if the linear rail position has changed
            if linRailPos ~= currentJointAngles(1)
                % If simulation toggle is on, animate the new position with container
                if (self.workspace1.simulationToggle == 1)
                    self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, updatedJointAngles, steps);
                    newJointAngles = self.workspace1.Dobot1.model.getpos;
                end
                % If real robot is used, send a command to move it to new location
                if (self.workspace1.realRobotToggle == 1)
                    [~, realLinRailPos] = self.workspace1.Dobot1.GetRealJointAngles(updatedJointAngles(2:end), linRailPos);
                    self.ROSCom1.MoveRail(realLinRailPos);
                    newRailPos = self.ROSCom1.GetRail();
                    newJointAngles = currentJointAngles;
                    newJointAngles(1) = -newRailPos;
                end
            else
                newJointAngles = currentJointAngles;
            end
        end
        
        %% Function for generic joint angle motion
        function newJointAngles = JointCommand(self, currentJointAngles, finalJointAngles, steps)
            % If simulation toggle is on, animate the new position
            if (self.workspace1.simulationToggle == 1)
                if (steps == 1)
                    self.workspace1.Dobot1.model.animate([currentJointAngles(1), finalJointAngles]);
                else
                    self.workspace1.AnimateDobot(currentJointAngles, [currentJointAngles(1), finalJointAngles], steps);
                end
                newJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            % If real robot is used, send a command to move it to new location
            if (self.workspace1.realRobotToggle == 1)
                [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(finalJointAngles, currentJointAngles(1));
                self.ROSCom1.MoveJoint(realJointAngles);
                actualRealJointAngles = self.ROSCom1.GetJoint();
                actualLinRailPos = self.ROSCom1.GetRail();
                [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(actualRealJointAngles, actualLinRailPos);
                newJointAngles = [modelLinRailPos, modelJointAngles];
            end
        end
        
        %% Function for generic joint angle motion with container
        function newJointAngles = JointCommandSimultaneous(self, containerIndex, currentJointAngles, finalJointAngles, steps)
            % If simulation toggle is on, animate the new position with container
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, [currentJointAngles(1), finalJointAngles], steps);
                newJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            % If real robot is used, send a command to move it to new location
            if (self.workspace1.realRobotToggle == 1)
                [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(finalJointAngles, currentJointAngles(1));
                self.ROSCom1.MoveJoint(realJointAngles);
                actualRealJointAngles = self.ROSCom1.GetJoint();
                actualLinRailPos = self.ROSCom1.GetRail();
                [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(actualRealJointAngles, actualLinRailPos);
                newJointAngles = [modelLinRailPos, modelJointAngles];
            end
        end
        
        %% Function for camera calibration
        function CameraCalibrationRun(self)
            % Check that kinect toggle is on
            if (self.workspace1.kinectToggle == 0)
                disp("Calibration cannot be run in simulated system. ")
                return
            end
            % Get the real Dobot joint angles and linear rail position
            realJointAngles = self.ROSCom1.GetJoint();
            realLinRailPos = self.ROSCom1.GetRail();
            % Convert them to model joint angles
            [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(realJointAngles, realLinRailPos);
            currentJointAngles = [modelLinRailPos, modelJointAngles];
            % Get the real end effector transform
            endEffectorTr = self.workspace1.Dobot1.model.fkine(currentJointAngles);
            calibrationTagTr = transl(0,0,0);
            % Offset as per calibration piece
            calibrationTagTr(1,4) = endEffectorTr(1,4) - 0.045;
            calibrationTagTr(2,4) = endEffectorTr(2,4);
            calibrationTagTr(3,4) = endEffectorTr(3,4);
            % Send to kinect class to calibrate it
            self.kinect1.FindCalibrationTransform(calibrationTagTr);
        end
        
        %% Function for visual servoing simulation - safety symbol retreat
        function SafetySymbolSimulation(self)
            % Ensure that simulation is turned on
            if (self.workspace1.simulationToggle == 0)
                disp("Safety symbol simulation can only be run in simulation mode. ")
                return
            end
            % Toggle on the flag for running the safety retreat simulation
            self.safetySymbolSimulationCheck = 1;
            % Set up safety symbol object for control as a SerialLink and
            % plot with centrol point also plotted and stored
            safetySymbolLink = Link('alpha',0,'a',0.001,'d',0,'offset',0);
            safetySymbol = SerialLink(safetySymbolLink, 'name', 'Safety Symbol');
            safetySymbol.base = transl(self.workspace1.conveyorOffset(1) + 0.3 ...
                        , self.workspace1.conveyorOffset(2), self.workspace1.conveyorOffset(3) + self.workspace1.conveyorHeight ...
                        + 0.075);
            points = [safetySymbol.base(1,4); safetySymbol.base(2,4); safetySymbol.base(3,4)];
            [faceData, vertexData, plyData] = plyread('SafetySymbol.ply','tri');
            safetySymbol.faces = {faceData, []};
            safetySymbol.points = {vertexData, []};
            [viewAz, viewEl] = view;
            plot3d(safetySymbol, 0,'noarrow','workspace',self.workspace1.workspaceSize,'delay',0,'view', [viewAz, viewEl]);
            hold on
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end 
            handles = findobj('Tag', safetySymbol.name);
            h = get(handles,'UserData');
            try
                h.link(1).Children.FaceVertexCData = [plyData.vertex.red ...
                                                      , plyData.vertex.green ...
                                                      , plyData.vertex.blue]/255;
                h.link(1).Children.FaceColor = 'interp';
            catch
                disp("No colour in ply file")
            end
            % Controller ID: May need to change if there are any errors
            controllerID = 1;
            % Try to create a joystick object, may fail if user does not
            % have the right toolbox. End function if so.
            try DS4Controller = vrjoystick(controllerID);
            catch
                disp("Simulink 3D Animation toolbox may not be installed, or controller ID may need to change. " )
                self.remoteControlCheck = 0;
                return
            end
            % Initialise OS dependent controller readings
            leftJoystickHorizontal = 1;
            leftJoystickVertical = 2;
            if (ispc)
                rightJoystickHorizontal = 3;
                rightJoystickVertical = 6;
            elseif (isunix)
                rightJoystickHorizontal = 4;
                rightJoystickVertical = 5;
            else
                disp("If you are using MacOS, please reconsider. ")
                return
            end
            % Initialise Cameras; left and right for stereo vision
            cameraBaseline = 0.075;
            focalLength = 0.0075;
            simKinectLeft = CentralCamera('focal', focalLength, 'pixel', 10e-6, ...
                'resolution', [1280 720], 'centre', [640 360], 'name', 'Kinect Left Cam');
            simKinectRight = CentralCamera('focal', focalLength, 'pixel', 10e-6, ...
                'resolution', [1280 720], 'centre', [640 360], 'name', 'Kinect Right Cam');
            kinectTrLeft = transl(self.workspace1.kinectOffset(1) - (cameraBaseline/2), self.workspace1.kinectOffset(2), self.workspace1.kinectOffset(3)) ...
                                    * trotz(pi) * troty(pi);
            kinectTrRight = transl(self.workspace1.kinectOffset(1) + (cameraBaseline/2), self.workspace1.kinectOffset(2), self.workspace1.kinectOffset(3)) ...
                                    * trotz(pi) * troty(pi);                
            simKinectLeft.T = kinectTrLeft;
            simKinectRight.T = kinectTrRight;
            simKinectLeft.clf()
            simKinectRight.clf()
            simKinectLeft.plot_camera(points, 'scale', 0.035);
            simKinectRight.plot_camera(points, 'scale', 0.035);
            grid on
            % Initialise RMRC parameters. Can be tuned for smoother operation
            dT = 0.15;      % Time step
            counter = 0;    % Loop counter
            movementGain = 0.05; % Amount for distance per tick
            lambda = 0.1;   % Damping coefficient
            retreatGain = 0.005; % Gain for setting retreat velocity
            tic;            % Start time
            % Commence control loop, idle until there is user input.
            % Indefinite loop until remote control check is cleared by GUI
            % or by controller-cancel input
            while (self.safetySymbolSimulationCheck == 1)
                pause(0.002)
                hold on
                % Iterate counter
                counter = counter + 1;
                % Obtain joystick data
                [axes, buttons, ~] = read(DS4Controller);
                % Set velocity for safety symbol motion
                xSymVelocity = movementGain * axes(leftJoystickHorizontal);
                ySymVelocity = movementGain * -axes(rightJoystickVertical);
                zSymVelocity = movementGain * -axes(leftJoystickVertical);
                % Move safety symbol base as per velocity and animate
                if (xSymVelocity ~= 0) || (ySymVelocity ~= 0) || (zSymVelocity ~= 0)
                    safetySymbol.base(1,4) = safetySymbol.base(1,4) + xSymVelocity * dT;
                    safetySymbol.base(2,4) = safetySymbol.base(2,4) + ySymVelocity * dT;
                    safetySymbol.base(3,4) = safetySymbol.base(3,4) + zSymVelocity * dT;
                    safetySymbol.animate(0)
                end
                % Delete the previous plot
                if (counter ~= 1)
                    delete(pointsPlot)
                end
                % re-plot the safety symbol middle point
                points = [safetySymbol.base(1,4); safetySymbol.base(2,4); safetySymbol.base(3,4)];
                pointsPlot = plot3(points(1), points(2), points(3), 'r.');
                % Plot the safety symbol middle point on left and right
                % camera image figures
                imagePointsLeft = simKinectLeft.plot(points, 'Tcam', kinectTrLeft, 'o');
                imagePointsRight = simKinectRight.plot(points, 'Tcam', kinectTrRight, 'o');
                % Check if the point is within both image fields of view
                if (imagePointsLeft(1) < simKinectLeft.nu && imagePointsRight(1) < simKinectRight.nu) ...
                        && (imagePointsLeft(2) < simKinectLeft.nv && imagePointsRight(2) < simKinectRight.nv) ...
                        && (imagePointsLeft(1) > 0) && (imagePointsLeft(2) > 0) ...
                        && (imagePointsRight(1) > 0) && (imagePointsRight(2) > 0) 
                    % Calculate the image transform to the point, converted
                    % to the global frame:
                    uLeftAdjusted = imagePointsLeft(1) - simKinectLeft.u0;
                    vLeftAdjusted = imagePointsLeft(2) - simKinectLeft.v0;
                    uRightAdjusted = imagePointsRight(1) - simKinectRight.u0;
                    Z = (cameraBaseline * focalLength) / (uLeftAdjusted - uRightAdjusted);
                    X = ((uLeftAdjusted * Z) / focalLength);
                    Y = ((vLeftAdjusted * Z) / focalLength);
                    Z = Z * 10^5;
                    imageTr = simKinectLeft.T * transl(X,Y,Z) * troty(pi) * trotz(pi);
                    % Get the current end effector transform
                    currentJointAngles = self.workspace1.Dobot1.model.getpos;
                    endEffTr = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                    % Calculate the distance from the end effector to the
                    % point
                    distToEndEff = sqrt((imageTr(1,4) - endEffTr(1,4))^2 + (imageTr(2,4) - endEffTr(2,4))^2 + (imageTr(3,4) - endEffTr(3,4))^2);
                    % If the point is within a certain range, carry out a
                    % movement of the end-effector to move away from it
                    if (distToEndEff < 0.2)
                        % Determine the ratio of X Y and Z that comprise
                        % the total distance from end-eff to the point
                        weightingDenominator = abs(imageTr(1,4) - endEffTr(1,4)) + abs(imageTr(2,4) - endEffTr(2,4)) + abs(imageTr(3,4) - endEffTr(3,4));
                        xDistWeighting = (endEffTr(1,4) - imageTr(1,4)) / weightingDenominator;
                        yDistWeighting = (endEffTr(2,4) - imageTr(2,4)) / weightingDenominator;
                        zDistWeighting = (endEffTr(3,4) - imageTr(3,4)) / weightingDenominator;
                        % Determine a total retreat velocity which is based
                        % on the gain and inversely proportional to
                        % distance (ie. velocity increase as distance
                        % decreases)
                        retreatVelocity = (retreatGain / distToEndEff);
                        % Split velocity into X Y Z components based on
                        % their weightings (which can be negative)
                        xVelocityComponent = retreatVelocity * xDistWeighting;
                        yVelocityComponent = retreatVelocity * yDistWeighting;
                        zVelocityComponent = retreatVelocity * zDistWeighting;
                        % Get the jacobian of the Dobot in current form (without linear rail)
                        jacobian = self.workspace1.Dobot1.model2.jacob0(currentJointAngles(2:end));
                        % Take the 3x3 (x,y,z axes only, with first three
                        % robot joint to control them)
                        jacobian3X3 = jacobian(1:3, 1:3);
                        % Use DLS to reduce the approach to singularities
                        DLSinvJacob = (transpose(jacobian3X3) * (jacobian3X3 + lambda*eye(3))) \ transpose(jacobian3X3);
                        % Determine the change in joint angles required to
                        % achieve desired X Y Z velocities
                        qDotX = (DLSinvJacob(:,1) * xVelocityComponent)';
                        qDotY = (DLSinvJacob(:,2) * yVelocityComponent)';
                        qDotZ = (DLSinvJacob(:,3) * zVelocityComponent)';
                        % Store previous joint angles before updating
                        prevJointAngles = currentJointAngles;
                        % Apply joint velocities to increment joint angles
                        currentJointAngles(2:4) = currentJointAngles(2:4) + ((qDotX + qDotY + qDotZ) * dT);
                        currentJointAngles(5) = pi/2 - currentJointAngles(4) - currentJointAngles(3);
                        currentJointAngles(6) = -currentJointAngles(2);
                        % Check joint limits
                        inLimits = self.workspace1.Dobot1.CheckJointLimits(currentJointAngles);
                        if ~all(inLimits)
                            jointLimitIndex = find(~inLimits);
                            currentJointAngles(jointLimitIndex) = prevJointAngles(jointLimitIndex);
                            currentJointAngles(5) = pi/2 - currentJointAngles(4) - currentJointAngles(3);
                        end
                        % Update the Dobot to new joint angles
                        self.workspace1.Dobot1.model.animate(currentJointAngles)
                    end
                end
                % If button 4 is pressed, end the simulation
                if buttons(4)
                    self.safetySymbolSimulationCheck = 0;
                end
                if (self.safetySymbolSimulationCheck == 0)
                    hold on
                    safetySymbol.base = transl(10,10,10);
                    safetySymbol.animate(0)
                    simKinectLeft.delete;
                    simKinectRight.delete;
                    break
                end
                % Pause until dT has passed for this iteration
                while (toc < dT * counter)
                    pause(0.001)
                end
            end
        end
        %% Function for simulating collision avoidance 
        function CollisionAvoidanceSimulation(self)
            % Ensure that simulation is turned on
            if (self.workspace1.simulationToggle == 0)
                disp("Safety symbol simulation can only be run in simulation mode. ")
                return
            end
            % Set simulation flag for indefinite looping
            self.collisionSimulationCheck = 1;
            % Return robot to central, up position
            currentJointAngles = self.workspace1.Dobot1.model.getpos;
            currentJointAngles = self.LinearRailCommand(currentJointAngles, -0.5, self.dobotShortSteps);
            currentJointAngles = self.JointCommand(currentJointAngles, self.workspace1.Dobot1.jointStateUp, self.dobotShortSteps);
            % Plan a set of waypoints for the robot to continuously cycle
            % through, plot them
            numPoints = 10;
            waypointX1 = ones(1,numPoints)*-0.15;
            waypointX2 = ones(1,numPoints)*0.15;
            waypointY = 0.15;
            waypointZ = linspace(0.3, 0.15, numPoints);
            waypointTrs = zeros(4,4,20);
            counter = 1;
            for i = 1:2:numPoints*2
                waypointTrs(:,:,i) = transl(waypointX1(counter), waypointY, waypointZ(counter));
                plot3(waypointTrs(1,4,i), waypointTrs(2,4,i), waypointTrs(3,4,i), 'r.')
                counter = counter + 1;
            end
            counter = 1;
            for i = 2:2:numPoints*2
                waypointTrs(:,:,i) = transl(waypointX2(counter), waypointY, waypointZ(counter));
                pointPlot = plot3(waypointTrs(1,4,i), waypointTrs(2,4,i), waypointTrs(3,4,i), 'r.');
                counter = counter + 1;
            end
            % Add a container for collision testing
            [~, containerIndex] = self.workspace1.AddContainer('CollisionTester', 4);
            % Toggle the remote control check so that an indefinite loop
            % can be used for remote control
            self.remoteControlCheck = 1;
            % Controller ID: May need to change if there are any errors
            controllerID = 1;
            % Try to create a joystick object, may fail if user does not
            % have the right toolbox. End function if so.
            try DS4Controller = vrjoystick(controllerID);
            catch
                disp("Simulink 3D Animation toolbox may not be installed, or controller ID may need to change. " )
                self.remoteControlCheck = 0;
                return
            end
            % Initialise OS dependent controller readings
            leftJoystickHorizontal = 1;
            leftJoystickVertical = 2;
            if (ispc)
                rightJoystickHorizontal = 3;
                rightJoystickVertical = 6;
            elseif (isunix)
                rightJoystickHorizontal = 4;
                rightJoystickVertical = 5;
            else
                disp("If you are using MacOS, please reconsider. ")
                return
            end
            % Initialise motion parameters. Can be tuned for smoother operation
            dT = 0.15;      % Time step
            counter = 0;    % Loop counter
            waypointCounter = 1; % Counter to iterate through waypoints
            robotStepCounter = 1; % Counter for the robot trajectory steps
            robotMoving = 0;    % Flag for whether the robot is in motion
            targetTransform = zeros(4,4); % Target transform for the robot
            targetTrajectory = zeros(self.dobotShortSteps,6); % Matrix for robot path
            movementGain = 0.075; % Amount for distance per tick
            tic;            % Start time
            % Commence indefinite loop for collision simulation
            while (self.collisionSimulationCheck == 1)
                pause(0.001)
                hold on
                % Iterate counter
                counter = counter + 1;
                % If the robot is not currently moving, create a new path
                if (robotMoving == 0)
                    robotStepCounter = 1;
                    targetTransform(:,:) = waypointTrs(:,:,waypointCounter);
                    waypointCounter = waypointCounter + 1;
                    if (waypointCounter > numPoints*2)
                        waypointCounter = 1;
                    end
                    [~, targetJointAngles] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                    targetTrajectory = self.workspace1.Dobot1.GetJointPathQQ(currentJointAngles, targetJointAngles, self.dobotShortSteps);
                end
                % Collision check all of the joint configurations in the
                % remaining robot trajectory 
                checkCollision = 0;
                for i = robotStepCounter:self.dobotShortSteps
                    checkCollision = self.workspace1.Dobot1.CheckCollision(targetTrajectory(i,:), self.workspace1.containerStorage(containerIndex), 0);
                    if (checkCollision == 1)
                        robotMoving = 0;
                        warning("Collision for trajectory detected. Changing path... ")
                        break
                    else
                        robotMoving = 1;
                    end
                end
                % Step the robot if trajectory has passed the collision check
                if (robotMoving == 1)
                    if (robotStepCounter < self.dobotShortSteps)
                        currentJointAngles = targetTrajectory(robotStepCounter,:);
                        robotStepCounter = robotStepCounter + 1;
                        self.workspace1.Dobot1.model.animate(currentJointAngles)
                    else
                        robotMoving = 0;
                        robotStepCounter = 1;
                    end
                end
                % Obtain joystick data
                [axes, buttons, ~] = read(DS4Controller);
                % Set velocity for safety symbol motion
                xContainerVelocity = movementGain * axes(leftJoystickHorizontal);
                yContainerVelocity = movementGain * -axes(rightJoystickVertical);
                zContainerVelocity = movementGain * -axes(leftJoystickVertical);
                % Move container base as per velocity and animate
                if (xContainerVelocity ~= 0) || (yContainerVelocity ~= 0) || (zContainerVelocity ~= 0)
                    self.workspace1.containerStorage(containerIndex).model.base(1,4) = self.workspace1.containerStorage(containerIndex).model.base(1,4) + xContainerVelocity * dT;
                    self.workspace1.containerStorage(containerIndex).model.base(2,4) = self.workspace1.containerStorage(containerIndex).model.base(2,4) + yContainerVelocity * dT;
                    self.workspace1.containerStorage(containerIndex).model.base(3,4) = self.workspace1.containerStorage(containerIndex).model.base(3,4) + zContainerVelocity * dT;
                    self.workspace1.containerStorage(containerIndex).model.animate(0)
                end
                % End simulation with button press
                if buttons(1)
                    self.collisionSimulationCheck = 0;
                    self.workspace1.containerStorage(containerIndex).model.base = transl(10,10,10);
                    self.workspace1.containerStorage(containerIndex).model.animate(0)
                    self.workspace1.containerStorage(containerIndex) = [];
                    delete(pointPlot);
                end
                % Pause until dT has passed for this iteration
                while (toc < dT * counter)
                    pause(0.001)
                end
            end
        end
    end
end