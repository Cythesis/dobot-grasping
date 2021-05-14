classdef Controller < handle
    
    properties
        
        % Input objects
        workspace1; 
        ROSCom1;
        kinect1;
        % Default control constants
        springStroke = 0.008;
        conveyorSteps = 100;
        dobotShortSteps = 15;
        dobotShortestSteps = 5;
        dobotLongSteps = 100;
        dobotMidSteps = 50;
        emergencyStopCheck = 0;
        remoteControlCheck = 0;
        cartesianToJointToggle = 0;
        jointSetToggle = 0;
        
    end
    
    methods
        
        %% CONSTRUCTOR FUNCTION
        function self = Controller(workspaceInput, ROSCommunicator, kinectInput)
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
            linRailPos = -0.75;
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
                try [containerConveyEnd, newContainerTag] = self.kinect1.StoreFood(storedTags);
                    self.workspace1.containerStorage(containerIndex).tag = newContainerTag;
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
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0,0,(-self.springStroke - 0.015));
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
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0.065, -0.045, 0.025);
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
                        try storageLocation = self.kinect1.GetTargetRaw(self.workspace1.containerStorage(containerIndex).tag);
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
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
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
            self.emergencyStopCheck = 1;
            while(self.emergencyStopCheck == 1)
                pause(0.001)
            end
        end
        
        %% Clear emergency stop function (for GUI callback)
        function ClearEmergencyStop(self)
            self.emergencyStopCheck = 0;
        end
        
        %% Check the shelf capacity function (for GUI callback)
        function [shelf1Capacity, shelf2Capacity] =  GetCapacityStatus(self)
            shelf1Capacity = self.workspace1.shelf1NumStored / (self.workspace1.maxNumContainers/2);
            shelf2Capacity = self.workspace1.shelf2NumStored / (self.workspace1.maxNumContainers/2);
        end
        
        %% Function to jog the Dobot end effector via x,y,z or r,p,y (GUI callback)
        function currentJointAngles = JogPosition(self, jogAmount, jogParameter)
            moveLinRail = 0;
            moveJoints = 0;
            changeTransform = transl(0,0,0);
            for i = 1:(size(jogParameter, 2))
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
                        moveJoints = 1;
                        changeTransform = changeTransform * rpy2tr(deg2rad(jogAmount(i)),0,0);
                    case 5 % PITCH
                        moveJoints = 1;
                        changeTransform = changeTransform * rpy2tr(0,deg2rad(jogAmount(i)),0);
                    case 6 % YAW
                        moveJoints = 1;
                        changeTransform = changeTransform * rpy2tr(0,0,deg2rad(jogAmount(i)));
                    otherwise
                        disp("Invalid jog parameter input. ")
                        return
                end
            end
            if (self.workspace1.simulationToggle == 1)
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            if (self.workspace1.realRobotToggle == 1)
                realJointAngles = self.ROSCom1.GetJoint();
                realLinRailPos = self.ROSCom1.GetRail();
                [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(realJointAngles, realLinRailPos);
                currentJointAngles = [modelLinRailPos, modelJointAngles];
            end
            currentTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
            if (moveJoints == 1)
                targetTransform = currentTransform * changeTransform;
                try [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                    currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, 3);
                catch
                end
            end
            if (moveLinRail == 1)
                linRailPos = currentJointAngles(1) + jogAmount(jogParameter == 0);
                currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, 3);
            end
        end
        
        %% Function to jog the Dobot end effector via joint angles (GUI callback)
        function JogJoint(self, jogAmount, jogParameter)
            moveLinRail = 0;
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
            if (self.workspace1.simulationToggle == 1)
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            if (self.workspace1.realRobotToggle == 1)
                realJointAngles = self.ROSCom1.GetJoint();
                realLinRailPos = self.ROSCom1.GetRail();
                [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(realJointAngles, realLinRailPos);
                currentJointAngles = [modelLinRailPos, modelJointAngles];
            end
            if (moveLinRail == 0)
                jogAmount = deg2rad(jogAmount);
                targetJointAngles = currentJointAngles(2:end);
                targetJointAngles(joint) = targetJointAngles(joint) + jogAmount;
                targetJointAngles(4) = pi/2 - targetJointAngles(3) - targetJointAngles(2);
                currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            else
                linRailPos = currentJointAngles(1) + jogAmount;
                currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortestSteps);
            end
        end
        
        %% Function for remote control input with DS4 controller
        % Some of this code was derived from Robotics 41013 Lab 11.
        function StartRemoteControl(self, jointToggle, collisionsToggle)
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
            jointGain = 0.15;
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
                    baseVelocity = 0;
                    linRailVelocity = 0;
                    forearmVelocity = 0;
                    reararmVelocity = 0;
                    if (self.jointSetToggle == 0) % joint set: linear rail + base joint
                        baseVelocity = jointGain * axes(leftJoystickHorizontal);
                        linRailVelocity = railGain * -axes(rightJoystickHorizontal);
                    else % ie. joint set toggle == 1: forearm joint + reararm joint
                        forearmVelocity = jointGain * axes(leftJoystickVertical);
                        reararmVelocity = jointGain * axes(rightJoystickVertical);
                    end
                    % Store previous joint angles before updating
                    prevJointAngles = currentJointAngles;
                    currentJointAngles(1) = currentJointAngles(1) + (linRailVelocity * dT);
                    currentJointAngles(2) = currentJointAngles(2) + (baseVelocity * dT);
                    currentJointAngles(3) = currentJointAngles(3) + (forearmVelocity * dT);
                    currentJointAngles(4) = currentJointAngles(4) + (reararmVelocity * dT);
                    currentJointAngles(5) = pi/2 - currentJointAngles(4) - currentJointAngles(3);
                    % Check joint limits
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
                    if (pickedUp ~= 0)
                        self.workspace1.containerStorage(pickedUp).model.base(1:3, 4) = endEffTr(1:3, 4);
                        self.workspace1.containerStorage(pickedUp).model.animate(0);
                        if buttons(2)
                            disp("Dropping container... ")
                            pickedUp = 0;
                        end
                    end
                    self.workspace1.Dobot1.model.animate(currentJointAngles)
                    if buttons(6)
                        if (self.jointSetToggle == 0)
                            self.jointSetToggle = 1;
                        else
                            self.jointSetToggle = 0;
                        end
                    end
                    if buttons(3)
                        self.cartesianToJointToggle = 0;
                        break
                    end
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
                    if (pickedUp ~= 0)
                        self.workspace1.containerStorage(pickedUp).model.base(1:3, 4) = endEffTr(1:3, 4);
                        self.workspace1.containerStorage(pickedUp).model.animate(0);
                        if buttons(2)
                            disp("Dropping container... ")
                            pickedUp = 0;
                        end
                    end
                    self.workspace1.Dobot1.model.animate(currentJointAngles)
                    if buttons(3)
                        self.cartesianToJointToggle = 1;
                        break
                    end
                    if buttons(4)
                        self.remoteControlCheck = 0;
                        break
                    end
                    % Check loop time in case it is running longer than dT
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
            self.remoteControlCheck = 0;
        end
        
        %% Function for generic linear rail movement
        function newJointAngles = LinearRailCommand(self, currentJointAngles, positionInput, steps)
            linRailPos = positionInput;
            updatedJointAngles = currentJointAngles;
            updatedJointAngles(1) = linRailPos;
            if linRailPos ~= currentJointAngles(1)
                if (self.workspace1.simulationToggle == 1)
                    if (steps == 1)
                        self.workspace1.Dobot1.model.animate(updatedJointAngles);
                    else
                        self.workspace1.AnimateDobot(currentJointAngles, updatedJointAngles, steps);
                    end
                    newJointAngles = self.workspace1.Dobot1.model.getpos;
                end
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
        
        %% Function for generic linear rail movement with container
        function newJointAngles = LinearRailCommandSimultaneous(self, containerIndex, currentJointAngles, positionInput, steps)
            linRailPos = positionInput;
            updatedJointAngles = currentJointAngles;
            updatedJointAngles(1) = linRailPos;
            if linRailPos ~= currentJointAngles(1)
                if (self.workspace1.simulationToggle == 1)
                    self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, updatedJointAngles, steps);
                    newJointAngles = self.workspace1.Dobot1.model.getpos;
                end
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
            if (self.workspace1.simulationToggle == 1)
                if (steps == 1)
                    self.workspace1.Dobot1.model.animate([currentJointAngles(1), finalJointAngles]);
                else
                    self.workspace1.AnimateDobot(currentJointAngles, [currentJointAngles(1), finalJointAngles], steps);
                end
                newJointAngles = self.workspace1.Dobot1.model.getpos;
            end
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
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, [currentJointAngles(1), finalJointAngles], steps);
                newJointAngles = self.workspace1.Dobot1.model.getpos;
            end
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
            if (self.workspace1.kinectToggle == 0)
                disp("Calibration cannot be run in simulated system. ")
                return
            end
            realJointAngles = self.ROSCom1.GetJoint();
            realLinRailPos = self.ROSCom1.GetRail();
            [modelJointAngles, modelLinRailPos] = self.workspace1.Dobot1.GetModelJointAngles(realJointAngles, realLinRailPos);
            currentJointAngles = [modelLinRailPos, modelJointAngles];
            endEffectorTr = self.workspace1.Dobot1.model.fkine(currentJointAngles);
            calibrationTagTr = endEffectorTr;
            calibrationTagTr(1,4) = endEffectorTr(1,4) - 0.045;
            self.kinect1.FindCalibrationTransform(calibrationTagTr);
        end
    end
end