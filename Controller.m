classdef Controller < handle
    
    properties
        
        % Input objects
        workspace1; 
        ROSCom1;
        % Default control constants
        springStroke = 0.0083;
        conveyorSteps = 100;
        dobotShortSteps = 15;
        dobotShortestSteps = 5;
        dobotLongSteps = 100;
        dobotMidSteps = 50;
        
    end
    
    methods
        
        %% CONSTRUCTOR FUNCTION
        function self = Controller(workspaceInput, ROSCommunicator)
            self.workspace1 = workspaceInput;
            self.ROSCom1 = ROSCommunicator;
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
            else
                %> ROSCOM CURRENT JOINT ANGLES + LINEAR RAIL POSITION
            end
            
            % Move to 'up' position
            currentJointAngles = self.JointCommand(currentJointAngles, self.workspace1.Dobot1.jointStateUp, self.dobotShortSteps);
            
            % Move to initial 'ready' linear rail position
            linRailPos = -0.75;
            currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortSteps);
            
            % Move container along the conveyor
            %> TODO: WRITE TO ROSCOM TO MOVE CONVEYOR BY X AMOUNT
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
            
            % Move end-effector just above the container
            %> TODO: ASK ROSCOM WHAT THE TRANSFORM OF THE CONTAINER IS AFTER CONVEYOR HAS FINISHED MOVING
            [linRailPos, ~] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, containerConveyEnd);
            currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortestSteps);
            containerConveyEnd(3,4) = (containerConveyEnd(3,4) + 0.015);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, containerConveyEnd);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortSteps);
           
            % Attach to container, use the end-effector spring force and suction cup vacuum
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0,0,(-self.springStroke - 0.015));
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);
           
            % Retract the end-effector spring back to level position
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0,0,self.springStroke);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);

            % Lift container off the conveyor belt a bit
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0.045, -0.025, 0.025);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
         
            % Lift the container into a position above the respective shelf to clear any containers stored
            if (containerType == 1) || (containerType == 4)
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0.125,0,0);
                targetTransform(3,4) = targetStorageTransform(3,4) + 0.1;
            else
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                targetTransform(3,4) = targetStorageTransform(3,4) + 0.1;
            end
            [linRailPos, ~] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, targetTransform);
            currentJointAngles = self.LinearRailCommandSimultaneous(containerIndex, currentJointAngles, linRailPos, self.dobotShortSteps);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);

            % Move the container into a position ready to be moved into storage without fouling other containers
            if (containerType == 1) || (containerType == 4)
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                targetTransform(1,4) = targetStorageTransform(1,4);
                linRailPos = (targetTransform(1,4) - self.workspace1.Dobot1.model.base(1,4)) * -1;
            else
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                targetTransform(1,4) = targetStorageTransform(1,4);
                targetTransform(2,4) = targetStorageTransform(2,4);
                [linRailPos, ~] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, targetTransform);
            end
            currentJointAngles = self.LinearRailCommandSimultaneous(containerIndex, currentJointAngles, linRailPos, self.dobotShortSteps);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
           
            % Move the container into the final position
            targetTransform = targetStorageTransform;
            [linRailPos, ~] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, targetTransform);
            currentJointAngles = self.LinearRailCommandSimultaneous(containerIndex, currentJointAngles, linRailPos, self.dobotShortSteps);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
            
            % Turn off suction cup vacuum and move the end-effector away from the container
            %> TODO: TELL ROSCOM TO TURN OFF SUCTION VACUUM
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
            else
                %> ROSCOM CURRENT JOINT ANGLES + LINEAR RAIL POSITION, write over currentJointAngles
                %> ROSCOM KINECT GET CONTAINER ARTAG POSE, write over storageLocation
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
            targetTransform(3,4) = (storageLocation(3,4) + 0.015);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortSteps);
            
            % Drop end effector down with spring and turn on suction
            targetTransform = storageLocation;
            targetTransform(3,4) = (storageLocation(3,4) - self.springStroke);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            
            targetTransform = storageLocation;
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            
            % Lift end effector + container back up plus a little
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0, 0, 0.01);
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortestSteps);
            
            % If shelf 1: move Z upwards to clear containers, if shelf 2: move back Y axis to clear containers
            if (containerType == 1) || (containerType == 4)
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0, -0.075, 0.045);
                [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
            else
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0, 0, 0.1);
                [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
            end
            
            % Move linear rail back to conveyor drop location
            linRailPos = -0.75;
            currentJointAngles = self.LinearRailCommandSimultaneous(containerIndex, currentJointAngles, linRailPos, self.dobotShortSteps);
            
            % Move end effector above conveyor and release suction.
            dropTransform = transl(self.workspace1.conveyorOffset(1) - 0.25 ...
                        , self.workspace1.conveyorOffset(2) ...
                        , self.workspace1.conveyorOffset(3) + self.workspace1.conveyorHeight ...
                        + self.workspace1.containerHeights(containerType));
            [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, dropTransform);
            currentJointAngles = self.JointCommandSimultaneous(containerIndex, currentJointAngles, targetJointAngles, self.dobotShortSteps);
            
            % Return end effector to top position and run item off conveyor belt
            currentJointAngles = self.JointCommand(currentJointAngles, self.workspace1.Dobot1.jointStateUp, self.dobotShortSteps);
            %> TODO: WRITE TO ROSCOM TO MOVE CONVEYOR BY X AMOUNT
            if (self.workspace1.simulationToggle == 1)
                containerConveyStart = self.workspace1.containerStorage(containerIndex).model.base;
                containerConveyEnd = transl(self.workspace1.conveyorOffset(1) + 0.35 ...
                        , self.workspace1.conveyorOffset(2) ...
                        , self.workspace1.conveyorOffset(3) + self.workspace1.conveyorHeight ...
                        + self.workspace1.containerHeights(containerType));
                self.workspace1.AnimateContainer(containerIndex, containerConveyStart, containerConveyEnd, self.conveyorSteps);
                % Reset the shelf-filled index corresponding to the container removed
                if (containerType == 1) || (containerType == 4)
                    self.workspace1.shelfFilledIndex(2, self.workspace1.containerStorage(containerIndex).shelfIndex) = 0;
                    self.workspace1.shelf2NumStored = self.workspace1.shelf2NumStored - 1;
                else
                    self.workspace1.shelfFilledIndex(1, self.workspace1.containerStorage(containerIndex).shelfIndex) = 0;
                    self.workspace1.shelf1NumStored = self.workspace1.shelf1NumStored - 1;
                end
                % Completely remove the container from simulated workspace
                self.workspace1.containerStorage(containerIndex).model.base = transl(5,0,0);
                self.workspace1.containerStorage(containerIndex).model.animate(0);
                self.workspace1.containerStorage(containerIndex) = [];
            end
            
        end
        
        function EmergencyStop(self)
            
        end
        
        function [shelf1Capacity, shelf2Capacity] =  GetCapacityStatus(self)
            
            shelf1Capacity = self.workspace1.shelf1NumStored / (self.workspace1.maxNumContainers/2);
            shelf2Capacity = self.workspace1.shelf2NumStored / (self.workspace1.maxNumContainers/2);
            
        end
        
        function JogPosition(self, jogAmount, jogParameter)
            moveLinRail = 0;
            switch jogParameter
                case 0 % LINEAR RAIL
                    changeTransform = eye(4);
                    moveLinRail = 1;
                case 1 % X AXIS
                    changeTransform = transl(0,jogAmount,0);
                case 2 % Y AXIS
                    changeTransform = transl(-jogAmount,0,0);
                case 3 % Z AXIS
                    changeTransform = transl(0,0,jogAmount);
                case 4 % ROLL
                    changeTransform = rpy2tr(deg2rad(jogAmount),0,0);
                case 5 % PITCH
                    changeTransform = rpy2tr(0,deg2rad(jogAmount),0);
                case 6 % YAW
                    changeTransform = rpy2tr(0,0,deg2rad(jogAmount));
                otherwise
                    disp("Invalid jog parameter input. ")
                    return
            end
            
            if (self.workspace1.simulationToggle == 1)
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
                currentTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                if moveLinRail == 0
                    targetTransform = currentTransform * changeTransform;
                    [targetJointAngles, ~] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
                    currentJointAngles = self.JointCommand(currentJointAngles, targetJointAngles, self.dobotShortestSteps);
                else
                    linRailPos = currentJointAngles(1) + jogAmount;
                    currentJointAngles = self.LinearRailCommand(currentJointAngles, linRailPos, self.dobotShortestSteps);
                end
            end
            if (self.workspace1.realRobotToggle == 1)
                %> TODO: Get current joint angles and send end effector command to Dobot for position
            end
        end
        
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
                if moveLinRail == 0
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
            if (self.workspace1.realRobotToggle == 1)
                %> TODO: Get current joint angles and send end effector command to Dobot for position
            end
        end
        
        
        function newJointAngles = LinearRailCommand(self, currentJointAngles, positionInput, steps)
            linRailPos = positionInput;
            updatedJointAngles = currentJointAngles;
            updatedJointAngles(1) = linRailPos;
            if linRailPos ~= currentJointAngles(1)
                if (self.workspace1.simulationToggle == 1)
                    self.workspace1.AnimateDobot(currentJointAngles, updatedJointAngles, steps);
                    newJointAngles = self.workspace1.Dobot1.model.getpos;
                end
                [~, realLinRailPos] = self.workspace1.Dobot1.GetRealJointAngles(updatedJointAngles(2:end), linRailPos);
                %> TODO: WRITE THE NEW LINEAR RAIL POSITION TO ROSCOM, update newJointAngles
            else
                newJointAngles = currentJointAngles;
            end
        end
        
        function newJointAngles = LinearRailCommandSimultaneous(self, containerIndex, currentJointAngles, positionInput, steps)
            linRailPos = positionInput;
            updatedJointAngles = currentJointAngles;
            updatedJointAngles(1) = linRailPos;
            if linRailPos ~= currentJointAngles(1)
                if (self.workspace1.simulationToggle == 1)
                    self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, updatedJointAngles, steps);
                    newJointAngles = self.workspace1.Dobot1.model.getpos;
                end
                [~, realLinRailPos] = self.workspace1.Dobot1.GetRealJointAngles(updatedJointAngles(2:end), linRailPos);
                %> TODO: WRITE THE NEW LINEAR RAIL POSITION TO ROSCOM, update newJointAngles
            else
                newJointAngles = currentJointAngles;
            end
        end
        
        function newJointAngles = JointCommand(self, currentJointAngles, finalJointAngles, steps)
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateDobot(currentJointAngles, [currentJointAngles(1), finalJointAngles], steps);
                newJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(self.workspace1.Dobot1.jointStateUp, currentJointAngles(1));
            %> SEND REAL JOINT ANGLES TO ROSCOM update newJointAngles
        end
        
        function newJointAngles = JointCommandSimultaneous(self, containerIndex, currentJointAngles, finalJointAngles, steps)
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, [currentJointAngles(1), finalJointAngles], steps);
                newJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(self.workspace1.Dobot1.jointStateUp, currentJointAngles(1));
            %> SEND REAL JOINT ANGLES TO ROSCOM update newJointAngles
        end
    end
end