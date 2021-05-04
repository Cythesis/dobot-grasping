classdef Controller < handle
    
    properties
        
        % Input objects
        workspace1; 
        ROSCom1;
        % Default control constants
        springStroke = 0.0083;
        conveyorSteps = 100;
        dobotShortSteps = 25;
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
            
            % Get the target storage transform and the index of the container
            [targetStorageTransform, containerIndex] = self.workspace1.AddContainer(containerLabel, containerType);
            
            %% Get the initial pose
            if (self.workspace1.simulationToggle == 1)
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            else
                %> ROSCOM CURRENT JOINT ANGLES + LINEAR RAIL POSITION
            end
            
            %% Move to 'up' position
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateDobot(currentJointAngles, [currentJointAngles(1), self.workspace1.Dobot1.jointStateUp], self.dobotShortSteps);
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(self.workspace1.Dobot1.jointStateUp, currentJointAngles(1));
            %> SEND REAL JOINT ANGLES TO ROSCOM
            
            %% Move to initial 'ready' linear rail position
            linRailPos = -0.75;
            updatedJointAngles = currentJointAngles;
            updatedJointAngles(1) = linRailPos;
            if linRailPos ~= currentJointAngles(1)
                if (self.workspace1.simulationToggle == 1)
                    self.workspace1.AnimateDobot(currentJointAngles, updatedJointAngles, self.dobotShortSteps);
                    currentJointAngles = self.workspace1.Dobot1.model.getpos;
                end
                [~, realLinRailPos] = self.workspace1.Dobot1.GetRealJointAngles(updatedJointAngles(2:end), linRailPos);
                %> TODO: WRITE THE NEW LINEAR RAIL POSITION TO ROSCOM
            end
            
            %% Move container along the conveyor
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
            
            %% Move end-effector just above the container
            %> TODO: ASK ROSCOM WHAT THE TRANSFORM OF THE CONTAINER IS
            %> AFTER CONVEYOR HAS FINISHED MOVING
            %> TODO: ASK ROSCOM FOR CURRENT JOINT ANGLES OF DOBOT
            %> Update currentJointAngles and containerConveyEnd as per ROSCOM
            [linRailPos, updatedJointAngles] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, containerConveyEnd);
            if linRailPos ~= currentJointAngles(1)
                if (self.workspace1.simulationToggle == 1)
                    self.workspace1.AnimateDobot(currentJointAngles, updatedJointAngles, self.dobotShortestSteps);
                    currentJointAngles = self.workspace1.Dobot1.model.getpos;
                end
                [~, realLinRailPos] = self.workspace1.Dobot1.GetRealJointAngles(updatedJointAngles(2:end), linRailPos);
                %> TODO: WRITE THE NEW LINEAR RAIL POSITION TO ROSCOM
            end
            containerConveyEnd(3,4) = (containerConveyEnd(3,4) + 0.015);
            [targetJointAngles, targetSimulationJointAngles] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, containerConveyEnd);
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(targetJointAngles, linRailPos);
            %> TODO: WRITE THE NEW DOBOT JOINT ANGLES TO ROSCOM targetJointAngles
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateDobot(currentJointAngles, targetSimulationJointAngles, self.dobotShortSteps);
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            
            %% Attach to container, use the end-effector spring force and suction cup vacuum
            %> TODO: ASK ROSCOM TO TURN ON SUCTION CUP VACUUM
            %> TODO: ASK ROSCOM FOR CURRENT JOINT ANGLES OF DOBOT, UPDATE currentJointAngles
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0,0,(-self.springStroke - 0.015));
            [targetJointAngles, targetSimulationJointAngles] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(targetJointAngles, linRailPos);
            %> TODO: WRITE THE NEW DOBOT JOINT ANGLES TO ROSCOM realJointAngles
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateDobot(currentJointAngles, targetSimulationJointAngles, self.dobotShortestSteps);
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            
            %% Retract the end-effector spring back to level position
            %> TODO: ASK ROSCOM FOR CURRENT JOINT ANGLES OF DOBOT, UPDATE currentJointAngles
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0,0,self.springStroke);
            [targetJointAngles, targetSimulationJointAngles] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(targetJointAngles, linRailPos);
            %> TODO: WRITE THE NEW DOBOT JOINT ANGLES TO ROSCOM realJointAngles
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateDobot(currentJointAngles, targetSimulationJointAngles, self.dobotShortestSteps);
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            
            %% Lift container off the conveyor belt a bit
            %> TODO: ASK ROSCOM FOR CURRENT JOINT ANGLES OF DOBOT, UPDATE currentJointAngles
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0.045, -0.025, 0.025);
            [targetJointAngles, targetSimulationJointAngles] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(targetJointAngles, linRailPos);
            %> TODO: WRITE THE NEW DOBOT JOINT ANGLES TO ROSCOM realJointAngles
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, targetSimulationJointAngles, self.dobotShortSteps);
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            
            %% Lift the container into a position above the respective shelf to clear any containers stored
            %> TODO: ASK ROSCOM FOR CURRENT JOINT ANGLES OF DOBOT, UPDATE currentJointAngles
            if (containerType == 1) || (containerType == 4)
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0.125,0,0);
                targetTransform(3,4) = targetStorageTransform(3,4) + 0.1;
            else
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                targetTransform(3,4) = targetStorageTransform(3,4) + 0.1;
            end
            [linRailPos, updatedJointAngles] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, targetTransform);
            if linRailPos ~= currentJointAngles(1)
                if (self.workspace1.simulationToggle == 1)
                    self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, updatedJointAngles, self.dobotShortSteps);
                    currentJointAngles = self.workspace1.Dobot1.model.getpos;
                end
                [~, realLinRailPos] = self.workspace1.Dobot1.GetRealJointAngles(updatedJointAngles(2:end), linRailPos);
                %> TODO: WRITE THE NEW LINEAR RAIL POSITION TO ROSCOM
            end
            [targetJointAngles, targetSimulationJointAngles] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(targetJointAngles, linRailPos);
            %> TODO: WRITE THE NEW DOBOT JOINT ANGLES TO ROSCOM realJointAngles
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, targetSimulationJointAngles, self.dobotShortSteps);
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            
            %% Move the container into a position ready to be moved into storage without fouling other containers
            %> TODO: ASK ROSCOM FOR CURRENT JOINT ANGLES OF DOBOT, UPDATE currentJointAngles
            if (containerType == 1) || (containerType == 4)
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                targetTransform(1,4) = targetStorageTransform(1,4);
                linRailPos = (targetTransform(1,4) - self.workspace1.Dobot1.model.base(1,4)) * -1;
                updatedJointAngles = [linRailPos, currentJointAngles(2:end)];
            else
                targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles);
                targetTransform(1,4) = targetStorageTransform(1,4);
                targetTransform(2,4) = targetStorageTransform(2,4);
                [linRailPos, updatedJointAngles] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, targetTransform);
            end
            if linRailPos ~= currentJointAngles(1)
                if (self.workspace1.simulationToggle == 1)
                    self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, updatedJointAngles, self.dobotShortSteps);
                    currentJointAngles = self.workspace1.Dobot1.model.getpos;
                end
                [~, realLinRailPos] = self.workspace1.Dobot1.GetRealJointAngles(updatedJointAngles(2:end), linRailPos);
                %> TODO: WRITE THE NEW LINEAR RAIL POSITION TO ROSCOM
            end
            [targetJointAngles, targetSimulationJointAngles] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(targetJointAngles, linRailPos);
            %> TODO: WRITE THE NEW DOBOT JOINT ANGLES TO ROSCOM realJointAngles
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, targetSimulationJointAngles, self.dobotShortSteps);
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            
            %% Move the container into the final position
            %> TODO: ASK ROSCOM FOR CURRENT JOINT ANGLES OF DOBOT, UPDATE currentJointAngles
            targetTransform = targetStorageTransform;
            [linRailPos, updatedJointAngles] = self.workspace1.Dobot1.GetLinRailPos(currentJointAngles, targetTransform);
            if linRailPos ~= currentJointAngles(1)
                if (self.workspace1.simulationToggle == 1)
                    self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, updatedJointAngles, self.dobotShortSteps);
                    currentJointAngles = self.workspace1.Dobot1.model.getpos;
                end
                [~, realLinRailPos] = self.workspace1.Dobot1.GetRealJointAngles(updatedJointAngles(2:end), linRailPos);
                %> TODO: WRITE THE NEW LINEAR RAIL POSITION TO ROSCOM
            end
            [targetJointAngles, targetSimulationJointAngles] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(targetJointAngles, linRailPos);
            %> TODO: WRITE THE NEW DOBOT JOINT ANGLES TO ROSCOM realJointAngles
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateSimulatenously(containerIndex, currentJointAngles, targetSimulationJointAngles, self.dobotShortSteps);
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
            
            %% Turn off suction cup vacuum and move the end-effector away from the container
            %> TODO: ASK ROSCOM FOR CURRENT JOINT ANGLES OF DOBOT, UPDATE currentJointAngles
            %> TODO: TELL ROSCOM TO TURN OFF SUCTION VACUUM
            targetTransform = self.workspace1.Dobot1.model.fkine(currentJointAngles) * transl(0, -0.045, 0.03);
            [targetJointAngles, targetSimulationJointAngles] = self.workspace1.Dobot1.GetLocalPose(currentJointAngles, targetTransform);
            [realJointAngles, ~] = self.workspace1.Dobot1.GetRealJointAngles(targetJointAngles, linRailPos);
            %> TODO: WRITE THE NEW DOBOT JOINT ANGLES TO ROSCOM realJointAngles
            if (self.workspace1.simulationToggle == 1)
                self.workspace1.AnimateDobot(currentJointAngles, targetSimulationJointAngles, self.dobotShortSteps);
                currentJointAngles = self.workspace1.Dobot1.model.getpos;
            end
        end
        
        function RetrieveContainer()
            
        end
        
        function EmergencyStop()
            
        end
        
        function [shelf1Capacity, shelf2Capacity] =  GetCapacityStatus()
            
        end
        
        function JogPosition(jogAmount, jogParameter)
            
        end
        
        function JogJoint(jogAmount, jogParameter)
            
        end
        
    end
    
end