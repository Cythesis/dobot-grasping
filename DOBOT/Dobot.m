%%              Dobot With Linear Rail Class
% This class creates a serial link object to simulate a Dobot Magician, 
% attached to a linear rail. 
% The following describes links 0 - 6:

% LINK0: Linear rail; prismatic link.
% LINK1: 'BaseAngle'; yaw of the entire robot
% LINK2: 'RearArmAngle'; the first link between base and end-effector
% LINK3: 'ForeArmAngle'; the second link between base and end effector
% LINK4: 'N/A'; the parallel linkage which is not controllable
% LINK5: 'ServoAngle'; yaw rotation of the end-effector
% LINK6: End-effector; not a joint

% PHYSICAL PARAMETERS:
% Dobot maximum payload capacity: 500g
% LINK0 Max Velocity: 150mm/s, Max Acceleration: 150mm/s^2
% LINK1-5 Max Velocity: 320deg/s
% LINK6 Max Velocity: 480deg/s

% CALLABLE CLASS FUNCTIONS:
%
% Constructor function:
% self = Dobot(baseTransformInput, workspaceSizeInput)
%           - Input arguments optional
% 
% Function to calculate a new linear rail position for a given point:
% [linRailPos, updatedJointAngles] = GetLinRailPos(currentJointAngles, inputTransform)
%           - Must input 1x6 joint angle matrix and 4x4 global point transform
%
% Function to perform inverse kinematics (not including linear rail) to a
% given point from current position:
% [finalJointAngles, finalSimulationJointAngles] = GetLocalPose(currentJointAngles, inputTransform)
%           - Must input 1x6 joint angle matrix and 4x4 global point transform
%
% Function to generate a path between two known joint positions; for simulation
% jointPath = GetJointPathQQ(currentJointAngles, finalJointAngles, steps)
%           - Must input 1x6 joint angle matrix for initial and final poses
%           - steps input optional
%
% Function to convert the model joint angles into real Dobot joint angles
% to be sent to the robot
% [realJointAngles, realLinRailPos] = GetRealJointAngles(modelJointAngles, modelLinRailPos)
%           - Must input 1x5 joint angle matrix (exclude linear rail) and
%           linear rail position as a scalar value
%
% Function to generate a generic guess pose for  given point, to be used
% for inverse kinematics calculations
% [modelGuessPose, inBoundaryCheck] = GetGuessPose(currentJointAngles, inputTransform)
%           - Must input 1x6 joint angle matrix and 4x4 global point transform
% 
% Function to calculate the azimuth angle to a point relative to the robot
% azimuth = GetAzimuth(relativeTransform)
%           - Must 4x4 input point relative to robot's link 1 (ie. base that
%           slides along the linear rail
% 
% Function to check if a relative point is in the immediate range of the
% robot (ie. not including use of the linear rail)
% inRange = CheckInRange(relativeTransform)
%           - Must 4x4 input point relative to robot's link 1 (ie. base that
%           slides along the linear rail
%
% Function to check if a global point is within the workspace of the robot
% (range including the linear rail)
% inBoundary = CheckInBounds(inputTransform)
%           - Must input 4x4 global point transform
% 

classdef Dobot < handle
    
    properties
        % Use for SerialLink() robot object. This model will contain both
        % the Dobot and its linear rail. However, the real linear rail is
        % not connected to the Dobot as a prismatic kind of first joint.
        % Therefore, this model will only be used for simulation,
        % specifically, for animation purposes.
        model;
        % model2 will be another SerialLink() which will only take into
        % account the Dobot joints, not including the linear rail, so it
        % will not be accounted for in inverse kinematics and other calcs.
        % model2 will not be used for animation.
        model2;
        % Set some default joint states for linear rail model
        jointStateDefault = [0, deg2rad([0, 5, 115, -30, 0])];
        jointStateDefault2 = deg2rad([0, 5, 115, -30, 0]);
        % Give a name for reference
        name = 'Dobot';
        % Set a workspace size
        workspaceSize;
        % Set some range parameters for boundary checks: 
        % Ie. in range if in top left circle 1 and also circle 2, but while not in circle 3 and 4. 
        % Distances to circle origins measured from origin of Dobot's link 1
        rangeCircle1 = struct('radius', 0.27959, 'zDistOrigin', 0.00000, 'normalDistOrigin', 0.05254);
        rangeCircle2 = struct('radius', 0.12310, 'zDistOrigin', 0.00177, 'normalDistOrigin', 0.21102);
        rangeCircle3 = struct('radius', 0.14700, 'zDistOrigin', 0.13449, 'normalDistOrigin', 0.06431);
        rangeCircle4 = struct('radius', 0.14592, 'zDistOrigin', -0.15009, 'normalDistOrigin', 0.06796);
        % Set a default number of steps for unspecified paths - simulation
        % only
        defaultSteps = 50;
    end
        
    methods
        %% Constructor function
        function self = Dobot(baseTransformInput, workspaceSizeInput)
            % Create default case for insufficient input arguments, extend
            % if more arguments are included
            defaultTransform = transl(0,0,0) * rpy2tr(0,0,0);
            defaultWorkspace = [-1, 1, -1, 1, 0, 1];
            switch nargin
                case 2
                    self.workspaceSize = workspaceSizeInput;
                case 1
                    if isempty(baseTransformInput)
                        baseTransformInput = defaultTransform;
                    elseif isempty(workspaceInput)
                        self.workspaceSize = defaultWorkspace;
                    end
                case 0
                    baseTransformInput = defaultTransform;
                    self.workspaceSize = defaultWorkspace;
            end
            % Run constructor functions
            self.GetDobot(baseTransformInput);
            self.RenderDobot();
        end
        %% Function to create a SerialLink() object with DH parameters
        function GetDobot(self, baseTransform)
            
            % Define DH parameters for SerialLink() - 
            % FOR MODEL INCLUDING LINEAR RAIL
            links(1) = Link([0     0       0       -pi/2    1]); % Prismatic link 'sigma', 1,     'a', -0.133,    'alpha', pi/2   
            links(2) = Link('d', 0.08,      'a', 0,         'alpha', pi/2   );
            links(3) = Link('d', 0,         'a', -0.135,    'alpha', 0, 'offset', -pi/2       );
            links(4) = Link('d', 0,         'a', -0.147,    'alpha', 0      );
            links(5) = Link('d', 0,         'a', -0.05254,  'alpha', -pi/2  );
            links(6) = Link('d', -0.08,      'a', 0,         'alpha', 0     ); % End-effector offset
            
            % Define joint limits for SerialLink(), based on suggested -
            % FOR MODEL INCLUDING LINEAR RAIL
            links(1).qlim =         [   -1,     0     ];
            links(2).qlim = deg2rad([   -135,   135   ]);
            links(3).qlim = deg2rad([   5,      80    ]);
            links(4).qlim = deg2rad([   15,     170   ]);
            links(5).qlim = deg2rad([   -90,    90    ]);
            links(6).qlim = deg2rad([   -85,    85    ]);
            
            % Copying values for model2 WITHOUT LINEAR RAIL
            links2 = links;
            % Remove linear rail
            links2(1) = [];
            
            % Create SerialLink() object for linear rail simulated model
            self.model = SerialLink(links, 'name', self.name);
            % Create another SerialLink() object for non-linear rail model
            self.model2 = SerialLink(links2, 'name', self.name);
            % Adjust base locations
            self.model.base = baseTransform * transl(-0.5,0,0.133) * rpy2tr(pi/2, -pi/2, 0);
            self.model2.base = baseTransform * transl(-0.5,0,0.133) * rpy2tr(0, 0, -pi/2);
        end
        %% Function to render Dobot using modelled ply files
        function RenderDobot(self)
            % Iterate through each link and load its respective ply; store
            % face, vertex and ply data.
            for i = 0:self.model.n
                [ faceData, vertexData, plyData{i+1} ] = plyread(['DOBOTLINK',num2str(i),'.ply'],'tri');
                self.model.faces{i+1} = faceData;
                self.model.points{i+1} = vertexData;
            end
            % Display on 3D plot
            self.model.plot3d(self.jointStateDefault, 'noarrow', 'workspace', self.workspaceSize);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            % Try to correctly colour the arm (if colours are in ply file data)
            for i = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(i+1).Children.FaceVertexCData =     [plyData{i+1}.vertex.red ...
                                                              , plyData{i+1}.vertex.green ...
                                                              , plyData{i+1}.vertex.blue]/255;
                    h.link(i+1).Children.FaceColor = 'interp';
                catch 
                end
            end
        end
        %% Function to find an ideal linear rail position given a target transform; specifically for
        % model 2 calculations
        function [linRailPos, updatedJointAngles] = GetLinRailPos(self, currentJointAngles, inputTransform)
            % Check if the desired point is actually reachable, store check
            inBoundaryCheck = self.CheckInBounds(inputTransform);
            if inBoundaryCheck == 1
                % Obtain current joint transforms, get the first link transform
                [~, allJointTransforms] = self.model.fkine(currentJointAngles);
                link1Transform = allJointTransforms(:,:,1);
                % Input transform is in global frame. Make the origin to be the
                % base of link 1 (not linear rail) for range check
                relativeTransform = link1Transform\inputTransform;
                % Check if within reach of robot arm
                inRangeCheck = self.CheckInRange(relativeTransform);
                if inRangeCheck == 1
                    linRailPos = currentJointAngles(1);
                else % not already in range. Linear rail to be moved into range
                    checkIncrements = 0.0025; % in metres, distance between each check
                    tempBaseTransform = link1Transform;
                    if inputTransform(1,4) >= self.model.base(1,4) + -0.5*self.model.qlim(1) % positive half of rail
                        % Iterate through a number of possible global positions along the rail until the first
                        % in-range solution is reached. Start at position = 0 end for a point in positive rail region
                        checkPositions = self.model.base(1,4):checkIncrements:(self.model.base(1,4)-self.model.qlim(1));
                    else % ie. negative half of the rail
                        % same as above, but start from linear rail position = max
                        checkPositions = (self.model.base(1,4)-self.model.qlim(1)):-checkIncrements:self.model.base(1,4);
                    end
                    % Initialise iteration counters
                    possiblePositionIndexes = zeros(1, size(checkPositions, 2));
                    firstPossiblePositionIndex = 0;
                    % begin iteration to check for the first in-range location
                    for i = 1:size(checkPositions, 2)
                        % base transform at the iterated check position
                        tempBaseTransform(1,4) = checkPositions(i);
                        % Take the relative transform from the iterated
                        % base position to desired transform
                        tempRelativeTransform = tempBaseTransform\inputTransform;
                        % Check if in range yet
                        tempRangeCheck = self.CheckInRange(tempRelativeTransform);
                        if (tempRangeCheck == 1)
                            % Fill matrix of all possible position indexes
                            possiblePositionIndexes(i) = 1;
                            % Store the index of the first possible position
                            if (i == 1)
                                firstPossiblePositionIndex = i;
                            else % ie. i is not = 1:
                                if (possiblePositionIndexes(i - 1) == 0)
                                    firstPossiblePositionIndex = i;
                                end
                            end
                        else
                            possiblePositionIndexes(i) = 0;
                            % If we have passed the first set of possible
                            % locations, we want to ignore any subsequent
                            % sets of locations, so break the loop here
                            if (i > 1)
                                if (possiblePositionIndexes(i - 1) == 1)
                                    break
                                end
                            end
                        end
                    end
                    % Check if there are possible positions stored
                    totalPossiblePositions = sum(possiblePositionIndexes);
                    if (totalPossiblePositions == 0)
                        disp("Couldn't find an in-range linear rail position. ")
                        linRailPos = currentJointAngles(1);
                    else
                        % use the position in the middle of the set of
                        % possible positions as the final linear rail pos
                        midPosition = checkPositions(ceil(totalPossiblePositions/2) + firstPossiblePositionIndex);
                        % Convert global position back to the joint space
                        linRailPos = ((midPosition - self.model.base(1,4)) * -1);
                    end
                end
            else % ie. boundary check == 0
                linRailPos = currentJointAngles(1);
                disp("The target transform is unreachable. ")
            end
            updatedJointAngles = currentJointAngles;
            updatedJointAngles(1) = linRailPos;
        end
        %% Function to get a joint pose for a point within reach;
        % This function calculates the final joint positions for the Dobot
        % without using the linear rail (ie. model2) so to reach the point,
        % you must travel to aan appropriate position on the linear rail first.
        % Input joint angles should be 1x6 matrix (include linear rail position),
        % and the output will be 1x5 (excludes a linear rail position).
        function [finalJointAngles, finalSimulationJointAngles] = GetLocalPose(self, currentJointAngles, inputTransform)
            % Ikcon and ikine are having a very hard time with points in
            % the negative link1 angle region. Therefore, we will calculate
            % for the mirrored point relative to the robot, and then switch
            % the base angle to negative to get the actual pose.
            % Ensure that the base location of model2 is correct before
            % performing any calcs:
            self.model2.base(1,4) = ((currentJointAngles(1) - self.model.base(1,4)) * -1);
            % Initialise a check value to keep track of whether the point was mirrored:
            mirrorCheck = 0;
            % Convert input point to the mirrored point about the y
            % axis relative to the robot but only if it is on the negative side:
            if (self.model2.base(1,4) < inputTransform(1,4))
                % Equation to mirror the point relative to the robot base:
                inputTransform(1,4) = (2*self.model2.base(1,4) - inputTransform(1,4));
                mirrorCheck = 1;
            end
            if (size(currentJointAngles, 2) ~= self.model.n)
                disp("Must input a 1x6 joint matrix which includes linear rail.")
                return
            end
            [modelGuessPose, inBoundaryCheck] = self.GetGuessPose(currentJointAngles, inputTransform);
            if (inBoundaryCheck == 0)
                disp("The target transform is not reachable. ")
                return
            end
            if modelGuessPose(1) ~= currentJointAngles(1)
                disp("The target is reachable but requires additional linear rail displacement. ")
                return
            end
            % Convert the 6-link joint matrices to 5 link joint matrices for model2
            model2GuessPose = modelGuessPose(2:end);
            model2CurrentJointAngles = currentJointAngles(2:end);
            % If the transform has passed the above checks, then it is okay
            % to proceed and find a model pose without the linear rail (model2)
            try [ikconJointAngles, ~, ~] = self.model2.ikcon(inputTransform, model2GuessPose);
                % there are cases where guess pose fails and returns complex solutions. Must ensure this is not the case:
                complexCheck = isreal(ikconJointAngles);
                if (complexCheck == 0)
                    % if it is the case, provide the current pose as the guess instead.
                    [ikconJointAngles, ~, ~] = self.model2.ikcon(inputTransform, model2CurrentJointAngles);
                    disp("Model pose guess was complex")
                end
            catch 
                disp("An error occurred when running ikcon. ")
                ikconJointAngles = model2CurrentJointAngles;
            end
            try ikineJointAngles = self.model2.ikine(inputTransform, model2GuessPose);
                % there are cases where guess pose fails and returns complex solutions. Must ensure this is not the case:
                complexCheck = isreal(ikineJointAngles);
                if (complexCheck == 0)
                    % if it is the case, provide the current pose as the guess instead.
                    disp("Model guess pose was complex")
                    mask = [1, 1, 1, 0, 0, 0];
                    ikineJointAngles = self.model2.ikine(inputTransform, model2CurrentJointAngles, mask);
                end
            catch
                % disp("An error occurred when running ikine. ")
                ikineJointAngles = model2CurrentJointAngles;
            end
            % Now we have an ikine and ikcon solution. Pick one that is most suitable:
            % Requirement 1: ikine solution is within joint limits; check all joints
            jointLimitRequirement = zeros(1, (self.model2.n + 1));
            for i = 1:self.model2.n
                lowerLimit = self.model2.qlim(i,1);
                upperLimit = self.model2.qlim(i,2);
                if (ikineJointAngles(i) < lowerLimit) || (ikineJointAngles(i) > upperLimit)
                    jointLimitRequirement(i) = 0;
                else
                    jointLimitRequirement(i) = 1;
                end
            end
            % Also check link4 = pi/2 - link2 - link3 within reasonable threshold
            linkCheck = ikineJointAngles(4) + ikineJointAngles(2) + ikineJointAngles(3) - pi/2;
            if (linkCheck > 0.025) || (linkCheck < -0.025)
                jointLimitRequirement(end) = 0;
            else
                jointLimitRequirement(end) = 1;
            end
            % Requirement 2: ikine solution is closer to desired end-effector
            % position than ikcon solution - or they are both very close
            ikineEndEffTr = self.model2.fkine(ikineJointAngles);
            ikconEndEffTr = self.model2.fkine(ikconJointAngles);
            % Calculate distances
            ikineDist = sqrt((ikineEndEffTr(1,4) - inputTransform(1,4))^2 + (ikineEndEffTr(2,4) - inputTransform(2,4))^2 + (ikineEndEffTr(3,4) - inputTransform(3,4))^2);
            ikconDist = sqrt((ikconEndEffTr(1,4) - inputTransform(1,4))^2 + (ikconEndEffTr(2,4) - inputTransform(2,4))^2 + (ikconEndEffTr(3,4) - inputTransform(3,4))^2);
            % Determine the case based on some threshold; smallest dist = best
            if ((ikineDist - ikconDist) < -0.01)
                distanceRequirement = 1;
            elseif ((ikineDist - ikconDist) > 0.01)
                distanceRequirement = 0;
            else % ie. distances are pretty much the same, within threshold
                distanceRequirement = 2;
            end
            % Requirement 3: ikine solution had a closer link 2 angle to the value of the azimuth
            % found when making a guess for the pose (an ideal azimuth) than ikcon solution
            idealAzimuth = model2GuessPose(1);
            if (abs(ikineJointAngles(1) - idealAzimuth) < abs(ikconJointAngles(1) - idealAzimuth))
                angleRequirement = 1;
            else % ie. ikcon solution closer to ideal azimuth than ikine
                angleRequirement = 0;
            end
            % Check on requirement 1:
            if all(jointLimitRequirement == 1)
                % Check on requirement 2:
                if (distanceRequirement == 1)
                    finalJointAngles = ikineJointAngles;
                    disp("Using Ikine")
                elseif (distanceRequirement == 2)
                    % Check on requirement 3:
                    if (angleRequirement == 1)
                        finalJointAngles = ikineJointAngles;
                        disp("Using Ikine")
                    else % ie. angle requirement == 0
                        finalJointAngles = ikconJointAngles;
                        % disp("Using Ikcon")
                    end
                else % ie. dist requirement == 0
                    finalJointAngles = ikconJointAngles;
                    % disp("Using Ikcon")
                end
            else % ie. joint requirement == 0
                finalJointAngles = ikconJointAngles;
                % disp("Using Ikcon")
            end
            % If at the beginning the point was mirrored, reset the base
            % rotation back to its correct side:
            if mirrorCheck == 1
                finalJointAngles(1) = -1 * finalJointAngles(1);
            end
            % Lastly, for simulation purposes, convert back to the 6-link
            % model for the creation of a joint path for the robot. This will not be
            % used for the real robot.
            finalSimulationJointAngles = [currentJointAngles(1), finalJointAngles];
        end
        %% Function to generate a joint angle path matrix from two known 
        % initial and final joint angle matrices (for simulation model, so use 1x6 matrix)
        function jointPath = GetJointPathQQ(self, currentJointAngles, finalJointAngles, steps)
            if nargin ~= 4
                steps = self.defaultSteps;
            end
            if (size(currentJointAngles, 2) ~= self.model.n)
                disp("Input joint matrix must be for 6-joint link simulation. ")
                return
            end
            jointPath = jtraj(currentJointAngles, finalJointAngles, steps);
            jointPath((jointPath(:, 1) > self.model.qlim(1,2)), 1) = 0;
            jointPath((jointPath(:, 1) < self.model.qlim(1,1)), 1) = self.model.qlim(1,1);
        end
        %% Function to convert model joint space to real robot joint space
        function [realJointAngles, realLinRailPos] = GetRealJointAngles(self, modelJointAngles, modelLinRailPos)
            if (size(modelJointAngles, 2) > 5)
                disp("Must use 5-link joint matrix as input. ")
                return
            end
            realJointAngles = zeros(1, (size(modelJointAngles, 2) - 1)); % Joint 5 will be ommitted
            realLinRailPos = modelLinRailPos * -1; % Reverse direction of linear rail (robot toolbox only likes negative prismatic values)
            realJointAngles(1, 1) = modelJointAngles(1);
            realJointAngles(1, 2) = modelJointAngles(2);    % Angle does not require adjustment
            realJointAngles(1, 3) = modelJointAngles(3) + modelJointAngles(3) - (pi/2); % adjust, accounting for mechanical linkage
            realJointAngles(1, 4) = modelJointAngles(5);    % Servo angle is the same (joint 5 ommitted) 
        end
        %% Function to produce a default guess pose for a given transform
        function [modelGuessPose, inBoundaryCheck] = GetGuessPose(self, currentJointAngles, inputTransform)
            % Check if the desired point is actually reachable, store check
            inBoundaryCheck = self.CheckInBounds(inputTransform);
            % Obtain current joint transforms, get the first link transform
            [~, allJointTransforms] = self.model.fkine(currentJointAngles);
            link1Transform = allJointTransforms(:,:,1);
            % Input transform is in global frame. Make the origin to be the
            % base of link 1 (not linear rail) to get a relative angle
            relativeTransform = (link1Transform)\inputTransform;
            % Obtain azimuth for current position to desired transform
            currentAzimuth = self.GetAzimuth(relativeTransform);
            % Check if within reach of robot arm - not including linear rail
            % because the arm joints are much faster  than the rail, and 
            % ideally we just use them
            inRangeCheck = self.CheckInRange(relativeTransform);
            % Default values to use for guess pose
            currentLinRailPos = currentJointAngles(1);
            link3 = deg2rad(30);
            link4 = deg2rad(70);
            link5 = pi/2 - link3 - link4;
            % If already in range, dont use the lin rail because it is much
            % slower. If not yet in range but it is in the workspace, move
            % the linear rail to the minimum extent to be in range, and then give a guess
            if inRangeCheck == 1
                modelGuessPose = [currentLinRailPos, currentAzimuth, link3, link4, link5, -currentAzimuth];
            elseif inBoundaryCheck == 1
                % The linear rail must be used to reach the point. Create a
                % new guess link1 location along the rail for the guess
                relativeY = inputTransform(2,4) - self.model.base(2,4);
                if relativeY > 0.265 % Limit at 0.265 for the sake of the guess pose
                    relativeY = 0.265;
                end
                newBaseDX = sqrt((0.265)^2 - (relativeY)^2);
                % Ready to obtain new Link1 transform depending on location
                % of the point along the linear rail
                if (inputTransform(1,4) >= (self.model.base(1,4) + -0.5*self.model.qlim(1))) % Global X coord of input point positive half of rail
                    newBaseTransform = link1Transform;
                    newBaseTransform(1,4) = inputTransform(1,4) - newBaseDX;
                    % Make sure the new X location is not beyond the rail
                    if newBaseTransform(1,4) > self.model.base(1,4) + -self.model.qlim(1)
                        newBaseTransform(1,4) = self.model.base(1,4) + -self.model.qlim(1);
                    end
                else % Global X coord of input point negative half of rail
                    newBaseTransform = link1Transform;
                    newBaseTransform(1,4) = inputTransform(1,4) + newBaseDX;
                    % Make sure the new X location is not beyond the rail
                    if newBaseTransform(1,4) < self.model.base(1,4)
                        newBaseTransform(1,4) = self.model.base(1,4);
                    end
                end
                % Obtain a new relative transform with new Link1 transform
                newRelativeTransform = (newBaseTransform)\inputTransform;
                % Obtain new azimuth for new Link1 transform
                newAzimuth = self.GetAzimuth(newRelativeTransform);
                % Obtain the new position of the linear rail
                newLinRailPos = newBaseTransform(1,4) - self.model.base(1,4);
                % Finally, we can obtain the guess pose
                modelGuessPose = [-newLinRailPos, newAzimuth,link3, link4, link5, -newAzimuth];
            else % not in the robot workspace. Return current pose for guess
                modelGuessPose = currentJointAngles;
            end
        end
        %% Function to get azimuth from desired robot base location (LINK 1 NOT LINEAR RAIL) to a point
        function azimuth = GetAzimuth(self, relativeTransform)
            % Get an X and Y variable relative to the robot (in robot's
            % co-ordinate space, not the same as the global co-ord space)
            relativeXDist = relativeTransform(1,4);
            relativeYDist = relativeTransform(2,4);
            % Create a case for every quadrant to determine the yaw angle
            % (azimuth) from the link 1 transform to the desired point
            if (relativeXDist <= 0) && (relativeYDist >= 0)
                azimuth = atan(relativeYDist/relativeXDist);
            elseif (relativeXDist <= 0) && (relativeYDist < 0)
                azimuth = atan(relativeYDist/relativeXDist);
            elseif (relativeXDist > 0) && (relativeYDist <= 0)
                azimuth = (atan(relativeYDist/relativeXDist) + pi);
            elseif (relativeXDist > 0) && (relativeYDist > 0)
                azimuth = (atan(relativeYDist/relativeXDist) - pi);
            end
        end
        %% Function to determine if a point is within the robot's immediate range (not including linear rail reach)
        function inRange = CheckInRange(self, relativeTransform)
            % Convert the X and Y distances to the relative desired point
            % into a single normal distance perpendicular to the Z axis
            pointNormalDist = sqrt((relativeTransform(1,4))^2 + (relativeTransform(2,4))^2);
            pointZDist = relativeTransform(3,4);
            % Define requirements of point to be in reach
            % 1: Be in top left quadrant of circle 1
            pointDistance1 = sqrt((pointNormalDist - self.rangeCircle1.normalDistOrigin)^2 + (pointZDist - self.rangeCircle1.zDistOrigin)^2);
            requirement1 = (pointZDist >= 0) && (pointDistance1 <= self.rangeCircle1.radius);
            % 2: Be in circle 2
            pointDistance2 = sqrt((pointNormalDist - self.rangeCircle2.normalDistOrigin)^2 + (pointZDist - self.rangeCircle2.zDistOrigin)^2);
            requirement2 = (pointDistance2 <= self.rangeCircle2.radius);
            % 3: Do not be in circle 3
            pointDistance3 = sqrt((pointNormalDist - self.rangeCircle3.normalDistOrigin)^2 + (pointZDist - self.rangeCircle3.zDistOrigin)^2);
            requirement3 = ~(pointDistance3 <= self.rangeCircle3.radius);
            % 4: Do not be in circle 4
            pointDistance4 = sqrt((pointNormalDist - self.rangeCircle4.normalDistOrigin)^2 + (pointZDist - self.rangeCircle4.zDistOrigin)^2);
            requirement4 = ~(pointDistance4 <= self.rangeCircle4.radius);
            % Do not have azimuth outside joint limits
            azimuth = self.GetAzimuth(relativeTransform);
            requirement5 = (azimuth <= self.model2.qlim(1,2)) && (azimuth >= self.model2.qlim(1,1));
            % Finally, meet the requirements as follows:
            inRange = (requirement1 || requirement2) && requirement3 && requirement4 && requirement5;
        end
        %% Function to determine if a point is within the robot's entire workspace
        function inBoundary = CheckInBounds(self, inputTransform)
            % Extract an X, Y, and Z from desired global transform
            pointX = inputTransform(1, 4);
            pointY = inputTransform(2, 4);
            pointZ = inputTransform(3, 4);
            % Define some useful limits for more efficient determination
            linearRailGlobalMax = self.model.base(1,4) + -self.model.qlim(1);
            linearRailGlobalMin = self.model.base(1,4);
            robotMaxReach = 0.335;
            robotYGlobalMax = self.model.base(2,4) + robotMaxReach;
            robotYGlobalMin = self.model.base(2,4) - 0.230;
            robotZGlobalMax = self.model.base(3,4) + 0.280;
            robotZGlobalMin = 0;
            % Create a local transform for any point with an appropriate
            % base location for the robot, ie. where the normal distance
            % becomes equal to the robot reach
            relativeY = pointY - self.model.base(2,4);
            if (pointZ >= (0.03233 + self.model.base(3,4))) % the point where the max reach is defined by either circle 1 or 2
                % use circle 1 perimeter
                normDist = sqrt((self.rangeCircle1.radius)^2 - (pointZ - self.rangeCircle1.zDistOrigin - self.model.base(3,4))^2) ...
                                + self.rangeCircle1.normalDistOrigin - 0.01;
            else
                % use circle 2 perimeter
                normDist = sqrt((self.rangeCircle2.radius)^2 - (pointZ - self.rangeCircle2.zDistOrigin - self.model.base(3,4))^2) ...
                                + self.rangeCircle2.normalDistOrigin - 0.01;
            end
            if (relativeY > normDist) || (relativeY < -normDist) % Set a limit for the Y value so that we don't sqrt negative number
                relativeY = normDist;
            end
            relativeX = sqrt((normDist)^2 - (relativeY)^2);
            if (pointX >= (self.model.base(1,4) + -0.5*self.model.qlim(1)))
                idealBaseTransform = self.model.base * rpy2tr(-pi/2,0,0);
                idealBaseTransform(1,4) = pointX - relativeX;
                if (idealBaseTransform(1,4) > linearRailGlobalMax)
                    idealBaseTransform(1,4) = linearRailGlobalMax;
                end
            else % ie. pointX is negative global
                idealBaseTransform = self.model.base * rpy2tr(-pi/2,0,0);
                idealBaseTransform(1,4) = pointX + relativeX;
                if (idealBaseTransform(1,4) < linearRailGlobalMin)
                    idealBaseTransform(1,4) = linearRailGlobalMin;
                end
            end
            % Take inverse to get the relative transform
            relativeTransform = (idealBaseTransform)\inputTransform;
            % First check if the point is within reasonable limits, then
            % check if the point is within reach using CheckInReach()
            if (pointY <= robotYGlobalMax) && (pointY >= robotYGlobalMin) ...
                    && (pointX <= (linearRailGlobalMax + robotMaxReach)) && (pointX >= (linearRailGlobalMin - robotMaxReach)) ...
                    && (pointZ <= robotZGlobalMax) && (pointZ >= robotZGlobalMin)
                if self.CheckInRange(relativeTransform) == 1
                    inBoundary = 1;
                else
                    inBoundary = 0;
                end
            else % ie. not in any reasonable range
                inBoundary = 0;
            end
        end
    end    
end