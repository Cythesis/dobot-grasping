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

classdef Dobot < handle
    
    properties
        % Use for SerialLink() robot object
        model;
        % Set some default joint states
        jointStateDefault = [0, deg2rad([0, 45, 45, 0, 0])];
        jointStateZero = [0, 0, 0, 0, 0, 0];
        % Give a name for reference
        name = 'Dobot';
        % Set a workspace size
        workspace;
        % Set some range parameters for boundary checks: 
        % Ie. in range if in top left circle 1 and also circle 2, but while not in circle 3 and 4. 
        % Distances to circle origins measured from origin of Dobot's link 1
        rangeCircle1 = struct('radius', 0.27959, 'zDistOrigin', 0.00000, 'normalDistOrigin', 0.05254);
        rangeCircle2 = struct('radius', 0.12310, 'zDistOrigin', 0.00177, 'normalDistOrigin', 0.21102);
        rangeCircle3 = struct('radius', 0.14700, 'zDistOrigin', 0.13449, 'normalDistOrigin', 0.06431);
        rangeCircle4 = struct('radius', 0.14592, 'zDistOrigin', -0.15009, 'normalDistOrigin', 0.06796);
        % Set a default number of steps for unspecified paths
        defaultSteps = 50;
    end
        
    methods
        %% Constructor function
        function self = Dobot(baseTransformInput, workspaceInput)
            % Create default case for insufficient input arguments, extend
            % if more arguments are included
            defaultTransform = transl(0,0,0) * rpy2tr(0,0,0);
            defaultWorkspace = [-1, 1, -1, 1, 0, 1];
            switch nargin
                case 2
                    self.workspace = workspaceInput;
                case 1
                    if isempty(baseTransformInput)
                        baseTransformInput = defaultTransform;
                    elseif isempty(workspaceInput)
                        self.workspace = defaultWorkspace;
                    end
                case 0
                    baseTransformInput = defaultTransform;
                    self.workspace = defaultWorkspace;
            end
            % Run constructor functions
            self.GetDobot(baseTransformInput);
            self.RenderDobot();
        end
        %% Function to create a SerialLink() object with DH parameters
        function GetDobot(self, baseTransform)
            % Define DH parameters for SerialLink()
            L(1) = Link([0     0       0       -pi/2    1]); % Prismatic link 'sigma', 1,     'a', -0.133,    'alpha', pi/2   
            L(2) = Link('d', 0.08,      'a', 0,         'alpha', pi/2   );
            L(3) = Link('d', 0,         'a', -0.135,    'alpha', 0, 'offset', -pi/2       );
            L(4) = Link('d', 0,         'a', -0.147,    'alpha', 0      );
            L(5) = Link('d', 0,         'a', -0.05254,  'alpha', -pi/2  );
            L(6) = Link('d', -0.08,      'a', 0,         'alpha', 0     ); % End-effector offset
            
            % Define joint limits for SerialLink(), based on suggested
            L(1).qlim =         [   -1,     0     ];
            L(2).qlim = deg2rad([   -135,   135   ]);
            L(3).qlim = deg2rad([   5,      80    ]);
            L(4).qlim = deg2rad([   15,     170   ]);
            L(5).qlim = deg2rad([   -90,    90    ]);
            L(6).qlim = deg2rad([   -85,    85    ]);
            
            % Create SerialLink() object
            self.model = SerialLink(L, 'name', self.name);
            % Adjust base location
            self.model.base = baseTransform * transl(-0.5,0,0.133) * rpy2tr(pi/2, -pi/2, 0);
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
            self.model.plot3d(self.jointStateDefault, 'noarrow', 'workspace', self.workspace);
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
        %% Function to convert model joint space to real robot joint space
        function realJointAngles = GetRealJointAngles(self, modelJointAngles)
            realJointAngles = zeros(1, (size(modelJointAngles, 2) - 1)); % Joint 5 will be ommitted
            realJointAngles(1, 1) = modelJointAngles(1) * -1; % Reverse direction of linear rail (robot toolbox only likes negative prismatic values)
            realJointAngles(1, 2) = modelJointAngles(2);
            realJointAngles(1, 3) = modelJointAngles(3);    % Angle does not require adjustment
            realJointAngles(1, 4) = modelJointAngles(4) + modelJointAngles(3) - (pi/2); % adjust, accounting for mechanical linkage
            realJointAngles(1, 5) = modelJointAngles(6);    % Servo angle is the same (joint 5 ommitted) 
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
            relativeTransform = inv(link1Transform) * inputTransform;
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
                if relativeY < 0.265 % Limit at 0.265 for the sake of the guess pose
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
                newRelativeTransform = inv(newBaseTransform) * inputTransform;
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
            % Finally, meet the requirements as follows:
            inRange = (requirement1 || requirement2) && requirement3 && requirement4;
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
            relativeTransform = inv(idealBaseTransform) * inputTransform;
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
        %% Function to obtain model joint angles from a desired end-effector transform with ikcon
        function modelJointAngles = GetRobotPose_Ikcon(self, currentJointAngles, inputTransform)
            [modelGuessPose, inBoundaryCheck] = self.GetGuessPose(currentJointAngles, inputTransform);
            if inBoundaryCheck == 1
                [modelJointAngles, error, ~] = self.model.ikcon(inputTransform, modelGuessPose);
                if error > 0.05
                    disp("The output transform may be significantly inaccurate.")
                end
            else
                disp("The target transform is not in range.")
                modelJointAngles = currentJointAngles;
            end
        end
        %% Function to obtain model joint angles from a desired end-effector transform with ikine
        function modelJointAngles = GetRobotPose_Ikine(self, currentJointAngles, inputTransform)
            [modelGuessPose, inBoundaryCheck] = self.GetGuessPose(currentJointAngles, inputTransform);
            if inBoundaryCheck == 1
                modelJointAngles = self.model.ikine(inputTransform, modelGuessPose);
            else
                disp("The target transform is not in range.")
                modelJointAngles = currentJointAngles;
            end
        end
        %% Function to obtain model joint angles from a desired end-effector transform with both ikine and ikcon
        function [modelJointAngles_ikcon, modelJointAngles_ikine, modelGuessPose] = GetRobotPose(self, currentJointAngles, inputTransform)
            % Use function to obtain an initial guess for inverse
            % kinematics
            [modelGuessPose, inBoundaryCheck] = self.GetGuessPose(currentJointAngles, inputTransform);
            % Provided the input transform is in range, perform inverse
            % kinematics with both ikine and ikcon. Guess pose typically takes
            % care of most joint angle conditions that we care about for ikine.
            if inBoundaryCheck == 1
                % ikcon solution
                try [modelJointAngles_ikcon, error, ~] = self.model.ikcon(inputTransform, modelGuessPose);
                    % there are cases where guess pose fails and returns
                    % complex solutions. Must ensure this is not the case:
                    complexCheck = isreal(modelJointAngles_ikcon);
                    if (complexCheck == 0)
                        % if it is the case, provide the current pose as
                        % the guess instead.
                        [modelJointAngles_ikcon, error, ~] = self.model.ikcon(inputTransform, currentJointAngles);
                    end
                catch 
                    disp("An error occurred when running ikcon. ")
                    modelJointAngles_ikcon = currentJointAngles;
                    error = 1;
                end
                % detect end-effector error with some threshold
                if error > 0.05
                    disp("The ikcon output transform may be significantly inaccurate.")
                end
                % ikine solution
                try modelJointAngles_ikine = self.model.ikine(inputTransform, modelGuessPose);
                    % there are cases where guess pose fails and returns
                    % complex solutions. Must ensure this is not the case:
                    complexCheck = isreal(modelJointAngles_ikine);
                    if (complexCheck == 0)
                        % if it is the case, provide the current pose as
                        % the guess instead.
                        mask = [1, 1, 1, 0, 0, 0];
                        modelJointAngles_ikine = self.model.ikine(inputTransform, currentJointAngles, mask);
                    end
                catch
                    disp("An error occurred when running ikine. ")
                    modelJointAngles_ikine = currentJointAngles;
                end
            else
                disp("The target transform is not in range.")
                % Return the same initial joint configuration if the target
                % is not in range
                modelJointAngles_ikcon = currentJointAngles;
                modelJointAngles_ikine = currentJointAngles;
            end
        end
        %% Function to obtain a path of joint angles to a desired end-effector transform with ikcon
        function modelJointPath = GetJointPathQT_Ikcon(self, currentJointAngles, inputTransform, steps)
            if nargin ~= 4
                steps = self.defaultSteps;
            end
            finalJointAngles = self.GetRobotPose_Ikcon(currentJointAngles, inputTransform);
            modelJointPath = jtraj(currentJointAngles, finalJointAngles, steps);
            if modelJointPath(:, 1) > 0
                modelJointPath(:, 1) = 0;
            end
            if modelJointPath(:, 1) < self.model.qlim(1)
                modelJointPath(:, 1) = self.model.qlim(1);
            end
        end
        %% Function to obtain a path of joint angles to a desired end-effector transform with ikine
        function modelJointPath = GetJointPathQT_Ikine(self, currentJointAngles, inputTransform, steps)
            if nargin ~= 4
                steps = self.defaultSteps;
            end
            finalJointAngles = self.GetRobotPose_Ikine(currentJointAngles, inputTransform);
            modelJointPath = jtraj(currentJointAngles, finalJointAngles, steps);
            if modelJointPath(:, 1) > 0
                modelJointPath(:, 1) = 0;
            end
            if modelJointPath(:, 1) < self.model.qlim(1)
                modelJointPath(:, 1) = self.model.qlim(1);
            end
        end
        %% Function to obtain a path of joint angles to a desired end-effector transform with both ikine and ikcon
        function modelJointPath = GetJointPathQT(self, currentJointAngles, inputTransform, steps)
            % If steps are not specified, use the default steps 
            if nargin ~= 4
                steps = self.defaultSteps;
            end
            % Obtain solutions for ikcon and ikine of the final transform
            [ikconJointAngles, ikineJointAngles, guessJointAngles] = self.GetRobotPose(currentJointAngles, inputTransform);
            
            % Begin final joint angle selection criteria:
            % Requirement 1: ikine solution is within joint limits; check all joints
            jointLimitRequirement = zeros(1, (self.model.n + 1));
            for i = 1:self.model.n
                lowerLimit = self.model.qlim(i,1);
                upperLimit = self.model.qlim(i,2);
                if (ikineJointAngles(i) < lowerLimit) || (ikineJointAngles(i) > upperLimit)
                    jointLimitRequirement(i) = 0;
                else
                    jointLimitRequirement(i) = 1;
                end
            end
            % Also check link5 = pi/2 - link3 - link4 within
            % reasonable threshold
            linkCheck = ikineJointAngles(5) + ikineJointAngles(3) + ikineJointAngles(4) - pi/2;
            if (linkCheck > 0.05) || (linkCheck < -0.05)
                jointLimitRequirement(end) = 0;
            else
                jointLimitRequirement(end) = 1;
            end
            % Requirement 2: ikine solution is closer to desired end-effector
            % position than ikcon solution - or they are both very close
            ikineEndEffTr = self.model.fkine(ikineJointAngles);
            ikconEndEffTr = self.model.fkine(ikconJointAngles);
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
            idealAzimuth = guessJointAngles(2);
            if (abs(ikineJointAngles(2) - idealAzimuth) < abs(ikconJointAngles(2) - idealAzimuth))
                angleRequirement = 1;
            else % ie. ikcon solution closer to ideal azimuth than ikine
                angleRequirement = 0;
            end
            % Check on requirement 1:
            if all(jointLimitRequirement == 1)
                % Check on requirement 2:
                if (distanceRequirement == 1)
                    selectedJointAngles = ikineJointAngles;
                    disp("Using Ikine")
                elseif (distanceRequirement == 2)
                    % Check on requirement 3:
                    if (angleRequirement == 1)
                        selectedJointAngles = ikineJointAngles;
                        disp("Using Ikine")
                    else % ie. angle requirement == 0
                        selectedJointAngles = ikconJointAngles;
                        disp("Using Ikcon")
                    end
                else % ie. dist requirement == 0
                    selectedJointAngles = ikconJointAngles;
                    disp("Using Ikcon")
                end
            else % ie. joint requirement == 0
                selectedJointAngles = ikconJointAngles;
                disp("Using Ikcon")
            end
            
            % Begin joint angle trajectory generation
            modelJointPath = jtraj(currentJointAngles, selectedJointAngles, steps);
            % Ensure that the linear rail joint limits are not breached in
            % the trajectory even by a small margin
            modelJointPath((modelJointPath(:, 1) > self.model.qlim(1,2)), 1) = 0;
            modelJointPath((modelJointPath(:, 1) < self.model.qlim(1,1)), 1) = self.model.qlim(1,1);
        end
    end    
end