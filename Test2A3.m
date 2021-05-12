%% Simplified Dobot Test
% close all; clear all; clc; clf; 
% 
% links(1) = Link('d', 0.08,      'a', 0,         'alpha', pi/2   );
% links(2) = Link('d', 0,         'a', -0.135,    'alpha', 0, 'offset', -pi/2       );
% links(3) = Link('d', 0,         'a', -0.147,    'alpha', 0      );
% 
% links(1).qlim = deg2rad([   -135,   135   ]);
% links(2).qlim = deg2rad([   5,      80    ]);
% links(3).qlim = deg2rad([   15,     170   ]);
% 
% q0 = deg2rad([0, 45, 90]);
% 
% robot.model = SerialLink(links);
% robot.model.base = transl(0,0,0) * rpy2tr(0, 0, -pi/2);
% robot.model.delay = 0.00001;
% 
% robot.model.plot(q0)
% robot.model.teach()
% 
% controllerID = 1;
% 
% try DS4Controller = vrjoystick(controllerID);
% catch
%     disp("Simulink 3D Animation toolbox may not be installed, or controller ID may need to change. " )
%     return
% end
% 
% currentJointAngles = robot.model.getpos;
% dT = 0.25;      % Time step
% counter = 0;    % Loop counter
% axisGain = 0.02; % Amount for distance per tick gain on robot joints
% lambda = 0.1;   % Damping coefficient
% tic;            % Start time
% 
% while (1)
%     counter = counter + 1;
%     [axes, ~, ~] = read(DS4Controller);
%     
%     xVelocity = axisGain * axes(1);
%     yVelocty = axisGain * -axes(6);
%     zVelocity = axisGain * -axes(2);
%     
%     jacobian = robot.model.jacob0(currentJointAngles);
%     jacobian3X3 = jacobian(1:3, 1:3);
%     DLSinvJacob = (transpose(jacobian3X3) * (jacobian3X3 + lambda*eye(3))) \ transpose(jacobian3X3);
%     qDotX = (DLSinvJacob(:,1) * xVelocity)';
%     qDotY = (DLSinvJacob(:,2) * yVelocty)';
%     qDotZ = (DLSinvJacob(:,3) * zVelocity)';
%     % Store previous joint angles before updating
%     prevJointAngles = currentJointAngles;
%     % Apply joint velocities to increment joint angles
%     currentJointAngles = currentJointAngles + ((qDotX + qDotY + qDotZ) * dT);
%     
%     % Update simulation or real robot if any have changed:
%     if any(currentJointAngles - prevJointAngles)
%         robot.model.plot(currentJointAngles);
%     end
%     % Check loop time in case it is running longer than dT
%     if (toc > dT * counter)
%         disp("Loop time exceeded allocated time step in remote control function. ")
%     end
%     % Pause until dT has passed for this iteration
%     while (toc < dT * counter)
%     end
% end

%%
controller.StartRemoteControl()
