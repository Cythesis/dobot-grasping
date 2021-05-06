classdef RosPublish < handle
    properties(Access = private)
        % Subscriber objects
        arPoseSub;
        
        % Publisher objects 
        targetJointPub;
        targetPosePub;
        statusPub;
        toolStatePub;
        railStatusPub;
        railPosPub;
        
        % Publisher messages
        targetJointMsg;
        targetPoseMsg;
        statusMsg;
        toolStateMsg;
        railStatusMsg;
        railPosMsg;
    end
    methods(Access = public)
        
        function self = RosPublish()
            % Create ros subscribers
            self.arPoseSub = rossubscriber('/tf');
            % Create ros publishers
%             [self.targetJointPub,self.targetJointMsg] = rospublisher('/dobot_magician/target_joint_states');
%             [self.targetPosePub,self.targetPoseMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
%             [self.statusPub,self.statusMsg] = rospublisher('/dobot_magician/target_safety_status');
%             [self.toolStatePub, self.toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
%             [self.railStatusPub, self.railStatusMsg] = rospublisher('/dobot_magician/target_rail_status');
%             [self.railPosPub,self.railPosMsg] = rospublisher('/dobot_magician/target_rail_position');
        end
        
%         function MoveJoint(self, joint)                                     % Message type joint = [a,b,c,d];
%            Point = rosmessage("trajectory_msgs/JointTrajectoryPoint");      % Create message
%            Point.Positions = joint;                                         % Fill in message 
%            self.targetJointTrajMsg.Points = Point;                          % Fill message in msg object
%            send(self.targetJointTrajPub,self.targetJointTrajMsg);           % Send the message
%         end
%         
%         function MovePose(self, pose, rotation)                             % Message type pose = [x,y,z];
%            self.targetPoseMsg.Position.X = pose(1);                         % Message type rotation = [qw,qx,qy,qz];
%            self.targetPoseMsg.Position.Y = pose(2);
%            self.targetPoseMsg.Position.Z = pose(3);
%            qua = eul2quat(rotation);
%            self.targetPoseMsg.Orientation.W = qua(1);
%            self.targetPoseMsg.Orientation.X = qua(2);
%            self.targetPoseMsg.Orientation.Y = qua(3);
%            self.targetPoseMsg.Orientation.Z = qua(4);
%            
%            send(self.targetPosePub,self.targetPoseMsg);
%         end
%         
%        function InitaliseRobot(self)
%             self.statusMsg.Data = 2; 
%             send(self.statusPub,self.statusMsg);
%        end
% 
%        function EStopRobot(self)
%             self.statusMsg.Data = 3; 
%             send(self.statusPub,self.statusMsg);
%        end
%        
%        function RailOn(self,status)
%            self.railStatusMsg.Data = status;
%            send(self.railStatusPub,self.railStatusMsg);
%        end
%        
%        function MoveRail(self,pos)
%            self.railPosMsg.Data = pos;
%            send(self.railPosPub,self.railPosMsg);
%        end
       
       function [ID, pose, euler] = GetCurrentArPose(self)
           msg = self.arPoseSub.LatestMessage;
           
           ID = msg.Transforms.ChildFrameId;
           ID = split(ID,"_");
           ID = ID(3);
           ID = ID{1,1};
           ID = str2double(ID);
           
           translation = msg.Transforms.Transform.Translation;
           pose = [translation.X,translation.Y,translation.Z];
           
           rotation = msg.Transforms.Transform.Rotation;
           quaternion = [rotation.W,rotation.X,rotation.Y,translation.Z];
           euler = quat2eul(quaternion);
       end
    end
end

