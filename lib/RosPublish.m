classdef RosPublish < handle
    properties(Access = private)
        % Subscriber objects
        arPoseSub;
        jointStateSub;
        railStateSub;
        
        % Publisher objects 
        targetJointPub;
        targetPosePub;
        statusPub;
        toolStatePub;
        railStatusPub;
        railPosPub;
        beltPub;
        
        % Publisher messages
        targetJointMsg;
        targetPoseMsg;
        statusMsg;
        toolStateMsg;
        railStatusMsg;
        railPosMsg;
        beltMsg;
    end
    methods(Access = public)
        
        function self = RosPublish()
            % Create ros subscribers
%             self.arPoseSub = rossubscriber('/tf',@arPoseCallback);
            self.jointStateSub = rossubscriber('/dobot_magician/joint_states');
            self.railStateSub = rossubscriber('/dobot_magician/rail_position');
            % Create ros publishers
            [self.targetJointPub,self.targetJointMsg] = rospublisher('/dobot_magician/target_joint_states');
            [self.targetPosePub,self.targetPoseMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
            [self.statusPub,self.statusMsg] = rospublisher('/dobot_magician/target_safety_status');
            [self.toolStatePub, self.toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
            [self.railStatusPub, self.railStatusMsg] = rospublisher('/dobot_magician/target_rail_status');
            [self.railPosPub,self.railPosMsg] = rospublisher('/dobot_magician/target_rail_position');
            [self.beltPub,self.beltMsg] = rospublisher('/dobot_magician/target_e_motor_state');
        end
        
        function MoveJoint(self, joint)                                     % Message type joint = [a,b,c,d];
           Point = rosmessage("trajectory_msgs/JointTrajectoryPoint");      % Create message
           Point.Positions = joint;                                         % Fill in message 
           self.targetJointMsg.Points = Point;                          % Fill message in msg object
           send(self.targetJointPub,self.targetJointMsg);           % Send the message
           while 1
               a = self.GetJoint().Position;
               b = self.targetJointMsg.Points.Positions;
               pause(0.01);
               if (abs(a(1)-b(1)) < 0.01) && (abs(a(2)-b(2)) < 0.01) && (abs(a(3)-b(3)) < 0.01) && (abs(a(4)-b(4)) < 0.01)
                   break
               end
           end
           disp("All done")
        end
        
        
        function MovePose(self, pose, rotation)                             % Message type pose = [x,y,z];
           self.targetPoseMsg.Position.X = pose(1);                         % Message type rotation = [qw,qx,qy,qz];
           self.targetPoseMsg.Position.Y = pose(2);
           self.targetPoseMsg.Position.Z = pose(3);
           qua = eul2quat(rotation);
           self.targetPoseMsg.Orientation.W = qua(1);
           self.targetPoseMsg.Orientation.X = qua(2);
           self.targetPoseMsg.Orientation.Y = qua(3);
           self.targetPoseMsg.Orientation.Z = qua(4);
           
           send(self.targetPosePub,self.targetPoseMsg);
        end
        
       function InitaliseRobot(self)
            self.statusMsg.Data = 2; 
            send(self.statusPub,self.statusMsg);
       end

       function Estop(self)
            self.statusMsg.Data = 3; 
            send(self.statusPub,self.statusMsg);
       end
       
       function RailOn(self,status)
           self.railStatusMsg.Data = status;
           send(self.railStatusPub,self.railStatusMsg);
       end
       
       function MoveRail(self,pos)
           self.railPosMsg.Data = pos;
           send(self.railPosPub,self.railPosMsg);
           while 1
               
               a = self.GetRail().Data;
               b = self.railPosMsg.Data;
               pause(0.01);
               if abs(a-b) < 0.005
                   break
               end
           end
           disp("All done")
       end
       
%        function [ID, transform] = GetCurrentArPose(self)
%            msg = self.arPoseSub.LatestMessage;
%            
%            ID = msg.Transforms.ChildFrameId;
%            ID = split(ID,"_");
%            ID = ID(3);
%            ID = ID{1,1};
%            ID = str2double(ID);
%            
%            translation = msg.Transforms.Transform.Translation;
%            pose = [translation.X,translation.Y,translation.Z];
%            
%            rotation = msg.Transforms.Transform.Rotation;
%            quaternion = [rotation.W,rotation.X,rotation.Y,translation.Z];
%            euler = quat2eul(quaternion);
%            transform = transl(pose(1),pose(2),pose(2)) * rpy2tr(euler(1),euler(2),euler(3),euler(4));
%        end

       function [msg] = GetJoint(self)
           msg = self.jointStateSub.LatestMessage;
       end
       
       function [msg] = GetRail(self)
           msg = self.railStateSub.LatestMessage;
       end
       
       function MoveBelt(self,enabled,velocity)
            self.beltMsg.Data = [enabled,velocity];
            send(self.beltPub,self.beltMsg);
       end
       
       
    end 
end

