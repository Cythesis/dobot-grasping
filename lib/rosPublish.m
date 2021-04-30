classdef rosPublish
    properties
        
    end
    methods
        function sendTraj(obj,pos)
            channel_1 = rospublisher("/target_joint_trajectory","trajectory_msgs/JointTrajectory");
            msg = rosmessage(channel_1);
            point = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            point.Positions = pos;
            msg.Points(1) = point;
            pause(1)
            send(channel_1,msg);
        end
        
        function sendCart(obj,x,y,z,qw,qx,qy,qz)
            channel_1 = rospublisher("/target_pose_trajectory","geometry_msgs/Pose");
            msg = rosmessage(channel_1);
            point = rosmessage('geometry_msgs/Point');
            quaternion = rosmessage('geometry_msgs/Quaternion');
            point.X = x;
            point.Y = y;
            point.Z = z;
            quaternion.X = qx;
            quaternion.Y = qy;
            quaternion.Z = qz;
            quaternion.W = qw;
            
            msg.Position(1) = point;
            msg.Orientation(1) = quaternion;
            
            send(channel_1,msg);
        end
    end
end

