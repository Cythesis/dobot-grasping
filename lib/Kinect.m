classdef Kinect < handle
    properties (Access = public)
        tKinect     % global location of Kinect
        arPoseSub   % ROS subscriber to /tf topic     
        msgs        % Struct array of ROS messages retrieved from callback
        buffer      % buffer size for ROS callback
    end
    methods
        %Constructor for Kinect class initialises class and sets default
        %values. Also servs to start callback function which retrieves pose
        %messages of AR tags.
        function self = Kinect(kinectToggle)
            if (kinectToggle == 1)
                %default kinect transform
                self.tKinect = [1.0000         0         0   -0.2300;...
                                     0   -1.0000   -0.0000    0.2170;...
                                     0    0.0000   -1.0000    0.9000;...
                                     0         0         0    1.0000];
                
                %Set buffer size for callback
                self.buffer = 20;
                
                %subscribe to ros topic
                self.arPoseSub = rossubscriber("/tf",@self.ArCallback ,"BufferSize", self.buffer);
            end
        end
        
        %Set function to manually change global transform of kinect.
        function SetCalibrationTransform(self, inputTransform)
            self.tKinect = inputTransform;
        end
        
        %Function that calculates the pose of the kinect using a global
        %pose of a marker and the kinect frame pose of the same marker.
        function FindCalibrationTransform(self, tGlobeMarker)
             tRaw = self.GetTargetRaw(0);
             tCamMarker = transl(tRaw(1,4),tRaw(2,4),-tRaw(3,4));
             tNew = tGlobeMarker*inv(tCamMarker);
             self.tKinect(1,4) = tNew(1,4);
             self.tKinect(2,4) = tNew(2,4);
             self.tKinect(3,4) = tNew(3,4);
        end
        
        %Returns the camera frame pose of a selected marker
        function tRaw = GetTargetRaw(self, selectedTag)
            errorFlag = 1;
            sample = self.msgs;
            num = length(sample);
            
            % -- Averaging -- %
%             ave = 0;
%             translationSum = zeros(4,4);
%             rotationSum = zeros(4,4);
            % -- Averaging -- %
            
            disp("Searching for tag... ")
            for i = 1:num
                ID = sample(i).msg.Transforms.ChildFrameId;
                ID = split(ID,"_");
                ID = ID(3);
                ID = ID{1,1};
                ID = str2double(ID);
                if ID == selectedTag
                    % -- Averaging -- %
%                     ave = ave + 1;
%                     translation = sample(i).msg.Transforms.Transform.Translation;
%                     translationSum = translationSum + transl(translation.X, translation.Y, translation.Z);
% 
%                     rotation = sample(i).msg.Transforms.Transform.Rotation;
%                     quaternion = [rotation.W,rotation.X,rotation.Y,translation.Z];
%                     euler = quat2eul(quaternion);
%                     rotationSum = rotationSum + rpy2tr(euler(3:-1:1));
                    % -- Averaging -- %
                    
                    % -- No Averaging -- %
                    index = i;
                    % -- No Averaging -- %
                    
                    errorFlag = 0;
                end
            end
            
            if errorFlag == 1
                disp('Error, requested tag not found')
                tRaw = 0;
                return
            end
            
            % -- Averaging -- %
%             translationAve = translationSum/ave;
%             rotationAve = rotationSum/ave;
%             
%             tRaw = translationAve*rotationAve;
            % -- Averaging -- %

            % -- No Averaging -- %
            translation = sample(index).msg.Transforms.Transform.Translation;
            translationT = transl(translation.X, translation.Y, translation.Z);

            rotation = sample(index).msg.Transforms.Transform.Rotation;
            quaternion = [rotation.W,rotation.X,rotation.Y,translation.Z];
            euler = quat2eul(quaternion);
            rotationT = rpy2tr(euler(3:-1:1));
            
            tRaw = translationT*rotationT;
            % -- No Averaging -- %
        end
        
        %Function used to collect items from the pantry to user.
        %Returns the global frame pose of a selected marker using the
        %camera frame pose and the global pose of the kinect.
        function tGlobe = RetrieveFood(self, selectedTag)
            tRaw = self.GetTargetRaw(selectedTag);
            tGlobe = self.tKinect * tRaw;
        end
        
        %Function used to add an item to the pantry.
        %Iterates through a vector of storred tags to find item not
        %currently storred, then returns the global pose and ID of the
        %new tag.
        function [tGlobe, tag] = StoreFood(self, storedTags)
            if isempty(storedTags)
                store = 1;
                storedTags = 999;
            else
                store = length(storedTags);
            end
            
            errorFlag = 1;
            sample = self.msgs;
            num = length(sample);
            
            % -- Averaging -- %
%             ave = 0;
%             translationSum = zeros(4,4);
%             rotationSum = zeros(4,4);
            % -- Averaging -- %
            
            for i = 1:num
                ID = sample(i).msg.Transforms.ChildFrameId;
                ID = split(ID,"_");
                ID = ID(3);
                ID = ID{1,1};
                ID = str2double(ID);
                for j = 1:store
                    if ID ~= storedTags(j)
                        % -- Averaging -- %
%                         ave = ave + 1;
%                         translation = sample(i).msg.Transforms.Transform.Translation;
%                         translationSum = translationSum + transl(translation.X, translation.Y, translation.Z);
% 
%                         rotation = sample(i).msg.Transforms.Transform.Rotation;
%                         quaternion = [rotation.W,rotation.X,rotation.Y,translation.Z];
%                         euler = quat2eul(quaternion);
%                         rotationSum = rotationSum + rpy2tr(euler(3:-1:1));
                        % -- Averaging -- %
                        % -- No Averaging -- %
                        index = i;
                        % -- No Averaging -- %
                        tag = ID;
                        errorFlag = 0;
                    end
                end
            end
            
            if errorFlag == 1
                disp('No new ID found')
                tag = -1;
                tGlobe = 0;
                return
            end
            % -- Averaging -- %
%             translationAve = translationSum/ave;
%             rotationAve = rotationSum/ave;
%             
%             tRaw = translationAve*rotationAve; 
            % -- Averaging -- %
            
            % -- No Averaging -- %
            translation = sample(index).msg.Transforms.Transform.Translation;
            translationT = transl(translation.X, translation.Y, translation.Z);

            rotation = sample(index).msg.Transforms.Transform.Rotation;
            quaternion = [rotation.W,rotation.X,rotation.Y,translation.Z];
            euler = quat2eul(quaternion);
            rotationT = rpy2tr(euler(3:-1:1));
            
            tRaw = translationT*rotationT;
            % -- No Averaging -- %
            
            tGlobe = self.tKinect * tRaw;
            
            self.msgs = [];
            
        end
        
        %Test function to display collected messages
        function Test(self)
            for i = 1:length(self.msgs)
                self.msgs(i).msg.Transforms.ChildFrameId
            end
            
        end
        
        %Callback function, run every time /tf topic receives a message.
        %Messages are saved in a class property for access by other
        %functions. Oldest messages are deleted to prevent array expanding
        %infinitly.
        function ArCallback(self, ~, message)
            self.msgs(end+1).msg = message;
            if length(self.msgs) > self.buffer
                self.msgs(1) = [];
            end
        end
    end
end
