classdef Kinect < handle
    properties (Access = public)
        tKinect     % global location of Kinect
        arPoseSub   % ROS subscriber to /tf topic     
        msgs        % Struct array of 
    end
    methods
        function self = Kinect()
            %default kinect transform
            self.tKinect = [1.0000         0         0   -0.2000;...
                                 0   -1.0000   -0.0000    0.2170;...
                                 0    0.0000   -1.0000    0.8828;...
                                 0         0         0    1.0000];
            
            %subscribe to ros topic
            self.arPoseSub = rossubscriber("/tf",@self.ArCallback ,"BufferSize", 20);
        end
        
        function SetCalibrationTransform(self, inputTransform)
            self.tKinect = inputTransform;
        end
        
        function FindCalibrationTransform(self, tGlobeMarker)
            tCamMarker = self.GetTargetRaw(0);
            self.tKinect = tGlobeMarker/tCamMarker;
        end
        
        function tRaw = GetTargetRaw(self, selectedTag)
            errorFlag = 1;
            sample = self.msgs;
            num = length(sample);
            
            disp('searching for tag')
            
            for i = 1:num
                ID = sample(i).msg.Transforms.ChildFrameId;
                ID = split(ID,"_");
                ID = ID(3);
                ID = ID{1,1};
                ID = str2double(ID);
%                 disp(ID)
                if ID == selectedTag
                    index = i;
%                     disp("index = "+ i)
                    errorFlag = 0;
                    break
                end
            end
            
            if errorFlag == 1
                disp('Error, requested tag not found')
                tRaw = 0;
                return
            end
            translation = sample(index).msg.Transforms.Transform.Translation;
            translationT = transl(translation.X, translation.Y, translation.Z);
           
            rotation = sample(index).msg.Transforms.Transform.Rotation;
            quaternion = [rotation.W,rotation.X,rotation.Y,translation.Z];
            euler = quat2eul(quaternion);
            rotationT = rpy2tr(euler);
            
            tRaw = translationT*rotationT            
        end
        
        function tGlobe = RetrieveFood(self, selectedTag)
            tRaw = self.GetTargetRaw(selectedTag);
            tGlobe = self.tKinect * tRaw;
        end
        
        function [tGlobe, tag] = StoreFood(self, storedTags)
            errorFlag = 0;
            store = length(storedTags);
            
            num = 14;
            msg = cell(num);
            for i = 1:num
                msg(i) = recieve(self.arPoseSub);
            end
            
            for i = 1:num
                ID = msg.Transforms.ChildFrameId;
                ID = split(ID,"_");
                ID = ID(3);
                ID = ID{1,1};
                ID = str2double(ID);
                for j = 1:store
                    if ID ~= storedTags(j)
                        index = i;
                        tag = ID;
                    else
                        errorFlag = 1;
                    end
                end
            end
            
            if errorFlag == 1
                disp('ID already exists/not found')
                return
            end
           
            translation = msg(index).Transforms.Transform.Translation;
            translationT = transl(translation.X, translation.Y, translation.Z);
           
            rotation = msg(index).Transforms.Transform.Rotation;
            quaternion = [rotation.W,rotation.X,rotation.Y,translation.Z];
            euler = quat2eul(quaternion);
            rotationT = rpy2tr(euler);
            
            tRaw = translationT*rotationT; 
            
            tGlobe = self.tKinect * tRaw;
            
        end
        
        function Test(self)
            for i = 1:length(self.msgs)
                self.msgs(i).msg.Transforms.ChildFrameId
            end
            
        end
%         
        function ArCallback(self, ~, message)
            self.msgs(end+1).msg = message;
            if length(self.msgs) > 20
                self.msgs(1) = [];
            end
        end
    end
end
