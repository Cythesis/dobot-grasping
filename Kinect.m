classdef Kinect
    properties
        tKinect
        arPoseSub
        tag
    end
    methods
        function self = Kinect()
            self.tKinect = [1.0000         0         0         0;...
                                 0   -1.0000   -0.0000    0.2170;...
                                 0    0.0000   -1.0000    0.8828;...
                                 0         0         0    1.0000];
            %subscribe to ros topic
            self.arPoseSub = rossubscriber("/tf");
        end
        
%         function SetCalibrationTransform(self, inputTransform)
%             self.tKinect = inputTransform;
%         end
        
%         function FindCalibration(self)
%             
%         end
        
        function tRaw = GetTargetRaw(self, selectedTag)
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
                if ID == selectedTag
                    ind = i;
                end
            end
           
            translation = msg(ind).Transforms.Transform.Translation;
            translationT = transl(translation.X, translation.Y, translation.Z);
           
            rotation = msg(ind).Transforms.Transform.Rotation;
            quaternion = [rotation.W,rotation.X,rotation.Y,translation.Z];
            euler = quat2eul(quaternion);
            rotationT = rpy2tr(euler);
            
            tRaw = translationT*rotationT;            
        end
        
        function tGlobe = RetrieveFood(self, selectedTag)
            
        end
        
        function [tGlobe, tag] = StoreFood(self, storedTags)
            
        end
    end
end