classdef Kinect
    properties
        calibrationT
    end
    methods
        function self = Kinect()
            calibrationT = [0.0000   -1.0000    0.0000         0; ...
                            0.0000    0.0000    1.0000    0.2170; ...
                           -1.0000         0    0.0000    0.8828; ...
                                 0         0         0    1.0000];
            
        end
        
        function SetCalibration(self)
            
        end
        
        function FindCalibration(self)
            
        end
    end
end