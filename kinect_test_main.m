% function kinect_test_main
    clear
    
    rosinit
    
    kinectCamera = Kinect
    arPoseSub = rossubscriber("/tf",@ArCallback ,"BufferSize", 15);
    
    disp('flag 2')
    
%     T = kinectCamera.GetTargetRaw(3);
%     disp('tag 0')
%     disp(T);
%     
%     T = kinectCamera.GetTargetRaw(4);
%     disp('tag 1')
%     disp(T);
%     
%     T = kinectCamera.GetTargetRaw(5);
%     disp('tag 2')
%     disp(T);
%     
%     rosshutdown
%     while 1
%         kinectCamera.Test()
%     end
% end

% function ArCallback(~, message, kinectCamera)
%     message.Transforms.ChildFrameId
%     kinectCamera.msgs = message;
% end