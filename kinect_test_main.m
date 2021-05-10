function kinect_test_main
    clear
    rosinit
    
    kinectCamera = Kinect;
    
    T = kinectCamera.GetTargetRaw(0);
    disp('tag 0')
    disp(T);
    
    T = kinectCamera.GetTargetRaw(1);
    disp('tag 1')
    disp(T);
    
    T = kinectCamera.GetTargetRaw(2);
    disp('tag 2')
    disp(T);
    
    rosshutdown
end