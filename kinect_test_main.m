function kinect_test_main
    rosinit
    
    kinectCamera = Kinect;
    
    T = kinectCamera.GetTargetRaw(5);
    
    disp(T);
    
    rosshutdown
end