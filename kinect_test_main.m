function kinect_test_main
    rosinit
    
    kinectCamera = Kinect;
    
    T = kinectCamera.GetTargetRaw(0);
    
    disp(T);
    
    rosshutdown
end