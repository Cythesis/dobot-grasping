function kinect_test_main
    rosinit
    
    kinectCamera = kinect.m;
    
    T = kinectCamera.GetTargetRaw(0);
    
    disp(T);
end