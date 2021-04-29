classdef CTAworkspace < handle
   properties
    %> Frame model
        model;
        
        %>
        workspace = [-10 10 -10 10 -10 10];   
   end
   %% area for CTAWorkspace
   methods  
       function obj = CTAworkspace()
           PlaceObject('Frame.ply', [0,0,0.7]);
           PlaceObject('1_container.ply', [2.8,-1.5,2.85]);
           PlaceObject('2_container.ply', [3.5,-1,1.5]);
           PlaceObject('3_container.ply', [5.5,-1.5,2.85]);
           PlaceObject('4_container.ply', [1,-1,1.5]);
           PlaceObject('Table.ply', [0,0,0.5]);
           PlaceObject('Table.ply', [0,0,0.5]);
       end
   
      
   end
   
end