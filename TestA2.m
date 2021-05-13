close all; clear all; clc; clf;

Dobot1 = Dobot();
Dobot1.model.teach();

%% Test section

q = Dobot1.model.getpos;
q(1) = -0.6;
Dobot1.model.animate(q);
[~, allT] = Dobot1.model.fkine(q);
link1T = allT(:,:,1);

%% Collision detector

q = [-0.65, deg2rad([0, 20,  70, 0, 0])];
[~, allT] = Dobot1.model.fkine(q);
Dobot1.model.animate(q);
Dobot1.model.teach()

% Make some obstacles
hold on
PlaceObject('ConveyorBelt.ply', [0.35, 0.217, 0.052]);
containerLink = Link('alpha',0,'a',0.001,'d',0,'offset',0);
container.model = SerialLink(containerLink, 'name', 'Container');
container.model.base = transl(0.175, 0.215, 0.16);
[faceData, vertexData, plyData] = plyread('Container1.ply','tri');
container.model.faces = {faceData, []};
container.model.points = {vertexData, []};
plot3d(container.model, 0,'noarrow');
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end 
handles = findobj('Tag', container.model.name);
h = get(handles,'UserData');
try
    h.link(1).Children.FaceVertexCData = [plyData.vertex.red ...
                                          , plyData.vertex.green ...
                                          , plyData.vertex.blue]/255;
    h.link(1).Children.FaceColor = 'interp';
catch
    disp("No colour in ply file")
end

check = Dobot1.CheckCollision(q, container)

% function check = SphereCollisions(linkTransforms, objectTransform, vertices)
% 
%     check = 0;
%     
%     %rotate(h, [0 0 1], 45);Dobot1.model.n
% 
%     linkEllipsoid = [];
%     radiuses = [];
%     centres = [];
% 
%     for linkIndex = 1:size(linkTransforms, 3)
%         switch linkIndex
%             case 1
%     %             centrePoint = [linkTransforms(1,4,linkIndex), -0.06, (linkTransforms(3,4,linkIndex) - 0.04)];
%     %             radii = [0.13, 0.16, (linkTransforms(3,4,linkIndex) + 0.025)/2];  
%     %             [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%     %             linkEllipsoids(linkIndex) = surf(x,y,z);
%             case 2
%     %             centrePoint = [(linkTransforms(1,4,linkIndex) + linkTransforms(1,4,linkIndex-1))/2, (linkTransforms(2,4,linkIndex) + linkTransforms(2,4,linkIndex-1))/2, (linkTransforms(3,4,linkIndex) + linkTransforms(3,4,linkIndex-1))/2 + 0.015];
%     %             radii = [0.1, 0.075, 0.1];
%     %             [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%     %             linkEllipsoids(linkIndex) = surf(x,y,z);
%                 centrePoint = [linkTransforms(1,4,linkIndex), linkTransforms(2,4,linkIndex), linkTransforms(3,4,linkIndex)];
%                 radii = [0.075, 0.075, 0.075];
%                 [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%                 linkEllipsoid(end+1) = surf(x,y,z);
%                 radiuses(size(linkEllipsoid,2)) = radii(1);
%                 centres(size(linkEllipsoid,2), 1:3) = centrePoint;
%             case 3
%                 centrePoint = [linkTransforms(1,4,linkIndex), linkTransforms(2,4,linkIndex), linkTransforms(3,4,linkIndex)];
%                 radii = [0.055, 0.055, 0.055];
%                 [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%                 linkEllipsoid(end+1) = surf(x,y,z);
%                 radiuses(size(linkEllipsoid,2)) = radii(1);
%                 centres(size(linkEllipsoid,2), 1:3) = centrePoint;
%                 centrePoint = [(linkTransforms(1,4,linkIndex) + linkTransforms(1,4,linkIndex-1))/2, (linkTransforms(2,4,linkIndex) + linkTransforms(2,4,linkIndex-1))/2, (linkTransforms(3,4,linkIndex) + linkTransforms(3,4,linkIndex-1))/2];
%                 [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%                 linkEllipsoid(end+1) = surf(x,y,z);
%                 radiuses(size(linkEllipsoid,2)) = radii(1);
%                 centres(size(linkEllipsoid,2), 1:3) = centrePoint;
%             case 4
%                 centrePoint = [linkTransforms(1,4,linkIndex), linkTransforms(2,4,linkIndex), linkTransforms(3,4,linkIndex)];
%                 radii = [0.04, 0.04, 0.04];
%                 [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%                 linkEllipsoid(end+1) = surf(x,y,z);
%                 radiuses(size(linkEllipsoid,2)) = radii(1);
%                 centres(size(linkEllipsoid,2), 1:3) = centrePoint;
%                 centrePoint = [(linkTransforms(1,4,linkIndex) + linkTransforms(1,4,linkIndex-1))/2, (linkTransforms(2,4,linkIndex) + linkTransforms(2,4,linkIndex-1))/2, (linkTransforms(3,4,linkIndex) + linkTransforms(3,4,linkIndex-1))/2];
%                 radii = [0.055, 0.055, 0.055];
%                 [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%                 linkEllipsoid(end+1) = surf(x,y,z);
%                 radiuses(size(linkEllipsoid,2)) = radii(1);
%                 centres(size(linkEllipsoid,2), 1:3) = centrePoint;
%             case 5
%                 centrePoint = [linkTransforms(1,4,linkIndex), linkTransforms(2,4,linkIndex), linkTransforms(3,4,linkIndex)];
%                 radii = [0.03, 0.03, 0.03];
%                 [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%                 linkEllipsoid(end+1) = surf(x,y,z);
%                 radiuses(size(linkEllipsoid,2)) = radii(1);
%                 centres(size(linkEllipsoid,2), 1:3) = centrePoint;
%                 centrePoint = [(linkTransforms(1,4,linkIndex) + linkTransforms(1,4,linkIndex-1))/2, (linkTransforms(2,4,linkIndex) + linkTransforms(2,4,linkIndex-1))/2, (linkTransforms(3,4,linkIndex) + linkTransforms(3,4,linkIndex-1))/2];
%                 [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%                 linkEllipsoid(end+1) = surf(x,y,z);
%                 radiuses(size(linkEllipsoid,2)) = radii(1);
%                 centres(size(linkEllipsoid,2), 1:3) = centrePoint;
%             case 6
%     %             centrePoint = [linkTransforms(1,4,linkIndex), linkTransforms(2,4,linkIndex), linkTransforms(3,4,linkIndex)];
%                 radii = [0.02, 0.02, 0.02];
%     %             [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%     %             linkEllipsoid6a = surf(x,y,z);
%                 centrePoint = [(linkTransforms(1,4,linkIndex) + linkTransforms(1,4,linkIndex-1))/2, (linkTransforms(2,4,linkIndex) + linkTransforms(2,4,linkIndex-1))/2, (linkTransforms(3,4,linkIndex) + linkTransforms(3,4,linkIndex-1))/2];
%                 [x,y,z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3));
%                 linkEllipsoid(end+1) = surf(x,y,z);
%                 radiuses(size(linkEllipsoid,2)) = radii(1);
%                 centres(size(linkEllipsoid,2), 1:3) = centrePoint;
%             otherwise
%                 disp("There is a problem. ")
%         end
% 
%         alpha(linkEllipsoid(:), 0.1);
%     end
% 
%     for i = 1:size(linkEllipsoid, 2)
%         points = vertices{1,1};
%         for j = 1:size(points, 1)
%             transforms(:,:,size(points, 1)) = objectTransform * transl(points(j,1), points(j,2), points(j,3));
%             dist = sqrt((transforms(1,4,j) - centres(i,1))^2 + (transforms(2,4,j) - centres(i,2))^2 + (transforms(3,4,j) - centres(i,3))^2);
%             if dist < radiuses(i)
%                 check = 1;
%                 return
%             end
%         end
%     end
%     
% end

% Rectangle prism point cloud for all objects to avoid








% IsCollision Function
% Inputs: a serial link robot model, a path of q values in a matrix, a set
% of faces, set of vertices, and set of face normals
% function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals)
%     result = false;
%     for qIndex = 1:size(qMatrix, 1)
%         % Get the transform of every joint (i.e. start and end of every link)
%         [~, linkTransforms] = robot.fkine(qMatrix(qIndex, :));
%         % Go through each link and also each triangle face
%         for linkIndex = 1 : (size(linkTransforms, 3) - 1)
%             
%             for faceIndex = 1:size(faces, 1)
%                 
%                 vertOnPlane = vertex(faces(faceIndex, 1)', :);
%                 
%                 [intersectP, check] = FindLinePlaneIntersection(faceNormals(faceIndex, :), ...
%                             vertOnPlane, linkTransforms(1:3, 4, linkIndex)', ...
%                             linkTransforms(1:3, 4, linkIndex + 1)'); 
%                 
%                 if check == 1 && CheckIsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%                     plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
%                     display('Intersection');
%                     result = true;
%                     return
%                 end
%             end    
%         end
%     end
% end
% 
% function result = CheckIsIntersectionPointInsideTriangle(intersectP,triangleVerts)
% 
%     u = triangleVerts(2,:) - triangleVerts(1,:);
%     v = triangleVerts(3,:) - triangleVerts(1,:);
% 
%     uu = dot(u,u);
%     uv = dot(u,v);
%     vv = dot(v,v);
% 
%     w = intersectP - triangleVerts(1,:);
%     wu = dot(w,u);
%     wv = dot(w,v);
% 
%     D = uv * uv - uu * vv;
% 
%     % Get and test parametric coords (s and t)
%     s = (uv * wv - vv * wu) / D;
%     if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
%         result = 0;
%         return;
%     end
% 
%     t = (uv * wu - uu * wv) / D;
%     if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
%         result = 0;
%         return;
%     end
% 
%     result = 1;                      % intersectP is in Triangle
% end

%% LinePlaneIntersection
% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
function [intersectionPoint,check] = FindLinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

    intersectionPoint = [0 0 0];
    u = point2OnLine - point1OnLine;
    w = point1OnLine - pointOnPlane;
    D = dot(planeNormal,u);
    N = -dot(planeNormal,w);
    check = 0; %#ok<NASGU>
    if abs(D) < 10^-7        % The segment is parallel to plane
        if N == 0           % The segment lies in plane
            check = 2;
            return
        else
            check = 0;       %no intersection
            return
        end
    end

    %compute the intersection parameter
    sI = N / D;
    intersectionPoint = point1OnLine + sI.*u;

    if (sI < 0 || sI > 1)
        check= 3;          %The intersection point  lies outside the segment, so there is no intersection
    else
        check=1;
    end
end

%% New method random point generator
% 
% numPoints = 20;
% maxX = 0.5;             minX = -0.5 - 0.32;
% maxY = 0.32;            minY = -0.23;
% maxZ = 0.133 + 0.28;    minZ = 0.012;
% points = zeros(1, 3, numPoints);
% transls = zeros(4, 4, numPoints);
% 
% for i = 1:numPoints
%     
%     points(1, 1, i) = ((maxX - minX) * rand(1,1)) + minX;
%     points(1, 2, i) = ((maxY - minY) * rand(1,1)) + minY;
%     points(1, 3, i) = ((maxZ - minZ) * rand(1,1)) + minZ;
%     transls(:,:,i) = transl(points(1,1,i), points(1,2,i), points(1,3,i));
%     while (Dobot1.CheckInBounds(transls(:,:,i)) == 0)
%         points(1, 1, i) = ((maxX - minX) * rand(1,1)) + minX;
%         points(1, 2, i) = ((maxY - minY) * rand(1,1)) + minY;
%         points(1, 3, i) = ((maxZ - minZ) * rand(1,1)) + minZ;
%         transls(:,:,i) = transl(points(1,1,i), points(1,2,i), points(1,3,i));
%         disp("Regenerating point " + i)
%     end
%     hold on
%     plot3(points(1,1,i), points(1,2,i), points(1,3,i), 'r.');
% 
% end
% 
% initialQ = Dobot1.model.getpos;
% currentQ = initialQ;
% steps = 50;
% 
% for i = 1:numPoints
%     [newlinpos, updateQ] = Dobot1.GetLinRailPos(currentQ, transls(:,:,i));
%     if newlinpos ~= currentQ(1)
%         qPath = Dobot1.GetJointPathQQ(currentQ, updateQ, steps);
%         currentQ = updateQ;
%         for j = 1:steps
%             Dobot1.model.animate(qPath(j, :))
%         end
%     end
%     [finalQ5, finalQ6] = Dobot1.GetLocalPose(currentQ, transls(:,:,i));
%     qPath = Dobot1.GetJointPathQQ(currentQ, finalQ6, steps);
%     currentQ = finalQ6;
%     for j = 1:steps
%         Dobot1.model.animate(qPath(j, :))
%     end
% end

%% Checking get lin rail pos function and new method of inverse kinematics

% currentQ = q;
% steps = 50;
% 
% hold on
% point = transl(0.5, 0.05, 0.2);
% plot3(point(1,4), point(2,4), point(3,4), 'r.');
% [newlinpos, updateQ] = Dobot1.GetLinRailPos(currentQ, point);
% disp("New linear rail position: " + newlinpos)
% qPath = Dobot1.GetJointPathQQ(currentQ, updateQ, steps);
% currentQ = updateQ;
% for i = 1:steps
%     Dobot1.model.animate(qPath(i, :))
% end
% [finalQ5, finalQ6] = Dobot1.GetLocalPose(currentQ, point);
% qPath = Dobot1.GetJointPathQQ(currentQ, finalQ6, steps);
% currentQ = finalQ6;
% for i = 1:steps
%     Dobot1.model.animate(qPath(i, :))
% end
% pause()
% 
% point = transl(0.3, -0.05, 0.2);
% plot3(point(1,4), point(2,4), point(3,4), 'r.');
% [newlinpos, updateQ] = Dobot1.GetLinRailPos(currentQ, point);
% disp("New linear rail position: " + newlinpos)
% qPath = Dobot1.GetJointPathQQ(currentQ, updateQ, steps);
% currentQ = updateQ;
% for i = 1:steps
%     Dobot1.model.animate(qPath(i, :))
% end
% [finalQ5, finalQ6] = Dobot1.GetLocalPose(currentQ, point);
% qPath = Dobot1.GetJointPathQQ(currentQ, finalQ6, steps);
% currentQ = finalQ6;
% for i = 1:steps
%     Dobot1.model.animate(qPath(i, :))
% end
% pause()
% 
% point = transl(-0.5, 0.05, 0.2);
% plot3(point(1,4), point(2,4), point(3,4), 'r.');
% [newlinpos, updateQ] = Dobot1.GetLinRailPos(currentQ, point);
% disp("New linear rail position: " + newlinpos)
% qPath = Dobot1.GetJointPathQQ(currentQ, updateQ, steps);
% currentQ = updateQ;
% for i = 1:steps
%     Dobot1.model.animate(qPath(i, :))
% end
% [finalQ5, finalQ6] = Dobot1.GetLocalPose(currentQ, point);
% qPath = Dobot1.GetJointPathQQ(currentQ, finalQ6, steps);
% currentQ = finalQ6;
% for i = 1:steps
%     Dobot1.model.animate(qPath(i, :))
% end

%% Custom inverse kinematics

% joint1Length = 0.135;
% joint2Length = 0.147;
% syms q1 q2
% 
% desiredZ = 0.2;
% desiredNorm = 0.2;
% 
% zFunction = joint1Length*cos(q1) + joint2Length*cos(q1 + q2) == desiredZ;
% normFunction = joint1Length*sin(q1) + joint2Length*sin(q1 + q2) == desiredNorm;
% 
% solutions = solve([zFunction, normFunction], [q1, q2])

%% Test some random movements

% numPoints = 20;
% maxX = 0.5;
% minX = -0.5 - 0.32;
% maxY = 0.32;
% minY = 0.075;
% maxZ = 0.133 + 0.28;
% minZ = 0.012;
% points = zeros(1, 3, numPoints);
% transls = zeros(4, 4, numPoints);
% 
% for i = 1:numPoints
%     
%     points(1, 1, i) = ((maxX - minX) * rand(1,1)) + minX;
%     points(1, 2, i) = ((maxY - minY) * rand(1,1)) + minY;
%     points(1, 3, i) = ((maxZ - minZ) * rand(1,1)) + minZ;
%     transls(:,:,i) = transl(points(1,1,i), points(1,2,i), points(1,3,i));
%     while (Dobot1.CheckInBounds(transls(:,:,i)) == 0)
%         points(1, 1, i) = ((maxX - minX) * rand(1,1)) + minX;
%         points(1, 2, i) = ((maxY - minY) * rand(1,1)) + minY;
%         points(1, 3, i) = ((maxZ - minZ) * rand(1,1)) + minZ;
%         transls(:,:,i) = transl(points(1,1,i), points(1,2,i), points(1,3,i));
%         disp("Regenerating point " + i)
%     end
%     hold on
%     plot3(points(1,1,i), points(1,2,i), points(1,3,i), 'r.');
% 
% end

% points(1, 1, 1) = -0.3;
% points(1, 2, 1) = 0.25;
% points(1, 3, 1) = 0.175;
% points(1, 1, 2) = -0.4890;
% points(1, 2, 2) = 0.3159;
% points(1, 3, 2) = 0.1077;
% points(1, 1, 3) = -0.25;
% points(1, 2, 3) = 0.308;
% points(1, 3, 3) = 0.2;

% hold on
% plot3(points(1,1,1), points(1,2,1), points(1,3,1), 'r.');
% plot3(points(1,1,2), points(1,2,2), points(1,3,2), 'g.');
% plot3(points(1,1,3), points(1,2,3), points(1,3,3), 'b.');

% initialQ = Dobot1.model.getpos;
% currentQ = initialQ;
% steps = 50;
% 
% for i = 1:numPoints
% %     plot3(points(1,1,i), points(1,2,i), points(1,3,i), 'r.');
%     transls(:,:,i) = transl(points(1,1,i), points(1,2,i), points(1,3,i));
% %     qGuess = Dobot1.GetGuessPose(currentQ, transls(:,:,i));
% %     Dobot1.model.animate(qGuess)
% %     disp("Guess Pose")
% %     pause()
% %     qTest1 = Dobot1.GetRobotPose_Ikcon(currentQ, transls(:,:,i))
% %     qTest1 = Dobot1.GetRobotPose_Ikine(currentQ, transls(:,:,i))
%     qPath = Dobot1.GetJointPathQT(currentQ, transls(:,:,i), steps);
%     for j = 1:size(qPath, 1)
%         Dobot1.model.animate(qPath(j, :))
%         drawnow()
%     end
%     currentQ = qPath(end, :);
%     disp(currentQ)
%     pause()
% end

%% Test some movements

% numPoints = 10;
% xSpacing = 0.14;
% firstPoint = -0.625;
% point = zeros(4,4,numPoints);
% qGuess = zeros(1,6,numPoints);
% 
% for i = 1:numPoints
%     j = (i-1) * xSpacing + firstPoint;
%     point(:,:,i) = transl(j, 0.125, 0.125);
% end
% for i = 1:numPoints
%     j = (i-1) * xSpacing + firstPoint;
%     point(:,:,i+numPoints) = transl(j, 0.275, 0.225);
% end
% for i = 1:numPoints
%     j = (i-1) * xSpacing + firstPoint;
%     point(:,:,i+2*numPoints) = transl(j, -0.125, 0.2);
% end
% 
% hold on
% 
% for i = 1:3*numPoints
%     plot3(point(1,4,i), point(2,4,i), point(3,4,i), 'r.');
% end
% 
% steps = 50;
% qCurrent = q;
% 
% for i = 1:3*numPoints
%     qPath1 = Dobot1.GetJointPathQT_Ikcon(qCurrent, point(:,:,i), steps);
%     qPath2 = Dobot1.GetJointPathQT_Ikine(qCurrent, point(:,:,i), steps);
%     qGuess1 = Dobot1.GetGuessPose(qCurrent, point(:,:,i));
%     qFinal = Dobot1.GetRobotPose(qCurrent, point(:,:,i));
%     qGuess5 = Dobot1.model.ikine(point(:,:,i), qGuess1);
%     
%     Dobot1.model.animate(qGuess1)
%     disp("guess 1")
%     pause()
%     Dobot1.model.animate(qGuess5)
%     disp("guess 5")
%     pause()
%     Dobot1.model.animate(qFinal)
%     disp("actual")
%     pause()
%     disp("path 1")
%     for j = 1:steps
%         Dobot1.model.animate(qPath1(j,:));
%         drawnow()
%     end
%     disp("path 2")
%     for j = 1:steps
%         Dobot1.model.animate(qPath2(j,:));
%         drawnow()
%     end
%     qCurrent = qPath1(end, :);
%     pause()
% end

%% Check if guess poses work

% numPoints = 7;
% xSpacing = 0.2;
% firstPoint = -0.6;
% point = zeros(4,4,numPoints);
% qGuess = zeros(1,6,numPoints);
% 
% for i = 1:numPoints
%     j = (i-1) * xSpacing + firstPoint;
%     point(:,:,i) = transl(j, -0.1, 0.2);
% end
% 
% hold on
% 
% for i = 1:numPoints
%     plot3(point(1,4,i), point(2,4,i), point(3,4,i), 'r.');
% end
% 
% for i = 1:numPoints
%     qGuess(:, :, i) = Dobot1.GetGuessPose(q, point(:,:,i));
%     Dobot1.model.animate(qGuess(:,:,i));
%     drawnow();
%     pause()
% end

%% Boundary plotting that actually works

% 
% spacing = 1.65;
% numSide = 45;
% verticalOffset = 0.05;
% verticalSpacing = 0.025;
% numPlanes = 18;
% 
% P = zeros(3, numSide^2, numPlanes); 
% T = zeros(4, 4, size(P, 2), numPlanes);
% b = zeros(1, size(P, 2), numPlanes);
% Pb = zeros(3, size(P, 2), numPlanes);
% 
% for i = 1:numPlanes
%     P(:,:,i) = mkgrid(numSide, spacing, 'T', transl(0,0,(verticalOffset + i*verticalSpacing)));
%     for j = 1:size(P, 2)
%         T(:, :, j, i) = transl(P(1, j, i), P(2, j, i), P(3, j, i));
%         b(:, j, i) = Dobot1.CheckInBounds(T(:, :, j, i));
%     end
%     pBound = P(:, logical(b(:, :, i)), i);
%     for j = 1:size(pBound, 2)
%         Pb(:,j,i) = pBound(:, j);
%     end
%     hold on
%     for j = 1:size(Pb, 2)
%         if Pb(:,j,i) ~= [0; 0; 0]
%             plot3(Pb(1,j,i), Pb(2,j,i), Pb(3,j,i), 'b.');
%         end
%     end
% end

%%


%% a bunch of mess

% % P1 = mkgrid(numSide, spacing, 'T', transl(0,0,0.05));
% P2 = mkgrid(numSide, spacing, 'T', transl(0,0,0.1));
% P3 = mkgrid(numSide, spacing, 'T', transl(0,0,0.15));
% P4 = mkgrid(numSide, spacing, 'T', transl(0,0,0.2));
% % P5 = mkgrid(numSide, spacing, 'T', transl(0,0,0.25));
% % P6 = mkgrid(numSide, spacing, 'T', transl(0,0,0.3));
% % P7 = mkgrid(numSide, spacing, 'T', transl(0,0,0.35));
% % plot_sphere(P1, 0.01, 'red');
% % plot_sphere(P2, 0.01, 'red');
% % plot_sphere(P3, 0.01, 'red');
% 
% % T1 = zeros(4,4,size(P1, 2));
% T2 = zeros(4,4,size(P2, 2));
% T3 = zeros(4,4,size(P3, 2));
% T4 = zeros(4,4,size(P4, 2));
% % T5 = zeros(4,4,size(P5, 2));
% % T6 = zeros(4,4,size(P6, 2));
% % T7 = zeros(4,4,size(P7, 2));
% 
% for i = 1:size(P2, 2)
% %     T1(:,:,i) = transl(P1(1, i), P1(2, i), P1(3, i));
%     T2(:,:,i) = transl(P2(1, i), P2(2, i), P2(3, i));
%     T3(:,:,i) = transl(P3(1, i), P3(2, i), P3(3, i));
%     T4(:,:,i) = transl(P4(1, i), P4(2, i), P4(3, i));
% %     T5(:,:,i) = transl(P5(1, i), P5(2, i), P5(3, i));
% %     T6(:,:,i) = transl(P6(1, i), P6(2, i), P6(3, i));
% %     T7(:,:,i) = transl(P7(1, i), P7(2, i), P7(3, i));
% end
% 
% % b1 = zeros(1,size(P1, 2));
% b2 = zeros(1,size(P2, 2));
% b3 = zeros(1,size(P3, 2));
% b4 = zeros(1,size(P4, 2));
% % b5 = zeros(1,size(P5, 2));
% % b6 = zeros(1,size(P6, 2));
% % b7 = zeros(1,size(P7, 2));
% 
% % T1b = zeros(4,4,size(T1, 2));
% % T2b = zeros(4,4,size(T2, 2));
% % T3b = zeros(4,4,size(T3, 2));
% % T4b = zeros(4,4,size(T4, 2));
% % T5b = zeros(4,4,size(T5, 2));
% % T6b = zeros(4,4,size(T6, 2));
% % T7b = zeros(4,4,size(T7, 2));
% 
% for i = 1:size(T2, 3)
% %     T1b(:,:,i) = inv(link1T) * T1(:,:,i);
% %     T2b(:,:,i) = inv(link1T) * T2(:,:,i);
% %     T3b(:,:,i) = inv(link1T) * T3(:,:,i);
% %     T4b(:,:,i) = inv(link1T) * T4(:,:,i);
% %     T5b(:,:,i) = inv(link1T) * T5(:,:,i);
% %     T6b(:,:,i) = inv(link1T) * T6(:,:,i);
% %     T7b(:,:,i) = inv(link1T) * T7(:,:,i);
%     
% %     b1(i) = Dobot1.CheckInBounds(T1(:,:,i));
%     b2(i) = Dobot1.CheckInBounds(T2(:,:,i));
%     b3(i) = Dobot1.CheckInBounds(T3(:,:,i));
%     b4(i) = Dobot1.CheckInBounds(T4(:,:,i));
% %     b5(i) = Dobot1.CheckInBounds(T5(:,:,i));
% %     b6(i) = Dobot1.CheckInBounds(T6(:,:,i));
% %     b7(i) = Dobot1.CheckInBounds(T7(:,:,i));
%     
% %     b1b(i) = Dobot1.CheckInRange(T1b(:,:,i));
% %     b2b(i) = Dobot1.CheckInRange(T2b(:,:,i));
% %     b3b(i) = Dobot1.CheckInRange(T3b(:,:,i));
% %     b4b(i) = Dobot1.CheckInRange(T4b(:,:,i));
% %     b5b(i) = Dobot1.CheckInRange(T5b(:,:,i));
% %     b6b(i) = Dobot1.CheckInRange(T6b(:,:,i));
% %     b7b(i) = Dobot1.CheckInRange(T7b(:,:,i));
% end
% 
% % P1b = P1(:,logical(b1));
% P2b = P2(:,logical(b2));
% P3b = P3(:,logical(b3));
% P4b = P4(:,logical(b4));
% % P5b = P5(:,logical(b5));
% % P6b = P6(:,logical(b6));
% % P7b = P7(:,logical(b7));
% 
% % P1c = P1(:,logical(b1b));
% % P2c = P2(:,logical(b2b));
% % P3c = P3(:,logical(b3b));
% % P4c = P4(:,logical(b4b));
% % P5c = P5(:,logical(b5b));
% % P6c = P6(:,logical(b6b));
% % P7c = P7(:,logical(b7b));
% 
% % plot_sphere(P1b, 0.003, 'green');
% plot_sphere(P2b, 0.003, 'green');
% plot_sphere(P3b, 0.003, 'green');
% plot_sphere(P4b, 0.003, 'green');
% % plot_sphere(P5b, 0.003, 'green');
% % plot_sphere(P6b, 0.003, 'green');
% % plot_sphere(P7b, 0.003, 'green');

% plot_sphere(P1c, 0.003, 'green');
% plot_sphere(P2c, 0.003, 'green');
% plot_sphere(P3c, 0.003, 'green');
% plot_sphere(P4c, 0.003, 'green');
% plot_sphere(P5c, 0.003, 'green');
% plot_sphere(P6c, 0.003, 'green');
% plot_sphere(P7c, 0.003, 'green');

%% Single plane boundary plot

% P1 = mkgrid(35, 0.7, 'T', transl(-0.25,0,0.25)*rpy2tr(0,pi/2,0));
% plot_sphere(P1, 0.0025, 'red');
% T1 = zeros(4,4,size(P1, 2));
% for i = 1:size(P1, 2)
%     T1(:,:,i) = transl(P1(1, i), P1(2, i), P1(3, i));
% end
% b1 = zeros(1,size(P1, 2));
% T1b = zeros(4,4,size(T1, 2));
% for i = 1:size(T1, 3)
%     T1b(:,:,i) = inv(link1T) * T1(:,:,i);
%     b1(i) = Dobot1.CheckInBounds(T1(:,:,i));
%     b1b(i) = Dobot1.CheckInRange(T1b(:,:,i));
% end
% P1b = P1(:,logical(b1));
% P1c = P1(:,logical(b1b));
% plot_sphere(P1b, 0.006, 'blue');
% plot_sphere(P1c, 0.005, 'green');

%% Boundary circle plot

% C1 = [-0.25,Dobot1.rangeCircle1.normalDistOrigin,Dobot1.rangeCircle1.zDistOrigin + Dobot1.model.base(3,4)] ;   % center of circle 
% C2 = [-0.25,Dobot1.rangeCircle2.normalDistOrigin,Dobot1.rangeCircle2.zDistOrigin + Dobot1.model.base(3,4)] ;   % center of circle 
% C3 = [-0.25,Dobot1.rangeCircle3.normalDistOrigin,Dobot1.rangeCircle3.zDistOrigin + Dobot1.model.base(3,4)] ;   % center of circle 
% C4 = [-0.25,Dobot1.rangeCircle4.normalDistOrigin,Dobot1.rangeCircle4.zDistOrigin + Dobot1.model.base(3,4)] ;   % center of circle 
% 
% constZOffset = 0.127;
% constXOffset = 0;
% 
% C1b = [0.25, 0.19973 + constXOffset, 0.04509 + constZOffset];
% C2b = [0.25, 0.05919 + constXOffset, 0.00000 + constZOffset];
% C3b = [0.25, 0.04549 + constXOffset, 0.13204 + constZOffset];
% C4b = [0.25, 0.07503 + constXOffset, -0.14462 + constZOffset];
% 
% R1 = Dobot1.rangeCircle1.radius;    % Radius of circle 
% R2 = Dobot1.rangeCircle2.radius;    % Radius of circle 
% R3 = Dobot1.rangeCircle3.radius;    % Radius of circle 
% R4 = Dobot1.rangeCircle4.radius;    % Radius of circle 
% 
% R1b = (0.26845/2);
% R2b = (0.55487/2);
% R3b = (0.28769/2);
% R4b = (0.26803/2);
% 
% theta = 0:0.01:2*pi;
% thetaB = 0:0.01:pi/2;
% 
% z1 = C1(3)+ R1*cos(thetaB);
% y1 = C1(2)+ R1*sin(thetaB);
% x1 = C1(1)+ zeros(size(z1));
% 
% z2 = C2(3)+ R2*cos(theta);
% y2 = C2(2)+ R2*sin(theta);
% x2 = C2(1)+ zeros(size(z2));
% 
% z3 = C3(3)+ R3*cos(theta);
% y3 = C3(2)+ R3*sin(theta);
% x3 = C3(1)+ zeros(size(z3));
% 
% z4 = C4(3)+ R4*cos(theta);
% y4 = C4(2)+ R4*sin(theta);
% x4 = C4(1)+ zeros(size(z4));
% 
% z1b = C1b(3)+ R1b*cos(theta);
% y1b = C1b(2)+ R1b*sin(theta);
% x1b = C1b(1)+ zeros(size(z1b));
% 
% z2b = C2b(3)+ R2b*cos(theta);
% y2b = C2b(2)+ R2b*sin(theta);
% x2b = C2b(1)+ zeros(size(z2b));
% 
% 
% z3b = C3b(3)+ R3b*cos(theta);
% y3b = C3b(2)+ R3b*sin(theta);
% x3b = C3b(1)+ zeros(size(z3b));
% 
% z4b = C4b(3)+ R4b*cos(theta);
% y4b = C4b(2)+ R4b*sin(theta);
% x4b = C4b(1)+ zeros(size(z4b));
% 
% hold on
% plot3(x1,y1,z1, 'red')
% plot3(x2,y2,z2, 'blue')
% plot3(x3,y3,z3, 'green')
% plot3(x4,y4,z4, 'k')
% 
% hold on
% plot3(x1b,y1b,z1b, 'red')
% plot3(x2b,y2b,z2b, 'blue')
% plot3(x3b,y3b,z3b, 'green')
% plot3(x4b,y4b,z4b, 'k')
% 
