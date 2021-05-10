classdef SetPosCartRequest < robotics.ros.Message
    %SetPosCartRequest MATLAB implementation of dobot_ros/SetPosCartRequest
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2017 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'dobot_ros/SetPosCartRequest' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'adc15b7ec843fd0b7cee4d793a701b71' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsPointClass = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Point') % Dispatch to MATLAB class for message type geometry_msgs/Point
    end
    
    properties (Dependent)
        Pos
    end
    
    properties (Access = protected)
        Cache = struct('Pos', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Pos'} % List of non-constant message properties
        ROSPropertyList = {'pos'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = SetPosCartRequest(msg)
            %SetPosCartRequest Construct the message object SetPosCartRequest
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function pos = get.Pos(obj)
            %get.Pos Get the value for property Pos
            if isempty(obj.Cache.Pos)
                obj.Cache.Pos = feval(obj.GeometryMsgsPointClass, obj.JavaMessage.getPos);
            end
            pos = obj.Cache.Pos;
        end
        
        function set.Pos(obj, pos)
            %set.Pos Set the value for property Pos
            validateattributes(pos, {obj.GeometryMsgsPointClass}, {'nonempty', 'scalar'}, 'SetPosCartRequest', 'Pos');
            
            obj.JavaMessage.setPos(pos.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Pos)
                obj.Cache.Pos.setJavaObject(pos.getJavaObject);
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Pos = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Recursively copy compound properties
            cpObj.Pos = copy(obj.Pos);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Pos = feval([obj.GeometryMsgsPointClass '.loadobj'], strObj.Pos);
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Pos = saveobj(obj.Pos);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.dobot_ros.SetPosCartRequest.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.dobot_ros.SetPosCartRequest;
            obj.reload(strObj);
        end
    end
end