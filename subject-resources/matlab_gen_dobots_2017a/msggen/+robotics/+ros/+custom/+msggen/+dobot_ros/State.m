classdef State < robotics.ros.Message
    %State MATLAB implementation of dobot_ros/State
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2017 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'dobot_ros/State' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'b46d58af67aa78b04d07603e797ae0a8' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsPointClass = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Point') % Dispatch to MATLAB class for message type geometry_msgs/Point
        StdMsgsHeaderClass = robotics.ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        EndpointPos
        HeadAngle
        BaseAngle
        RearArmAngle
        ForeArmAngle
        ServoAngle
        Pump
        GripperAngle
    end
    
    properties (Access = protected)
        Cache = struct('Header', [], 'EndpointPos', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'BaseAngle', 'EndpointPos', 'ForeArmAngle', 'GripperAngle', 'HeadAngle', 'Header', 'Pump', 'RearArmAngle', 'ServoAngle'} % List of non-constant message properties
        ROSPropertyList = {'base_angle', 'endpoint_pos', 'fore_arm_angle', 'gripper_angle', 'head_angle', 'header', 'pump', 'rear_arm_angle', 'servo_angle'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = State(msg)
            %State Construct the message object State
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
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'State', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function endpointpos = get.EndpointPos(obj)
            %get.EndpointPos Get the value for property EndpointPos
            if isempty(obj.Cache.EndpointPos)
                obj.Cache.EndpointPos = feval(obj.GeometryMsgsPointClass, obj.JavaMessage.getEndpointPos);
            end
            endpointpos = obj.Cache.EndpointPos;
        end
        
        function set.EndpointPos(obj, endpointpos)
            %set.EndpointPos Set the value for property EndpointPos
            validateattributes(endpointpos, {obj.GeometryMsgsPointClass}, {'nonempty', 'scalar'}, 'State', 'EndpointPos');
            
            obj.JavaMessage.setEndpointPos(endpointpos.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.EndpointPos)
                obj.Cache.EndpointPos.setJavaObject(endpointpos.getJavaObject);
            end
        end
        
        function headangle = get.HeadAngle(obj)
            %get.HeadAngle Get the value for property HeadAngle
            headangle = double(obj.JavaMessage.getHeadAngle);
        end
        
        function set.HeadAngle(obj, headangle)
            %set.HeadAngle Set the value for property HeadAngle
            validateattributes(headangle, {'numeric'}, {'nonempty', 'scalar'}, 'State', 'HeadAngle');
            
            obj.JavaMessage.setHeadAngle(headangle);
        end
        
        function baseangle = get.BaseAngle(obj)
            %get.BaseAngle Get the value for property BaseAngle
            baseangle = double(obj.JavaMessage.getBaseAngle);
        end
        
        function set.BaseAngle(obj, baseangle)
            %set.BaseAngle Set the value for property BaseAngle
            validateattributes(baseangle, {'numeric'}, {'nonempty', 'scalar'}, 'State', 'BaseAngle');
            
            obj.JavaMessage.setBaseAngle(baseangle);
        end
        
        function reararmangle = get.RearArmAngle(obj)
            %get.RearArmAngle Get the value for property RearArmAngle
            reararmangle = double(obj.JavaMessage.getRearArmAngle);
        end
        
        function set.RearArmAngle(obj, reararmangle)
            %set.RearArmAngle Set the value for property RearArmAngle
            validateattributes(reararmangle, {'numeric'}, {'nonempty', 'scalar'}, 'State', 'RearArmAngle');
            
            obj.JavaMessage.setRearArmAngle(reararmangle);
        end
        
        function forearmangle = get.ForeArmAngle(obj)
            %get.ForeArmAngle Get the value for property ForeArmAngle
            forearmangle = double(obj.JavaMessage.getForeArmAngle);
        end
        
        function set.ForeArmAngle(obj, forearmangle)
            %set.ForeArmAngle Set the value for property ForeArmAngle
            validateattributes(forearmangle, {'numeric'}, {'nonempty', 'scalar'}, 'State', 'ForeArmAngle');
            
            obj.JavaMessage.setForeArmAngle(forearmangle);
        end
        
        function servoangle = get.ServoAngle(obj)
            %get.ServoAngle Get the value for property ServoAngle
            servoangle = double(obj.JavaMessage.getServoAngle);
        end
        
        function set.ServoAngle(obj, servoangle)
            %set.ServoAngle Set the value for property ServoAngle
            validateattributes(servoangle, {'numeric'}, {'nonempty', 'scalar'}, 'State', 'ServoAngle');
            
            obj.JavaMessage.setServoAngle(servoangle);
        end
        
        function pump = get.Pump(obj)
            %get.Pump Get the value for property Pump
            pump = logical(obj.JavaMessage.getPump);
        end
        
        function set.Pump(obj, pump)
            %set.Pump Set the value for property Pump
            validateattributes(pump, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'State', 'Pump');
            
            obj.JavaMessage.setPump(pump);
        end
        
        function gripperangle = get.GripperAngle(obj)
            %get.GripperAngle Get the value for property GripperAngle
            gripperangle = double(obj.JavaMessage.getGripperAngle);
        end
        
        function set.GripperAngle(obj, gripperangle)
            %set.GripperAngle Set the value for property GripperAngle
            validateattributes(gripperangle, {'numeric'}, {'nonempty', 'scalar'}, 'State', 'GripperAngle');
            
            obj.JavaMessage.setGripperAngle(gripperangle);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
            obj.Cache.EndpointPos = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.HeadAngle = obj.HeadAngle;
            cpObj.BaseAngle = obj.BaseAngle;
            cpObj.RearArmAngle = obj.RearArmAngle;
            cpObj.ForeArmAngle = obj.ForeArmAngle;
            cpObj.ServoAngle = obj.ServoAngle;
            cpObj.Pump = obj.Pump;
            cpObj.GripperAngle = obj.GripperAngle;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
            cpObj.EndpointPos = copy(obj.EndpointPos);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.HeadAngle = strObj.HeadAngle;
            obj.BaseAngle = strObj.BaseAngle;
            obj.RearArmAngle = strObj.RearArmAngle;
            obj.ForeArmAngle = strObj.ForeArmAngle;
            obj.ServoAngle = strObj.ServoAngle;
            obj.Pump = strObj.Pump;
            obj.GripperAngle = strObj.GripperAngle;
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
            obj.EndpointPos = feval([obj.GeometryMsgsPointClass '.loadobj'], strObj.EndpointPos);
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
            
            strObj.HeadAngle = obj.HeadAngle;
            strObj.BaseAngle = obj.BaseAngle;
            strObj.RearArmAngle = obj.RearArmAngle;
            strObj.ForeArmAngle = obj.ForeArmAngle;
            strObj.ServoAngle = obj.ServoAngle;
            strObj.Pump = obj.Pump;
            strObj.GripperAngle = obj.GripperAngle;
            strObj.Header = saveobj(obj.Header);
            strObj.EndpointPos = saveobj(obj.EndpointPos);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.dobot_ros.State.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.dobot_ros.State;
            obj.reload(strObj);
        end
    end
end