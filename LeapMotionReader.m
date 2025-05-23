classdef LeapMotionReader < matlab.System
    % LeapMotionReader – calls Leap API each tick, returns Boom/Arm/Bucket angles

    properties (Access = private)
        frameId = -1;
        t0      = -1;
        boomA   = 0;
        armA    = 0;
        bucA    = 0;
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Called once at simulation start
            obj.frameId = -1;
            obj.t0      = -1;
            obj.boomA   = 0;
            obj.armA    = 0;
            obj.bucA    = 0;
            coder.extrinsic('matleap_frame','extract_index_joint_positions');
        end

        function [Boom, Arm, Buc] = stepImpl(obj)
            % Called each time step, only at runtime
            f = matleap_frame;   % extrinsic call

            if f.id ~= obj.frameId && ~isempty(f.hands)
                obj.frameId = f.id;
                if obj.t0 < 0
                    obj.t0 = f.timestamp;
                end

                out = extract_index_joint_positions(f);

                % Boom: meta→mcp
                h = out.mcp_Z - out.meta_Z;
                v = out.mcp_Y - out.meta_Y;
                obj.boomA = acos(h/hypot(h,v)) - 2;

                % Arm: mcp→pip
                h = out.pip_Z - out.mcp_Z;
                v = out.pip_Y - out.mcp_Y;
                obj.armA = acos(h/hypot(h,v)) + 1.37;

                % Bucket: pip→tip
                h = out.tip_Z - out.pip_Z;
                v = out.tip_Y - out.pip_Y;
                obj.bucA = (acos(h/hypot(h,v)) + 2)/1.8 + 0.2;
            end

            Boom = obj.boomA;
            Arm  = obj.armA;
            Buc  = obj.bucA;
        end

        function resetImpl(obj)
            setupImpl(obj);
        end

        function tf = isInputDirectFeedthroughImpl(~,~)
            tf = false;
        end

        function flag = supportsCodeGenerationImpl(~)
            flag = false;  % interpreted only
        end
    end

methods (Static, Access = protected)
    function num = getNumOutputsImpl(~)
        num = 3;
    end

    function [s1,s2,s3] = getOutputSizeImpl(~)
        s1 = [1 1];  s2 = [1 1];  s3 = [1 1];
    end

    function [t1,t2,t3] = getOutputDataTypeImpl(~)
        t1 = "double";  t2 = "double";  t3 = "double";
    end

    function [c1,c2,c3] = isOutputComplexImpl(~)
        c1 = false;  c2 = false;  c3 = false;
    end

    function [f1,f2,f3] = isOutputFixedSizeImpl(~)
        f1 = true;  f2 = true;  f3 = true;
    end

    function sts = getSampleTimeImpl(~)
        % Discrete sample time = 0.05 s (20 Hz)
        sts = matlab.system.SampleTimeSpecification( ...
                  'Type','Discrete', ...
                  'SampleTime',0.05);
    end
end


end


function out = extract_index_joint_positions(f)
    out.timestamp = f.timestamp;
    fields = {'meta','mcp','pip','tip'};
    for k = 1:4
        nm = fields{k};
        out.([nm '_X']) = NaN;
        out.([nm '_Y']) = NaN;
        out.([nm '_Z']) = NaN;
    end

    try
        finger = f.hands(1).digits(2);
        for idx = 1:4
            bone = finger.bones(idx);
            if idx < 4
                joint = bone.prev_joint;
            else
                joint = bone.next_joint;
            end
            nm = fields{idx};
            out.([nm '_X']) = joint(1);
            out.([nm '_Y']) = joint(2);
            out.([nm '_Z']) = joint(3);
        end
    catch
        % leave NaNs
    end
end
