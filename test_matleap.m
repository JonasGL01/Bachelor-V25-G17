% @file test_matleap.m
% @brief test matleap functionality
% @author Jeff Perry <jeffsp@gmail.com>
% @version 1.0
% @date 2013-09-12

function data = test_matleap
    [version] = matleap_version;
    fprintf('matleap version %d.%d\n', version(1), version(2));

    pause(1);  % Let hardware wake up

    frame_id = -1;
    frames = 0;
    data = [];
    t0 = -1;  % Initial timestamp
    tic;

    while toc < 10
        f = matleap_frame;

        if f.id ~= frame_id && ~isempty(f.hands)
            frame_id = f.id;
            out = extract_index_joint_positions(f);

            % Initialize t0 and compute relative time
            if t0 < 0
                t0 = f.timestamp;
            end
            out.time = (f.timestamp - t0) * 1e-6;  % Convert µs to seconds

            data = [data; out];
            frames = frames + 1;
        end
    end

    s = toc;
    fprintf('%d frames\n', frames);
    fprintf('%f seconds\n', s);
    fprintf('%f fps\n', frames / s);
end

function out = extract_index_joint_positions(f)
    out.timestamp = f.timestamp;

    % Initialize with NaNs
    fields = {'meta', 'mcp', 'pip', 'tip'};
    for i = 1:length(fields)
        out.([fields{i} '_X']) = NaN;
        out.([fields{i} '_Y']) = NaN;
        out.([fields{i} '_Z']) = NaN;
    end

    try
        finger = f.hands(1).digits(2);  % Index finger

        % Metacarpal base
        out.meta_X = finger.bones(1).prev_joint(1);
        out.meta_Y = finger.bones(1).prev_joint(2);
        out.meta_Z = finger.bones(1).prev_joint(3);

        % MCP joint
        out.mcp_X = finger.bones(2).prev_joint(1);
        out.mcp_Y = finger.bones(2).prev_joint(2);
        out.mcp_Z = finger.bones(2).prev_joint(3);

        % PIP joint
        out.pip_X = finger.bones(3).prev_joint(1);
        out.pip_Y = finger.bones(3).prev_joint(2);
        out.pip_Z = finger.bones(3).prev_joint(3);

        % Fingertip
        out.tip_X = finger.bones(4).next_joint(1);
        out.tip_Y = finger.bones(4).next_joint(2);
        out.tip_Z = finger.bones(4).next_joint(3);

    catch
        % Leave NaNs on failure
    end
end





