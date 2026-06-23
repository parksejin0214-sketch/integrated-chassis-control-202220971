function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR actuator allocation for AFS, DYC/ESC, ABS and CDC
%
%   Output brakeTorque is additive to scenario.brakeCmd in run_icc_scenario.m.
%   Therefore negative brakeTorque is intentionally allowed here for ABS brake
%   release; run_icc_scenario.m finally clamps total brake torque to [0, max].

    %% vehicle params
    if isfield(VEH, 'track_f'); t_f = VEH.track_f; else; t_f = 1.55; end
    if isfield(VEH, 'track_r'); t_r = VEH.track_r; else; t_r = 1.55; end
    if isfield(VEH, 'rw');      rw  = VEH.rw;      else; rw  = 0.31; end
    if isfield(VEH, 'mass');    m   = VEH.mass;    else; m   = 1500; end
    g = 9.81;

    %% steering pass-through
    steer = latCmd.steerAngle;
    steer = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer));

    %% longitudinal additive brake command
    T_lon = zeros(4,1);
    if isfield(lonCmd, 'brakeTorqueAdd')
        T_lon = lonCmd.brakeTorqueAdd(:);
        if numel(T_lon) ~= 4; T_lon = zeros(4,1); end
    elseif isfield(lonCmd, 'Fx_total') && lonCmd.Fx_total < 0
        Fx_brake_total = -lonCmd.Fx_total * getfield_default_local(lonCmd, 'brakeRatio', 1.0);
        T_lon = [0.30; 0.30; 0.20; 0.20] * Fx_brake_total * rw;
    end

    %% yaw moment -> differential brake allocation
    Mz = latCmd.yawMoment;
    ratio_f = 0.65;
    dT_front = ratio_f       * Mz / max(t_f, 1e-3) * rw;
    dT_rear  = (1-ratio_f)  * Mz / max(t_r, 1e-3) * rw;

    % Sign convention inherited from original project mapping.
    T_esc = [-dT_front; +dT_front; -dT_rear; +dT_rear];

    %% sum additive torques
    T = T_lon + T_esc;

    %% simple friction/authority protection for positive brake additions
    Fz_static = m*g/4;
    T_pos_lim = 0.90 * Fz_static * rw;
    for i = 1:4
        if T(i) > T_pos_lim
            T(i) = T_pos_lim;
        end
    end

    % Allow negative ABS release but bound it; final scenario code clamps total.
    T = max(-0.98*LIM.MAX_BRAKE_TRQ, min(LIM.MAX_BRAKE_TRQ, T));

    %% damping pass-through
    damping = verCmd;
    if numel(damping) ~= 4
        damping = 1500 * ones(4,1);
    else
        damping = damping(:);
    end

    actuatorCmd.steerAngle   = steer;
    actuatorCmd.brakeTorque  = T;
    actuatorCmd.dampingCoeff = damping;
end

function v = getfield_default_local(s, name, defaultVal)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = defaultVal;
    end
end