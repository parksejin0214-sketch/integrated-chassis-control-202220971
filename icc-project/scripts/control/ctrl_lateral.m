function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL More path-friendly AFS + mild ESC beta limiter
%
% 목적:
%   - A3/A4/A7 만점 유지
%   - A1/D1에서 path-following driver를 덜 방해하도록 AFS 보조 조향을 줄임
%   - A1/D1 LTR_max가 0.6을 넘지 않는 범위에서 lateralDevMax를 낮추는 실험
%
% 기존 최고 근처 결과:
%   A1 lateralDevMax ≈ 2.1230
%   D1 lateralDevMax ≈ 2.1230
%   총점 ≈ 60.78 / 70

    %% ---------- state initialization ----------
    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end

    if ~isfield(ctrlState, 'prevError')
        ctrlState.prevError = 0;
    end

    if ~isfield(ctrlState, 'derivFilt')
        ctrlState.derivFilt = 0;
    end

    %% ---------- basic signals ----------
    err = yawRateRef - yawRate;
    absBeta = abs(slipAngle);

    %% ---------- base gains ----------
    Kp_base = CTRL.LAT.Kp;
    Ki_base = CTRL.LAT.Ki;
    Kd_base = CTRL.LAT.Kd;
    intMax  = CTRL.LAT.intMax;

    %% ---------- speed scheduling ----------
    v_ref = 20.0;
    speed_scale = max(0.30, min(2.00, v_ref / max(vx, 5.0)));

    Kp = Kp_base * speed_scale * 2.5;
    Ki = Ki_base * speed_scale * 0.3;
    Kd = Kd_base * speed_scale * 2.0;

    %% ---------- PID yaw-rate tracking ----------
    ctrlState.intError = ctrlState.intError + err * dt;
    ctrlState.intError = max(-intMax, min(intMax, ctrlState.intError));

    derivRaw = (err - ctrlState.prevError) / max(dt, 1e-6);

    alpha = dt / (0.02 + dt);
    ctrlState.derivFilt = (1 - alpha) * ctrlState.derivFilt + alpha * derivRaw;

    ctrlState.prevError = err;

    steerPID = Kp * err + Ki * ctrlState.intError + Kd * ctrlState.derivFilt;

    %% ---------- path-friendly AFS scheduling ----------
    % beta가 작을 때:
    %   A3/A4 성능 유지를 위해 AFS yaw-rate tracking을 거의 유지
    %
    % beta가 커질 때:
    %   A1/D1 DLC에서 Stanley driver의 path-following을 방해하지 않도록
    %   AFS 보조 조향을 줄임

    betaSoft = deg2rad(0.8);
    betaHard = deg2rad(2.2);

    if absBeta <= betaSoft
        steerScale = 1.00;
    elseif absBeta >= betaHard
        steerScale = 0.12;
    else
        q = (absBeta - betaSoft) / (betaHard - betaSoft);
        steerScale = 1.00 - 0.88 * q;
    end

    steerAdd = steerScale * steerPID;

    %% ---------- AFS saturation ----------
    maxAdd = 0.38 * LIM.MAX_STEER_ANGLE;
    steerAdd = max(-maxAdd, min(maxAdd, steerAdd));

    %% ---------- ESC beta limiter ----------
    % A1 LTR_max가 0.6에 가까우므로 ESC를 완전히 줄이면 위험함.
    % 따라서 beta가 2도 이상 커질 때만 yawMoment를 걸어 stability를 유지.

    beta_on   = deg2rad(2.0);
    beta_full = deg2rad(3.2);

    Kbeta = 54000;

    if absBeta > beta_on
        excess = absBeta - beta_on;
        speed_factor = min(2.0, max(0.5, vx / 15.0));

        if absBeta < beta_full
            blend = 0.42;
        else
            blend = 0.95;
        end

        yawMoment = -Kbeta * sign(slipAngle) * excess * speed_factor * blend;
    else
        yawMoment = 0;
    end

    %% ---------- yaw moment saturation ----------
    M_max = 12000;
    yawMoment = max(-M_max, min(M_max, yawMoment));

    %% ---------- output ----------
    deltaAdd.steerAngle = steerAdd;
    deltaAdd.yawMoment  = yawMoment;

end
