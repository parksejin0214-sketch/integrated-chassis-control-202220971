function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL ABS slip-tracking controller
%
% 목적:
%   - A3/A4/A1/D1 같은 비제동 시나리오에서는 longitudinal actuation을 만들지 않음.
%   - B1 급제동 중에는 wheel slip ratio를 약 -0.12 근처로 유지.
%   - slip이 너무 크면 brake release, slip이 너무 작으면 small brake add.
%
% 핵심:
%   run_icc_scenario.m에서 최종 제동 토크는 대략
%       brake_total = brk_scenario + brakeESC
%   형태이므로, brakeTorqueAdd는 scenario brake에 더해지는 additive torque임.

    %% ---------- state initialization ----------
    if ~isfield(ctrlState, 'prevAbsTorque')
        ctrlState.prevAbsTorque = zeros(4,1);
    end

    if ~isfield(ctrlState, 'wheelSlip')
        ctrlState.wheelSlip = zeros(4,1);
    end

    %% ---------- read wheel slip ----------
    kappa = ctrlState.wheelSlip(:);

    if numel(kappa) ~= 4
        kappa = zeros(4,1);
    end

    %% ---------- default: no speed PI ----------
    Fx_total = 0;
    brakeRatio = 0;

    %% ---------- braking detection ----------
    brakingLikely = (vx > 2.0) && (ax < -1.0) && any(kappa < -0.04);

    %% ---------- ABS slip target ----------
    kappaRef = -0.12;

    releaseBand = 0.015;
    addBand     = 0.030;

    Krelease = 7200;
    Kadd     = 1800;

    absTorqueAdd = zeros(4,1);

    if brakingLikely
        for i = 1:4
            % e > 0 : 실제 slip이 목표보다 덜 음수, 즉 제동 여유 있음
            % e < 0 : 실제 slip이 목표보다 더 음수, 즉 lock 경향
            e = kappa(i) - kappaRef;

            if e < -releaseBand
                % kappa가 -0.12보다 더 음수이면 wheel lock 경향
                % 음수 torque로 scenario brake를 줄인다.
                absTorqueAdd(i) = Krelease * (e + releaseBand);

            elseif e > addBand
                % kappa가 목표보다 덜 음수이면 제동력이 부족한 상태
                % 단, 너무 크게 추가하면 lock되므로 작게만 더한다.
                absTorqueAdd(i) = Kadd * (e - addBand);

            else
                absTorqueAdd(i) = 0;
            end
        end
    end

    %% ---------- front/rear authority shaping ----------
    absTorqueAdd = absTorqueAdd .* [1.20; 1.20; 0.85; 0.85];

    %% ---------- saturation ----------
    negLimit = -0.95 * LIM.MAX_BRAKE_TRQ;
    posLimit =  0.22 * LIM.MAX_BRAKE_TRQ;

    absTorqueAdd = max(negLimit, min(posLimit, absTorqueAdd));

    %% ---------- low-pass filter ----------
    tau = 0.010;
    alpha = dt / (tau + dt);

    absTorqueAdd = (1 - alpha) * ctrlState.prevAbsTorque + alpha * absTorqueAdd;

    absTorqueAdd = max(negLimit, min(posLimit, absTorqueAdd));

    ctrlState.prevAbsTorque = absTorqueAdd;

    %% ---------- outputs ----------
    forceCmd.Fx_total       = Fx_total;
    forceCmd.brakeRatio     = brakeRatio;
    forceCmd.brakeTorqueAdd = absTorqueAdd;
    forceCmd.absActive      = any(abs(absTorqueAdd) > 1);

end