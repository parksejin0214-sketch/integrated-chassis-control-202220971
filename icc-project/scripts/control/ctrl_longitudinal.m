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
    % 진단 결과: vx>2.0 임계 때문에 vx=1.65 m/s 근방에서 ABS가 꺼지고
    % 후륜이 kappa=-1(완전 lock)로 방치되어 absSlipRMS가 0.19에 고정됨
    % (FL/FR RMS=0.086 정상, RL/RR RMS=0.19=완전 lock). 하한을 낮춰 저속까지 ABS 유지.
    brakingLikely = (vx > 0.3) && (ax < -1.0) && any(kappa < -0.04);

    %% ---------- ABS slip target ----------
    kappaRef = -0.125;

    % v5(dead-band 축소)와 v6(게인 절반+tau 4배) 둘 다 absSlipRMS를 거의
    % 못 줄였고 v6는 stoppingDistance까지 악화시킴. dead-band가 만드는
    % on/off 스위칭(relay) 구조 자체가 limit-cycle의 원인으로 보여,
    % dead-band를 없애고 e=0부터 연속적으로 작동하는 P 제어로 변경.
    % 게인은 반응속도 유지를 위해 v4 원본 값(7200/1800)으로 복원.
    Krelease = 7200;
    Kadd     = 1800;

    absTorqueAdd = zeros(4,1);

    if brakingLikely
        for i = 1:4
            % e > 0 : 실제 slip이 목표보다 덜 음수, 즉 제동 여유 있음
            % e < 0 : 실제 slip이 목표보다 더 음수, 즉 lock 경향
            e = kappa(i) - kappaRef;

            if e < 0
                % kappa가 -0.12보다 더 음수이면 wheel lock 경향
                % 음수 torque로 scenario brake를 줄인다.
                absTorqueAdd(i) = Krelease * e;
            else
                % kappa가 목표보다 덜 음수이면 제동력이 부족한 상태
                absTorqueAdd(i) = Kadd * e;
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
    % dead-band 제거로 더 이상 relay 스위칭이 없으므로 tau는 v4 원본(0.010)로 복원.
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
