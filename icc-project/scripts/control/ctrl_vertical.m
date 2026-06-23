function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC — Hybrid Skyhook-Groundhook semi-active damping
%
%   설계 개요:
%     - Skyhook: sprung mass 절대 진동 억제 (탑승자 승차감)
%     - Groundhook: unsprung mass 진동 억제 (타이어 노면 접촉, 핸들링)
%     - Hybrid α: 0.7·skyhook + 0.3·groundhook
%     - On-off semi-active: 조건 만족 시 cMax, 아니면 cMin
%     - cMin ≤ c ≤ cMax 제한

    %% -------- 내부 상태 초기화 (사용 안 하지만 시그니처 유지) --------
    if ~isfield(ctrlState, 'prevDamping'); ctrlState.prevDamping = 1500 * ones(4,1); end

    %% -------- 게인 --------
    cMin    = CTRL.VER.cMin;
    cMax    = CTRL.VER.cMax;
    cNom    = 0.5 * (cMin + cMax);  % 중립값
    alpha   = 0.7;  % skyhook 비중

    %% -------- per-wheel hybrid skyhook --------
    dampingCmd = zeros(4, 1);
    for i = 1:4
        zs_dot = suspState.zs_dot(i);
        zu_dot = suspState.zu_dot(i);
        rel    = zs_dot - zu_dot;  % 서스펜션 변형 속도

        % Skyhook 조건: sprung velocity 와 relative velocity 부호 같으면 ON
        sky_on = (zs_dot * rel) > 0;

        % Groundhook 조건: unsprung velocity 와 -relative velocity 부호 같으면 ON
        ground_on = (zu_dot * (-rel)) > 0;

        % Hybrid 결정
        score = alpha * double(sky_on) + (1 - alpha) * double(ground_on);

        if score > 0.5
            c = cMax;
        elseif score > 0.0
            c = cNom + (cMax - cNom) * score;
        else
            c = cMin;
        end

        % 1차 LPF 적용 (chattering 방지, tau=0.05s)
        beta = dt / (0.05 + dt);
        c_filt = (1 - beta) * ctrlState.prevDamping(i) + beta * c;

        dampingCmd(i) = max(cMin, min(cMax, c_filt));
    end

    ctrlState.prevDamping = dampingCmd;

end