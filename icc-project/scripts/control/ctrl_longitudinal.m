function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트된 내부 상태

    % 1. 호출자 워크스페이스에서 VEH(차량 파라미터) 구조체 가져오기 (물리적 차원 변환용)
    VEH = [];
    try
        if evalin('caller', "exist('VEH', 'var')")
            VEH = evalin('caller', 'VEH');
        elseif evalin('caller', "exist('params', 'var')")
            params = evalin('caller', 'params');
            if isfield(params, 'VEH')
                VEH = params.VEH;
            end
        end
    catch
    end
    if isempty(VEH)
        VEH.mass = 1500;
        VEH.rw = 0.31;
    end

    % 2. 속도 오차 계산
    e_v = vxRef - vx;

    % 3. 적분 누적 및 Anti-windup
    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end
    ctrlState.intError = ctrlState.intError + e_v * dt;

    % intMax는 Nm 단위(Torque)이므로 힘(Force) 단위로 환산하여 적분기 클램핑 수행
    F_intMax = CTRL.LON.intMax / VEH.rw;
    ctrlState.intError = max(-F_intMax / CTRL.LON.Ki, min(F_intMax / CTRL.LON.Ki, ctrlState.intError));

    % 4. PI 제어력 계산 (가속도 오차에 질량을 곱해 물리적인 힘[N]으로 스케일링)
    Fx = VEH.mass * (CTRL.LON.Kp * e_v + CTRL.LON.Ki * ctrlState.intError);

    % 최대 가속도 제한
    Fx_max = LIM.MAX_AX * VEH.mass;
    Fx = max(-Fx_max, min(Fx_max, Fx));

    % 5. 저크 제한 (Jerk Limit: 가속도의 급격한 변화율 제한)
    if ~isfield(ctrlState, 'prevForce')
        ctrlState.prevForce = Fx;
    end
    dFx = (Fx - ctrlState.prevForce) / dt;
    max_dFx = LIM.MAX_JERK * VEH.mass;
    dFx_clamped = max(-max_dFx, min(max_dFx, dFx));
    Fx_total = ctrlState.prevForce + dFx_clamped * dt;

    % 내부 상태 저장
    ctrlState.prevForce = Fx_total;

    % 6. 출력 대입
    forceCmd.Fx_total = Fx_total;

    % 제동비율 계산 (0 ~ 1 사이)
    max_brake_force = (4 * LIM.MAX_BRAKE_TRQ) / VEH.rw;
    if Fx_total < 0
        forceCmd.brakeRatio = min(1.0, abs(Fx_total) / max_brake_force);
    else
        forceCmd.brakeRatio = 0.0;
    end
end

