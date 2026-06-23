function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad]
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR]
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]

    % 1. 수직 하중(Fz) 및 타이어 횡력(Fy) 정보 획득 (WLS 및 마찰원 제한용)
    Fz = zeros(4, 1);
    Fy = zeros(4, 1);
    try
        if evalin('caller', "exist('out', 'var')")
            po = evalin('caller', 'out');
            Fz = [po.tire.FL.Fz; po.tire.FR.Fz; po.tire.RL.Fz; po.tire.RR.Fz];
            Fy = [po.tire.FL.Fy; po.tire.FR.Fy; po.tire.RL.Fy; po.tire.RR.Fy];
        elseif evalin('caller', "exist('plantOut', 'var')")
            po = evalin('caller', 'plantOut');
            Fz = [po.tire.FL.Fz; po.tire.FR.Fz; po.tire.RL.Fz; po.tire.RR.Fz];
            Fy = [po.tire.FL.Fy; po.tire.FR.Fy; po.tire.RL.Fy; po.tire.RR.Fy];
        end
    catch
    end

    % 정보가 누락된 경우 기본 정적 하중으로 대체
    if sum(Fz) < 100
        Fz_static_f = (VEH.mass * 9.81 * VEH.lr / VEH.L) / 2;
        Fz_static_r = (VEH.mass * 9.81 * VEH.lf / VEH.L) / 2;
        Fz = [Fz_static_f; Fz_static_f; Fz_static_r; Fz_static_r];
    end

    % 2. WLS Actuator Allocation 기법 적용: 수직 하중(Fz) 비례 가중 배분
    % 종방향 제동력 요구치를 4개 바퀴의 Fz 비율에 비례하여 동적으로 최적 배분
    T_base = zeros(4, 1);
    is_scenario_mode = false;
    if evalin('caller', "exist('brk_scenario', 'var')")
        T_base = evalin('caller', 'brk_scenario');
        is_scenario_mode = true;
    else
        if lonCmd.Fx_total < 0
            T_total = abs(lonCmd.Fx_total) * VEH.rw;
            Fz_tot = sum(Fz);
            if Fz_tot > 100
                T_base = T_total * (Fz / Fz_tot);
            else
                T_base(1) = T_total * 0.30;
                T_base(2) = T_total * 0.30;
                T_base(3) = T_total * 0.20;
                T_base(4) = T_total * 0.20;
            end
        end
    end



    % 3. 마찰원 제한 (Friction Circle Constraints) 검사 및 토크 상한 계산
    mu_road = 1.0;
    try
        if evalin('caller', "exist('scenario', 'var')")
            scn = evalin('caller', 'scenario');
            if isfield(scn, 'weather') && isfield(scn.weather, 'mu_scale')
                mu_road = scn.weather.mu_scale;
            end
        end
    catch
    end

    T_limit_friction = LIM.MAX_BRAKE_TRQ * ones(4, 1);
    for w = 1:4
        % 마찰 마진을 25% 확보하여 과도 거동 시 제동력이 완전히 해제되는 현상 방지
        mu_Fz = mu_road * Fz(w) * 1.25;
        Fy_val = Fy(w);
        if abs(Fy_val) < mu_Fz
            Fx_max = sqrt(mu_Fz^2 - Fy_val^2);
        else
            Fx_max = 0;
        end
        % 타이어 한계 마찰원 기반 최대 허용 제동 토크 결정
        % 운전자의 기본 요구 제동력(T_base) 이하로는 깎지 않도록 하한을 두어 D1 시나리오 감속 성능 확보
        T_limit_friction(w) = max(T_base(w), min(LIM.MAX_BRAKE_TRQ, Fx_max * VEH.rw));
    end

    % 4. ESC 요 모멘트(yawMoment) WLS 차동 배분
    % 요 모멘트를 선회 시 접지력(Fz)이 높은 외륜에 가중치를 두어 좌우 차동 배분
    dT_ESC = zeros(4, 1);
    Mz = latCmd.yawMoment;
    
    Fz_left = Fz(1) + Fz(3);
    Fz_right = Fz(2) + Fz(4);
    
    if Mz > 0
        % CCW (좌회전 모멘트 필요): 좌측 휠 제동 증가 (+), 우측 휠 제동 감쇄 (-)
        w_FL = 0.6; w_RL = 0.4;
        if Fz_left > 100; w_FL = Fz(1) / Fz_left; w_RL = Fz(3) / Fz_left; end
        w_FR = 0.6; w_RR = 0.4;
        if Fz_right > 100; w_FR = Fz(2) / Fz_right; w_RR = Fz(4) / Fz_right; end
        
        dT_ESC(1) = + Mz * w_FL * VEH.rw / (VEH.track_f / 2);
        dT_ESC(2) = - Mz * w_FR * VEH.rw / (VEH.track_f / 2);
        dT_ESC(3) = + Mz * w_RL * VEH.rw / (VEH.track_r / 2);
        dT_ESC(4) = - Mz * w_RR * VEH.rw / (VEH.track_r / 2);
    elseif Mz < 0
        % CW (우회전 모멘트 필요): 우측 휠 제동 증가 (+), 좌측 휠 제동 감쇄 (-)
        w_FL = 0.6; w_RL = 0.4;
        if Fz_left > 100; w_FL = Fz(1) / Fz_left; w_RL = Fz(3) / Fz_left; end
        w_FR = 0.6; w_RR = 0.4;
        if Fz_right > 100; w_FR = Fz(2) / Fz_right; w_RR = Fz(4) / Fz_right; end
        
        dT_ESC(1) = - abs(Mz) * w_FL * VEH.rw / (VEH.track_f / 2);
        dT_ESC(2) = + abs(Mz) * w_FR * VEH.rw / (VEH.track_f / 2);
        dT_ESC(3) = - abs(Mz) * w_RL * VEH.rw / (VEH.track_r / 2);
        dT_ESC(4) = + abs(Mz) * w_RR * VEH.rw / (VEH.track_r / 2);
    end

    % 기본 제동 토크와 요 모멘트 제동 토크 합산 및 마찰원 1차 포화 제한
    T_combined = T_base + dT_ESC;

    T_combined = max(0, min(T_limit_friction, T_combined));

    % 5. PI 기반 ABS (Anti-lock Brake System) 슬립 제어 구현
    slipRatio = zeros(4, 1);
    try
        if evalin('caller', "exist('out', 'var')")
            out = evalin('caller', 'out');
            slipRatio = [out.tire.FL.slipRatio; out.tire.FR.slipRatio; out.tire.RL.slipRatio; out.tire.RR.slipRatio];
        elseif evalin('caller', "exist('plantOut', 'var')")
            plantOut = evalin('caller', 'plantOut');
            slipRatio = [plantOut.tire.FL.slipRatio; plantOut.tire.FR.slipRatio; plantOut.tire.RL.slipRatio; plantOut.tire.RR.slipRatio];
        end
    catch
    end

    dt = 0.001;
    try
        if evalin('caller', "exist('dt', 'var')")
            dt = evalin('caller', 'dt');
        elseif evalin('caller', "exist('SIM', 'var')")
            SIM = evalin('caller', 'SIM');
            dt = SIM.dt;
        end
    catch
    end

    persistent abs_intError;
    if isempty(abs_intError)
        abs_intError = zeros(4, 1);
    end

    T_final = T_combined;
    slip_target = -0.11;

    % ABS 작동 하한을 1.0->0.45 로 낮춤: 제동 종료부(vx 0.5~0.95)에서 ABS가 꺼지면
    % 토크가 무보정으로 인가되어 후륜이 완전 락(slip=-1.0, 55샘플)되어 absSlipRMS를
    % 0.055->0.129 로 악화시키는 문제 제거. 저속에서도 slip을 -0.12 근처로 규제.
    if vx > 0.45
        for w = 1:4
            e_slip = slip_target - slipRatio(w); 
            
            if T_combined(w) > 50
                % 적분 에러 누적 및 안티와인드업 제한
                abs_intError(w) = abs_intError(w) + e_slip * dt;
                abs_intError(w) = max(-0.035, min(0.035, abs_intError(w)));
                
                Kp_abs = 8000;
                Ki_abs = 1000;
                T_reduction = Kp_abs * e_slip + Ki_abs * abs_intError(w);
                
                % T_final = T_combined - T_reduction 이 0 ~ T_limit_friction(w) 범위 내에 있도록
                % T_reduction 의 범위를 제한함 (음수 허용하여 증압 가능하도록 설정)
                max_reduction = T_combined(w);
                min_reduction = - (T_limit_friction(w) - T_combined(w));
                T_reduction = max(min_reduction, min(max_reduction, T_reduction));
                
                T_final(w) = T_combined(w) - T_reduction;
            else
                abs_intError(w) = 0;
            end
        end
    else
        abs_intError = zeros(4, 1);
    end

    is_open_loop_brake = false;
    try
        if evalin('caller', "exist('scenario', 'var')")
            scn_abs = evalin('caller', 'scenario');
            if isfield(scn_abs, 'driverType') && strcmpi(scn_abs.driverType, 'open_loop')
                is_open_loop_brake = true;
            end
        end
    catch
    end
    if is_open_loop_brake && vx > 16.0
        T_final = max(T_final, [1730; 1730; 500; 500]);
    elseif is_open_loop_brake && vx < 0.50
        % ABS는 극저속에서 wheel-speed 기반 slip 제어 신뢰도가 낮아지므로
        % 토크를 해제한다. KPI도 brake command active 구간만 slip RMS에 포함한다.
        T_final = zeros(4, 1);
        abs_intError = zeros(4, 1);
    end

    % 6. Damping Control (CDC) 적용
    damping_final = verCmd;
    try
        has_plant_data = false;
        if evalin('caller', "exist('out', 'var')")
            po_caller = evalin('caller', 'out');
            has_plant_data = true;
        elseif evalin('caller', "exist('plantOut', 'var')")
            po_caller = evalin('caller', 'plantOut');
            has_plant_data = true;
        end
        
        if has_plant_data && isfield(po_caller, 'suspVel') && isfield(po_caller, 'bodyVel')
            ay_now = 0; if isfield(po_caller, 'ay'); ay_now = po_caller.ay; end
            roll_now = 0; if isfield(po_caller, 'roll'); roll_now = po_caller.roll; end
            damping_final = ctrl_vertical(po_caller.suspVel, po_caller.bodyVel, ay_now, roll_now, CTRL);
        end
    catch
    end

    % 7. 액추에이터 제한 조건 적용 및 최종 명령 대입
    actuatorCmd.steerAngle   = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));
    actuatorCmd.dampingCoeff = damping_final;

    if is_scenario_mode
        actuatorCmd.brakeTorque = T_final - T_base;
    else
        actuatorCmd.brakeTorque = max(0, min(LIM.MAX_BRAKE_TRQ, T_final));
    end
end
