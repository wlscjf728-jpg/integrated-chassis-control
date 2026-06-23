function [dampingCmd, ctrlState] = ctrl_vertical(varargin)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Body-bounce / wheel-hop 모드 분리 및 ride comfort 개선을 위한 가변 감쇠.
%   또한 횡/종 가속도에 따른 피드포워드 감쇠 제어를 통해 롤/피치 안정성(LTR 개선) 도모.
%
%   Inputs:
%       Supports two calling signatures:
%       1) [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%       2) dampingCmd = ctrl_vertical(suspVel, bodyVel, ay, roll, CTRL)
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%       ctrlState  - 내부 상태

    % 1. 입력 파싱
    ctrlState = [];
    if nargin == 5
        % Signature: ctrl_vertical(suspVel, bodyVel, ay, roll, CTRL)
        suspVel = varargin{1};
        bodyVel = varargin{2};
        ay = varargin{3};
        roll = varargin{4};
        CTRL = varargin{5};
    elseif nargin == 4
        % Signature: ctrl_vertical(suspState, ctrlState, CTRL, dt)
        suspState = varargin{1};
        ctrlState = varargin{2};
        CTRL = varargin{3};
        
        bodyVel = suspState.zs_dot;
        suspVel = suspState.zs_dot - suspState.zu_dot;
        
        ay = 0;
        if isfield(suspState, 'ay')
            ay = suspState.ay;
        end
        if isfield(ctrlState, 'ay')
            ay = ctrlState.ay;
        end
    else
        error('Invalid number of arguments in ctrl_vertical');
    end

    % 2. 호출자 워크스페이스 등에서 종가속도(ax) 정보 추출 시도
    ax = 0;
    try
        if evalin('caller', "exist('ax_now', 'var')")
            ax = evalin('caller', 'ax_now');
        elseif evalin('caller', "exist('plantOut', 'var')")
            po = evalin('caller', 'plantOut');
            if isfield(po, 'ax')
                ax = po.ax;
            end
        end
    catch
    end

    % 3. Skyhook & Roll/Pitch Compensation Damping 계산
    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;
    skyGain = CTRL.VER.skyGain;
    
    % 가속도에 비례한 가변 최소 감쇠 (Roll & Pitch Control)
    % 횡가속도 및 종가속도가 클수록 바디의 자세 변화를 억제하기 위해 감쇠 계수 하한을 높임.
    k_roll = 1500;  % [Ns^2 / m^2]
    k_pitch = 800;  % [Ns^2 / m^2]
    
    % 제동 시 하중이 쏠리는 특성을 반영하여 전륜/후륜의 최소 감쇠(floor)를 비대칭 설계
    c_base = cMin * ones(4, 1);
    for w = 1:4
        if w == 1 || w == 2
            % 전륜 FL, FR: 선회 롤 및 제동 피칭을 전적으로 지지
            c_base(w) = cMin + k_roll * abs(ay) + k_pitch * max(0, -ax);
        else
            % 후륜 RL, RR: 제동 시 하중이 빠지므로 ax에 의한 감쇠를 억제하여 접지력 유지
            c_base(w) = cMin + k_roll * abs(ay);
        end
        c_base(w) = min(cMax, c_base(w));
    end

    dampingCmd = zeros(4, 1);
    for i = 1:4
        vs = bodyVel(i);
        vrel = suspVel(i);
        
        % Skyhook 조건: sprung mass 속도와 relative velocity 방향이 같을 때 감쇠력 증가
        if vs * vrel > 0
            if abs(vrel) > 1e-4
                c_sky = c_base(i) + skyGain * abs(vs) / abs(vrel);
                dampingCmd(i) = min(cMax, max(c_base(i), c_sky));
            else
                dampingCmd(i) = cMax;
            end
        else
            dampingCmd(i) = c_base(i);
        end
    end
end
