%% ====================================================================
%  四开关 Buck-Boost 数字电源 —— P5 创新点 A1「重写恒流环」对比仿真
%
%  针对源码 bug（PID.c:60-66，踩坑记录坑 5 / 坑 21）：
%    原版恒流环把电流环输出 i0（电流 LSB 量纲）直接加在电压参考
%    Vout_ref（电压 LSB 量纲）上 —— 量纲混用 + 积分只限上限无下限
%    -> 负载阶跃时「暴力限流」触发 2P2Z b1 负系数 bang-bang（坑 21）。
%
%  A1 改进（CV/CC 双环取小 + anti-windup）：
%    1) 量纲归一：电流环输出直接是「限流参考电压 Vlim」(V)，不混电流 LSB；
%    2) 取小逻辑：Vref_ctrl = min(Vref_eff, Vlim)，CV/CC 干净切换；
%    3) anti-windup：Vlim 饱和时积分反算，积分下限补 0（源码缺失）。
%
%  对比场景（恒流限流，非保护）：
%    Vin=24V，Vout_set=12V（Buck），Iref=10A（默认限流点）
%    负载 18Ω(0.67A,CV) --0.5s 阶跃--> 18//1Ω=0.947Ω(想冲 12.67A,CC)
%    --0.7s 恢复--> 18Ω。观察 Iout/Vout 是否干净限流、有无 bang-bang、
%    有无 OCP(10.5A) 误触发。
%
%  两种电流环（CURRENT_LOOP）：
%    'original' = 照搬源码（量纲混用，i0 直接加 Vref）
%    'A1'       = 重写（量纲归一 + min 取小 + 积分反算 anti-windup）
% =====================================================================
close all;

%% ================= 全局参数（README / function.h / PID.c） =================
Vin      = 24;      % 输入电压 [V]
L        = 22e-6;   % 电感 22uH
Cin      = 220e-6;  % 输入电容
Cout     = 690e-6;  % 输出电容 220u+470u
Rds_on   = 6.2e-3;  % MOS 导通电阻
fsw      = 181333;  % 开关频率（2P2Z 系数按此采样率整定，不可改）
Ts       = 1/fsw;
deadtime = 0.2e-6;
dz       = deadtime/Ts;

Rload_normal = 18;      % 正常负载 18Ω（12V 输出 0.67A，CV）
Rfault_val   = 1.0;     % 故障支路 1Ω，并联后 18//1 = 0.947Ω（12V 想冲 12.67A，CC）
Vout_set     = 12;      % 设定电压 12V（Buck）
Iref         = 10;      % 限流点 10A（默认）
Tsim         = 0.9;     % 仿真时长
breaker_times = [0.50 0.70];  % 0.5s 投入故障，0.7s 恢复

% A1 电流环积分斜率系数 KI_LOOP [V/(A·s)]
%  过流 2.67A 时 Vlim 下降斜率 = KI_LOOP*2.67 V/s；下调 2.53V 需 ~2.53/(KI_LOOP*2.67) s
%  取值需在「够快(<55ms OCP 保持)」与「不触发电压环 bang-bang」之间权衡。
%  坑 15/20 结论：Vlim(等效 Vref)变化斜率 120V/s 安全、2394V/s 触发 bang-bang。
%  KI_LOOP=75 -> 过流 2.67A 时斜率约 200V/s，下调 2.53V 约 12.7ms，仍 < 55ms OCP 窗口。
KI_LOOP = 75;

outdir = 'C:/Users/ll/Desktop/大创/Simulink';
if ~exist(outdir,'dir'), mkdir(outdir); end

%% ================= 建模型（一次，两模式复用） =================
model = 'buckboost_A1';
if bdIsLoaded(model), bdclose(model); end
new_system(model);
open_system(model);
load_system('sps_lib');

% 功率级
add_block('sps_lib/Sources/DC Voltage Source', [model '/Vin'],  'Position',[40  160 80  205]);
add_block('sps_lib/Power Electronics/Mosfet',                 [model '/Q2'],  'Position',[200 90  245 135]);
add_block('sps_lib/Power Electronics/Mosfet',                 [model '/Q4'],  'Position',[200 210 245 255]);
add_block('sps_lib/Power Electronics/Mosfet',                 [model '/Q3'],  'Position',[460 90  505 135]);
add_block('sps_lib/Power Electronics/Mosfet',                 [model '/Q5'],  'Position',[460 210 505 255]);
add_block('sps_lib/Passives/Series RLC Branch',               [model '/L'],   'Position',[330 130 375 175]);
add_block('sps_lib/Passives/Series RLC Branch',               [model '/Cin'], 'Position',[120 300 160 345]);
add_block('sps_lib/Passives/Series RLC Branch',               [model '/Cout'],'Position',[580 130 625 175]);
add_block('sps_lib/Passives/Series RLC Branch',               [model '/Rload'],'Position',[580 300 625 345]);
add_block('sps_lib/Power Electronics/Ideal Switch',             [model '/IdealSw'], 'Position',[700 320 745 365]);
add_block('sps_lib/Passives/Series RLC Branch',               [model '/Rfault'],'Position',[760 300 805 345]);
add_block('sps_lib/powergui',                                 [model '/powergui'],'Position',[40 400 120 445]);

% 测量
add_block('sps_lib/Sensors and Measurements/Voltage Measurement', [model '/Vm_in'], 'Position',[120 380 160 425]);
add_block('sps_lib/Sensors and Measurements/Current Measurement', [model '/Im_in'], 'Position',[80  160 120 205]);
add_block('sps_lib/Sensors and Measurements/Current Measurement', [model '/Im_L'],  'Position',[280 160 320 205]);
add_block('sps_lib/Sensors and Measurements/Voltage Measurement', [model '/Vm_out'],'Position',[640 380 680 425]);
add_block('sps_lib/Sensors and Measurements/Current Measurement', [model '/Im_out'],'Position',[640 160 680 205]);

% 控制器（5 输入 4 输出）
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/BB_LoopCtl'], 'Position',[820 90 900 250]);

% PWM 生成
add_block('simulink/Sources/Repeating Sequence', [model '/saw'], 'Position',[560 20 610 60]);
add_block('simulink/Math Operations/Sum', [model '/Sum_Q2'], 'Position',[820 20 850 50]);
add_block('simulink/Math Operations/Sum', [model '/Sum_Q5'], 'Position',[900 20 930 50]);
add_block('simulink/Sources/Constant', [model '/dz'], 'Position',[840 70 870 100]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q2'], 'Position',[860 20 890 60]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q4'], 'Position',[950 20 980 60]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q5'], 'Position',[860 90 890 130]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q3'], 'Position',[950 90 980 130]);

% 参考输入
add_block('simulink/Sources/From Workspace', [model '/Vref'], 'Position',[700 120 745 155]);
add_block('simulink/Sources/From Workspace', [model '/SwGate'], 'Position',[700 270 745 305]);
add_block('simulink/Sources/Constant', [model '/Iref'], 'Position',[700 220 740 255]);

% 记录
add_block('simulink/Sinks/To Workspace', [model '/Vout_ts'],'Position',[640 450 690 490]);
add_block('simulink/Sinks/To Workspace', [model '/Iout_ts'],'Position',[700 450 750 490]);
add_block('simulink/Sinks/To Workspace', [model '/Vref_ts'],'Position',[760 450 810 490]);
add_block('simulink/Sinks/To Workspace', [model '/DQ2_ts'], 'Position',[880 450 930 490]);
add_block('simulink/Sinks/To Workspace', [model '/DQ5_ts'], 'Position',[940 450 990 490]);
add_block('simulink/Sinks/To Workspace', [model '/Vlim_ts'],'Position',[1000 450 1050 490]);

%% ================= 电流环两种实现的代码片段 =================
% --- 原版（照搬源码 PID.c:55-66，量纲混用 + 只限积分上限）---
original_loop = sprintf([ ...
'    %% ---- 原版电流环（照搬源码，量纲混用）----\n' ...
'    IErr_LSB = (Iref_set - Iout) * LSB_PER_A;\n' ...
'    i0_LSB = Integral + ILOOP_KP*IErr_LSB + ILOOP_KD*(IErr_LSB - IErr1);\n' ...
'    Integral = Integral + ILOOP_KI*IErr_LSB;\n' ...
'    if Integral > ADC_MAX, Integral = ADC_MAX; end\n' ...
'    if Integral < 0, Integral = 0; end\n' ...
'    IErr1 = IErr_LSB;\n' ...
'    i0_V = i0_LSB * V_PER_LSB;\n' ...
'    Vref_ctrl = Vref_eff + i0_V;\n' ...
'    if Vref_ctrl > Vref_eff, Vref_ctrl = Vref_eff; end\n' ...
'    if Vref_ctrl < 0, Vref_ctrl = 0; end\n' ...
'    Vlim = Vref_ctrl;\n']);

% --- A1（重写：量纲归一 + min 取小 + 积分反算 anti-windup）---
a1_loop = sprintf([ ...
'    %% ---- A1 重写电流环（量纲归一 + 取小 + anti-windup）----\n' ...
'    IErr_A = Iref_set - Iout;                          %% 电流误差[A]\n' ...
'    Integral = Integral + KI_LOOP * IErr_A * Ts_ctrl;  %% 积分控制器，Integral 即 Vlim[V]\n' ...
'    Vlim = Integral;\n' ...
'    if Vlim > Vref_eff        %% CV 接管：Vlim 饱和到当前参考，积分反算防 windup\n' ...
'        Vlim = Vref_eff;\n' ...
'        Integral = Vref_eff;\n' ...
'    elseif Vlim < 0           %% 下限补齐（源码缺失）\n' ...
'        Vlim = 0;\n' ...
'        Integral = 0;\n' ...
'    end\n' ...
'    Vref_ctrl = Vlim;         %% 等效 min(Vref_eff, Vlim)\n']);

%% ================= 控制器模板 =================
tpl = [ ...
'function [D_Q2, D_Q5, mode_out, vlim_out] = BB_LoopCtl(Vout, Iout, Vref_set, Iref_set, Vin)\n' ...
'%% 双环（电流环可切换）+ 软启动 + 三模式 + OVP/OCP 兜底\n' ...
'persistent VErr1 VErr2 u1 Integral IErr1 ramp softstart_done BBFlag ovp_cnt ocp_cnt pwm_en\n' ...
'if isempty(VErr1)\n' ...
'    VErr1 = 0; VErr2 = 0; u1 = 0; IErr1 = 0; ramp = 0; softstart_done = 0; BBFlag = 0;\n' ...
'    Integral = %s;\n' ...
'    ovp_cnt = 0; ocp_cnt = 0; pwm_en = 1;\n' ...
'end\n' ...
'%% 定点常量\n' ...
'BUCK_b0 = 5271;  BUCK_b1 = -10363;  BUCK_b2 = 5093;\n' ...
'BOOST_b0 = 8044; BOOST_b1 = -15813; BOOST_b2 = 7772;\n' ...
'ILOOP_KP = 6; ILOOP_KI = 3; ILOOP_KD = 1;\n' ...
'LSB_PER_V = 155.5;  LSB_PER_A = 769.2;  V_PER_LSB = 1/155.5;\n' ...
'PERIOD = 30000;  ADC_MAX = 8190;\n' ...
'DUTY_BUCK_MAX  = 28200/PERIOD;  DUTY_BUCK_MIN  = 100/PERIOD;\n' ...
'DUTY_BOOST_MAX = 19500/PERIOD;  DUTY_BOOST_MIN = 100/PERIOD;\n' ...
'DUTY_BOOST_FIX = 1800/PERIOD;   DUTY_BUCK_FIX = 28200/PERIOD;  DUTY_MIX_FIX = 24000/PERIOD;\n' ...
'U0_BUCK_MAX  = 28200*256/3;  U0_BOOST_MAX = 19500*256/3;\n' ...
'Ts_ctrl = 1/181333;  KI_LOOP = %g;\n' ...
'OCP_N = round(55e-3/Ts_ctrl);   %% OCP 保持 55ms（源码 OCPCnt>10 @ 5ms 节拍）\n' ...
'%% ===== 软启动 ramp（120V/s 慢速，避免 2P2Z b1 负系数 bang-bang）=====\n' ...
'if ~softstart_done\n' ...
'    ramp = ramp + 120/181333;\n' ...
'    if ramp >= Vref_set, ramp = Vref_set; softstart_done = 1; end\n' ...
'end\n' ...
'if softstart_done, Vref_eff = Vref_set; else, Vref_eff = ramp; end\n' ...
'%% ===== 电流环（可切换）=====\n' ...
'%s' ...
'%% ===== BBMode 迟滞判定（function.c:1463，场景固定 Buck 但保留完整逻辑）=====\n' ...
'PreBBFlag = BBFlag;\n' ...
'switch BBFlag\n' ...
'    case 0\n' ...
'        if Vref_eff < 0.8*Vin, BBFlag = 1;\n' ...
'        elseif Vref_eff > 1.2*Vin, BBFlag = 2;\n' ...
'        else, BBFlag = 3; end\n' ...
'    case 1\n' ...
'        if Vref_eff > 1.2*Vin, BBFlag = 2;\n' ...
'        elseif Vref_eff > 0.85*Vin, BBFlag = 3; end\n' ...
'    case 2\n' ...
'        if Vref_eff < 0.8*Vin, BBFlag = 1;\n' ...
'        elseif Vref_eff < 1.15*Vin, BBFlag = 3; end\n' ...
'    case 3\n' ...
'        if Vref_eff < 0.8*Vin, BBFlag = 1;\n' ...
'        elseif Vref_eff > 1.2*Vin, BBFlag = 2; end\n' ...
'end\n' ...
'if BBFlag ~= PreBBFlag\n' ...
'    u1 = 0; Integral = 0;\n' ...
'end\n' ...
'%% ===== 电压环（2P2Z，按 BBFlag 选系数）=====\n' ...
'VErr_LSB = (Vref_ctrl - Vout) * LSB_PER_V;\n' ...
'if BBFlag == 1   %% Buck\n' ...
'    u0 = u1 + BUCK_b0*VErr_LSB + BUCK_b1*VErr1 + BUCK_b2*VErr2;\n' ...
'    if u0 > U0_BUCK_MAX, u0 = U0_BUCK_MAX; end\n' ...
'    if u0 < 0, u0 = 0; end\n' ...
'    VErr2 = VErr1; VErr1 = VErr_LSB; u1 = u0;\n' ...
'    D_Q2 = floor(u0/256)*3/PERIOD;  D_Q5 = DUTY_BOOST_FIX;\n' ...
'    if D_Q2 > DUTY_BUCK_MAX, D_Q2 = DUTY_BUCK_MAX; end\n' ...
'    if D_Q2 < DUTY_BUCK_MIN, D_Q2 = DUTY_BUCK_MIN; end\n' ...
'elseif BBFlag == 2   %% Boost\n' ...
'    u0 = u1 + BOOST_b0*VErr_LSB + BOOST_b1*VErr1 + BOOST_b2*VErr2;\n' ...
'    if u0 > U0_BOOST_MAX, u0 = U0_BOOST_MAX; end\n' ...
'    if u0 < 0, u0 = 0; end\n' ...
'    VErr2 = VErr1; VErr1 = VErr_LSB; u1 = u0;\n' ...
'    D_Q2 = DUTY_BUCK_FIX;  D_Q5 = floor(u0/256)*3/PERIOD;\n' ...
'    if D_Q5 > DUTY_BOOST_MAX, D_Q5 = DUTY_BOOST_MAX; end\n' ...
'    if D_Q5 < DUTY_BOOST_MIN, D_Q5 = DUTY_BOOST_MIN; end\n' ...
'else   %% Mix\n' ...
'    u0 = u1 + BOOST_b0*VErr_LSB + BOOST_b1*VErr1 + BOOST_b2*VErr2;\n' ...
'    if u0 > U0_BOOST_MAX, u0 = U0_BOOST_MAX; end\n' ...
'    if u0 < 0, u0 = 0; end\n' ...
'    VErr2 = VErr1; VErr1 = VErr_LSB; u1 = u0;\n' ...
'    D_Q2 = DUTY_MIX_FIX;  D_Q5 = floor(u0/256)*3/PERIOD;\n' ...
'    if D_Q5 > DUTY_BOOST_MAX, D_Q5 = DUTY_BOOST_MAX; end\n' ...
'    if D_Q5 < DUTY_BOOST_MIN, D_Q5 = DUTY_BOOST_MIN; end\n' ...
'end\n' ...
'%% ===== OVP/OCP 兜底保护（防发散）=====\n' ...
'if Vout >= 50, pwm_en = 0; end                 %% OVP 锁死\n' ...
'if (Iout >= 10.5), ocp_cnt = ocp_cnt + 1; else, ocp_cnt = 0; end\n' ...
'if ocp_cnt > OCP_N, pwm_en = 0; end            %% OCP 关断（55ms 保持，给电流环留限流窗口）\n' ...
'if pwm_en == 0\n' ...
'    D_Q2 = 0; D_Q5 = 0;\n' ...
'end\n' ...
'mode_out = BBFlag;\n' ...
'vlim_out = Vlim;\n' ...
'end\n'];

% 两种模式的 Integral 初始值
integral_init_original = '0';        % 源码 static int32_t I_Integral = 0
integral_init_a1       = 'Vref_set'; % A1：Vlim 初始饱和在设定参考

script_original = sprintf(tpl, integral_init_original, KI_LOOP, original_loop);
script_a1       = sprintf(tpl, integral_init_a1,       KI_LOOP, a1_loop);

rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','-and','Path',[model '/BB_LoopCtl']);
set_param([model '/BB_LoopCtl'], 'SystemSampleTime', num2str(Ts));
chart.Script = script_original;   % 先设一次以生成正确端口（5 入 4 出），循环内再切换模式

%% ---- 功率级参数 ----
set_param([model '/Vin'], 'Amplitude', num2str(Vin));
set_param([model '/Q2'], 'Ron', num2str(Rds_on));
set_param([model '/Q4'], 'Ron', num2str(Rds_on));
set_param([model '/Q3'], 'Ron', num2str(Rds_on));
set_param([model '/Q5'], 'Ron', num2str(Rds_on));
set_param([model '/L'],  'BranchType','L', 'Inductance', num2str(L));
set_param([model '/Cin'],'BranchType','RC', 'Resistance','0.01', 'Capacitance', num2str(Cin), 'Setx0','on', 'InitialVoltage', num2str(Vin));
set_param([model '/Cout'],'BranchType','RC', 'Resistance','0.03', 'Capacitance', num2str(Cout), 'Setx0','on', 'InitialVoltage','0');
set_param([model '/Rload'],'BranchType','R', 'Resistance', num2str(Rload_normal));
set_param([model '/Rfault'],'BranchType','R', 'Resistance', num2str(Rfault_val));
set_param([model '/IdealSw'], 'Ron', '0.01', 'IC', '0');
set_param([model '/Iref'], 'Value', num2str(Iref));

% PWM
set_param([model '/saw'], 'rep_seq_t', sprintf('[0 %s]', num2str(Ts)), 'rep_seq_y', '[0 1]');
set_param([model '/dz'], 'Value', num2str(dz));
set_param([model '/Sum_Q2'], 'Inputs', '++');
set_param([model '/Sum_Q5'], 'Inputs', '++');
set_param([model '/Rel_Q2'], 'Operator', '<');
set_param([model '/Rel_Q4'], 'Operator', '>');
set_param([model '/Rel_Q5'], 'Operator', '<');
set_param([model '/Rel_Q3'], 'Operator', '>');

% 记录块
for b = {'Vout_ts','Iout_ts','Vref_ts','DQ2_ts','DQ5_ts','Vlim_ts'}
    set_param([model '/' b{1}], 'SaveFormat','Timeseries', 'VariableName', b{1});
end

%% ---- 功率电路连线 ----
add_line(model, 'Vin/RConn1', 'Im_in/LConn1', 'autorouting','on');
add_line(model, 'Im_in/RConn1', 'Q2/LConn1', 'autorouting','on');
add_line(model, 'Im_in/RConn1', 'Cin/LConn1', 'autorouting','on');
add_line(model, 'Q2/RConn1', 'Q4/LConn1', 'autorouting','on');
add_line(model, 'Q2/RConn1', 'Im_L/LConn1', 'autorouting','on');
add_line(model, 'Im_L/RConn1', 'L/LConn1', 'autorouting','on');
add_line(model, 'L/RConn1', 'Q3/RConn1', 'autorouting','on');
add_line(model, 'L/RConn1', 'Q5/LConn1', 'autorouting','on');
add_line(model, 'Q3/LConn1', 'Im_out/LConn1', 'autorouting','on');
add_line(model, 'Im_out/RConn1', 'Cout/LConn1', 'autorouting','on');
add_line(model, 'Im_out/RConn1', 'Rload/LConn1', 'autorouting','on');
add_line(model, 'Im_out/RConn1', 'IdealSw/LConn1', 'autorouting','on');
add_line(model, 'IdealSw/RConn1', 'Rfault/LConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Q4/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Q5/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Cin/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Cout/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Rload/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Rfault/RConn1', 'autorouting','on');
add_line(model, 'Vm_in/LConn1', 'Im_in/RConn1', 'autorouting','on');
add_line(model, 'Vm_in/LConn2', 'Vin/LConn1', 'autorouting','on');
add_line(model, 'Vm_out/LConn1', 'Im_out/RConn1', 'autorouting','on');
add_line(model, 'Vm_out/LConn2', 'Vin/LConn1', 'autorouting','on');

%% ---- 信号连线 ----
add_line(model, 'Vm_out/1', 'BB_LoopCtl/1', 'autorouting','on');   % Vout
add_line(model, 'Im_out/1', 'BB_LoopCtl/2', 'autorouting','on');   % Iout
add_line(model, 'Vref/1', 'BB_LoopCtl/3', 'autorouting','on');     % Vref_set
add_line(model, 'Iref/1', 'BB_LoopCtl/4', 'autorouting','on');     % Iref_set
add_line(model, 'Vm_in/1', 'BB_LoopCtl/5', 'autorouting','on');    % Vin
add_line(model, 'BB_LoopCtl/1', 'Sum_Q2/1', 'autorouting','on');   % D_Q2
add_line(model, 'BB_LoopCtl/2', 'Sum_Q5/1', 'autorouting','on');   % D_Q5
add_line(model, 'dz/1', 'Sum_Q2/2', 'autorouting','on');
add_line(model, 'dz/1', 'Sum_Q5/2', 'autorouting','on');
add_line(model, 'saw/1', 'Rel_Q2/1', 'autorouting','on');
add_line(model, 'saw/1', 'Rel_Q4/1', 'autorouting','on');
add_line(model, 'saw/1', 'Rel_Q5/1', 'autorouting','on');
add_line(model, 'saw/1', 'Rel_Q3/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/1', 'Rel_Q2/2', 'autorouting','on');
add_line(model, 'Sum_Q2/1', 'Rel_Q4/2', 'autorouting','on');
add_line(model, 'BB_LoopCtl/2', 'Rel_Q5/2', 'autorouting','on');
add_line(model, 'Sum_Q5/1', 'Rel_Q3/2', 'autorouting','on');
add_line(model, 'Rel_Q2/1', 'Q2/1', 'autorouting','on');
add_line(model, 'Rel_Q4/1', 'Q4/1', 'autorouting','on');
add_line(model, 'Rel_Q5/1', 'Q5/1', 'autorouting','on');
add_line(model, 'Rel_Q3/1', 'Q3/1', 'autorouting','on');
add_line(model, 'SwGate/1', 'IdealSw/1', 'autorouting','on');
add_line(model, 'Vm_out/1', 'Vout_ts/1', 'autorouting','on');
add_line(model, 'Im_out/1', 'Iout_ts/1', 'autorouting','on');
add_line(model, 'Vref/1', 'Vref_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/1', 'DQ2_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/2', 'DQ5_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/4', 'Vlim_ts/1', 'autorouting','on');

% Solver
set_param(model, 'SolverType','Variable-step', 'Solver','ode23tb', ...
    'MaxStep','1e-7', 'RelTol','1e-4', 'AbsTol','1e-6');
set_param(model, 'ZeroCrossAlgorithm', 'Adaptive', 'IgnoredZcDiagnostic', 'none');

save_system(model, [outdir '/buckboost_A1.slx']);
disp('=== 模型已保存 ===');

%% ================= 场景时间序列（两种模式共用） =================
t_ref = (0:1e-3:Tsim)';
vref_data = [t_ref, Vout_set*ones(size(t_ref))];
sw_seq = zeros(size(t_ref));
sw_seq(t_ref >= breaker_times(1) & t_ref < breaker_times(2)) = 1;
sw_data = [t_ref, sw_seq];
set_param([model '/Vref'], 'VariableName', 'vref_data');
set_param([model '/SwGate'], 'VariableName', 'sw_data');
set_param(model, 'StopTime', num2str(Tsim));

%% ================= 逐模式仿真 =================
modes = {'original', 'A1'};
scripts = {script_original, script_a1};
results = struct();
for mi = 1:2
    fprintf('\n========== 模式：%s ==========\n', modes{mi});
    chart.Script = scripts{mi};
    simOut = sim(model);

    Vout_t = simOut.get('Vout_ts'); Iout_t = simOut.get('Iout_ts');
    DQ2_t  = simOut.get('DQ2_ts');  Vlim_t = simOut.get('Vlim_ts');
    Vref_t = simOut.get('Vref_ts');

    results(mi).name  = modes{mi};
    results(mi).Vout  = Vout_t;
    results(mi).Iout  = Iout_t;
    results(mi).DQ2   = DQ2_t;
    results(mi).Vlim  = Vlim_t;
    results(mi).Vref  = Vref_t;

    % 关键指标
    t  = Vout_t.Time;  v = Vout_t.Data;  i = Iout_t.Data;
    seg = t >= breaker_times(1) & t < breaker_times(2);   % 故障投入区间
    fprintf('  Iout 峰值 = %.2f A（故障区间内）\n', max(i(seg)));
    fprintf('  Iout 稳态(故障末) = %.2f A\n', i(find(seg,1,'last')));
    fprintf('  Vout 稳态(故障末) = %.2f V\n', v(find(seg,1,'last')));
    % bang-bang 检测：故障区间内 Iout 过零次数
    i_seg = i(seg);
    zc = sum(diff(sign(i_seg)) ~= 0);
    fprintf('  Iout 过零次数(故障区间) = %d（>0 表 bang-bang 振荡）\n', zc);
    fprintf('  Vout 峰值 = %.2f V\n', max(v));
end

%% ================= 对比绘图 =================
r1 = results(1); r2 = results(2);
t1 = r1.Vout.Time;   v1 = r1.Vout.Data;   i1 = r1.Iout.Data;
t2 = r2.Vout.Time;   v2 = r2.Vout.Data;   i2 = r2.Iout.Data;
tl1 = r1.Vlim.Time;  vlim1 = r1.Vlim.Data;
tl2 = r2.Vlim.Time;  vlim2 = r2.Vlim.Data;
tref = r1.Vref.Time; vref = r1.Vref.Data;

fig = figure('Name','A1_compare','Visible','off','Position',[60 60 1080 860]);
colors = lines(2);

% 子图1：Vout 对比
subplot(4,1,1);
plot(t1*1e3, v1, 'Color', colors(1,:), 'LineWidth', 1.1); hold on;
plot(t2*1e3, v2, 'Color', colors(2,:), 'LineWidth', 1.1);
plot(tref*1e3, vref, 'k--', 'LineWidth', 0.8);
ylabel('Vout (V)'); title('A1 创新点对比：输出电压 Vout（原版 vs 重写恒流环）'); grid on;
legend('原版(量纲混用)','A1(取小+anti-windup)','Vref','Location','best');
ylim([-2 14]);

% 子图2：Iout 对比
subplot(4,1,2);
plot(t1*1e3, i1, 'Color', colors(1,:), 'LineWidth', 1.1); hold on;
plot(t2*1e3, i2, 'Color', colors(2,:), 'LineWidth', 1.1);
yline(10, 'k:', 'Iref=10A'); yline(10.5, 'r:', 'OCP=10.5A');
ylabel('Iout (A)'); title('输出电流 Iout（红虚线 OCP 阈值 10.5A）'); grid on;
legend('原版','A1','Iref','OCP','Location','best');
ylim([-5 16]);

% 子图3：限流参考 Vlim 对比
subplot(4,1,3);
plot(tl1*1e3, vlim1, 'Color', colors(1,:), 'LineWidth', 1.1); hold on;
plot(tl2*1e3, vlim2, 'Color', colors(2,:), 'LineWidth', 1.1);
ylabel('Vlim (V)'); title('电流环输出限流参考 Vlim（A1 为真实电压量纲，原版为混用量纲）'); grid on;
legend('原版','A1','Location','best');

% 子图4：占空比 D_Q2 对比
subplot(4,1,4);
d1 = r1.DQ2.Data; d2 = r2.DQ2.Data; td1 = r1.DQ2.Time; td2 = r2.DQ2.Time;
plot(td1*1e3, d1, 'Color', colors(1,:), 'LineWidth', 0.8); hold on;
plot(td2*1e3, d2, 'Color', colors(2,:), 'LineWidth', 0.8);
ylabel('D_{Q2}'); title('Buck 上管占空比 D_{Q2}'); grid on; xlabel('ms');
legend('原版','A1','Location','best');
ylim([-0.05 1.05]);

png = sprintf('%s/A1_compare.png', outdir);
exportgraphics(fig, png, 'Resolution', 150);
fprintf('\n对比波形: %s\n', png);

disp('=== A1 对比仿真完成 ===');
