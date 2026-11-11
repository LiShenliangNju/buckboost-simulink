%% ====================================================================
%  四开关 Buck-Boost 数字电源 —— P5 创新点 A2「真 0V 输出」对比仿真
%
%  针对源码问题（手册 §7 坑 3 / §8 A2）：
%    输出下限 0.5V，非真 0V。三层根因：
%     (1) 软件钳位：function.c:183-195 多处 `SET_Value.Vout < 0.5 -> = 0.5`
%         —— 用户旋钮设不了 0V（第一责任人，0.5V 就是它）；
%     (2) 占空比下限：PID.c:141 `MIN_BUKC_DUTY=100`（function.h:20）
%         —— Buck 上管最小占空 0.33%，即使 Vref=0 输出也残留 ~0.08V；
%     (3) 输出电容残压：Cout=690uF 空载无泄放路径，残压下不去。
%
%  A2 改进（down-programmer 有源下拉，三档分层展示）：
%    original   = 软件钳位 0.5V + 占空比下限 100 + 无下拉   -> 稳态 ~0.5V
%    A2_partial = 解软件钳位(Vref=0) + 下限仍 100 + 无下拉  -> 稳态 ~0.08V（暴露硬件下限）
%    A2_full    = 解钳位 + 下限 0（Buck 上管可完全关断）+ down-programmer
%                 （并联 Rdump 有源下拉，泄放输出电容）    -> 稳态 ~几 mV
%
%  短路判据（function.c:1287）是双条件 `Iout>10.1A && Vout<0.5V`，
%  0V 输出时 Iout≈0，天然不误触发 ShortOff —— 本脚本输出 f_short 验证恒 0。
%
%  场景：Vin=24V，负载 100Ω（轻载，突出 down-programmer 泄放 vs 负载自放电差异）
%        Vout 12V 稳态 -> 0.5s 起 30V/s 斜坡降到目标（original=0.5V / 其余=0V）
%        （下降斜率须受控，否则 2P2Z b1 负系数 bang-bang，实测 120V/s 下降触发 ±100A 尖峰）
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

Rload    = 100;     % 轻载 100Ω（12V 输出 0.12A；突出 down-programmer 价值）
Rdump    = 1.0;     % down-programmer 下拉电阻 [Ω]（0.5V 时 0.5A 泄放，τ=Rdump*Cout≈0.69ms）
Iref     = 10;      % 限流点 10A（低压场景远不激活）
Tsim     = 1.30;    % 仿真时长
t_drop_start = 0.50; % 下降斜坡起点
t_drop_end   = 0.90; % 下降斜坡终点（12V -> 0V）
vref_slew = 30;     % 参考下降斜率 [V/s]（坑15：Vref 变化过快会触发 2P2Z b1 负系数 bang-bang，下降 30V/s 安全）

outdir = 'E:/Desktop/大创/Simulink';
if ~exist(outdir,'dir'), mkdir(outdir); end

%% ================= 三档模式配置 =================
% 每档：vref_target 目标电压 / duty_buck_min Buck最小占空比 / dp_en 是否启用down-programmer
modes = struct();
modes(1).name = 'original';   modes(1).vref_target = 0.5; modes(1).duty_buck_min = 100/30000; modes(1).dp_en = 0;
modes(2).name = 'A2_partial'; modes(2).vref_target = 0.0; modes(2).duty_buck_min = 100/30000; modes(2).dp_en = 0;
modes(3).name = 'A2_full';    modes(3).vref_target = 0.0; modes(3).duty_buck_min = 0;        modes(3).dp_en = 1;

%% ================= 建模型（一次，三档复用） =================
model = 'buckboost_A2';
if bdIsLoaded(model), bdclose(model); end
new_system(model);
open_system(model);
load_system('sps_lib');

% 功率级（四开关 + 输入/输出电容 + 负载 + 故障支路（本场景不用））
add_block('sps_lib/Sources/DC Voltage Source', [model '/Vin'],  'Position',[40  160 80  205]);
add_block('sps_lib/Power Electronics/Mosfet',   [model '/Q2'],  'Position',[200 90  245 135]);
add_block('sps_lib/Power Electronics/Mosfet',   [model '/Q4'],  'Position',[200 210 245 255]);
add_block('sps_lib/Power Electronics/Mosfet',   [model '/Q3'],  'Position',[460 90  505 135]);
add_block('sps_lib/Power Electronics/Mosfet',   [model '/Q5'],  'Position',[460 210 505 255]);
add_block('sps_lib/Passives/Series RLC Branch', [model '/L'],   'Position',[330 130 375 175]);
add_block('sps_lib/Passives/Series RLC Branch', [model '/Cin'], 'Position',[120 300 160 345]);
add_block('sps_lib/Passives/Series RLC Branch', [model '/Cout'],'Position',[580 130 625 175]);
add_block('sps_lib/Passives/Series RLC Branch', [model '/Rload'],'Position',[580 300 625 345]);
% down-programmer 支路：Q_dp（MOS）+ Rdump（下拉电阻），并联在 Vout 节点到 GND
add_block('sps_lib/Power Electronics/Mosfet',   [model '/Q_dp'], 'Position',[660 300 705 345]);
add_block('sps_lib/Passives/Series RLC Branch', [model '/Rdump'],'Position',[740 300 785 345]);
add_block('sps_lib/powergui',                   [model '/powergui'],'Position',[40 400 120 445]);

% 测量
add_block('sps_lib/Sensors and Measurements/Voltage Measurement', [model '/Vm_in'], 'Position',[120 380 160 425]);
add_block('sps_lib/Sensors and Measurements/Current Measurement', [model '/Im_in'], 'Position',[80  160 120 205]);
add_block('sps_lib/Sensors and Measurements/Current Measurement', [model '/Im_L'],  'Position',[280 160 320 205]);
add_block('sps_lib/Sensors and Measurements/Voltage Measurement', [model '/Vm_out'],'Position',[640 380 680 425]);
add_block('sps_lib/Sensors and Measurements/Current Measurement', [model '/Im_out'],'Position',[640 160 680 205]);

% 控制器（5 输入 6 输出：D_Q2,D_Q5,mode,vlim,dp_on,f_short）
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/BB_LoopCtl'], 'Position',[820 90 910 270]);

% PWM 生成
add_block('simulink/Sources/Repeating Sequence', [model '/saw'], 'Position',[560 20 610 60]);
add_block('simulink/Math Operations/Sum', [model '/Sum_Q2'], 'Position',[840 20 870 50]);
add_block('simulink/Math Operations/Sum', [model '/Sum_Q5'], 'Position',[920 20 950 50]);
add_block('simulink/Sources/Constant', [model '/dz'], 'Position',[860 70 890 100]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q2'], 'Position',[880 20 910 60]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q4'], 'Position',[980 20 1010 60]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q5'], 'Position',[880 90 910 130]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q3'], 'Position',[980 90 1010 130]);

% 参考输入
add_block('simulink/Sources/From Workspace', [model '/Vref'], 'Position',[700 120 745 155]);
add_block('simulink/Sources/Constant', [model '/Iref'], 'Position',[700 220 740 255]);

% 记录
add_block('simulink/Sinks/To Workspace', [model '/Vout_ts'],'Position',[640 460 690 500]);
add_block('simulink/Sinks/To Workspace', [model '/Iout_ts'],'Position',[700 460 750 500]);
add_block('simulink/Sinks/To Workspace', [model '/Vref_ts'],'Position',[760 460 810 500]);
add_block('simulink/Sinks/To Workspace', [model '/DQ2_ts'], 'Position',[820 460 870 500]);
add_block('simulink/Sinks/To Workspace', [model '/DP_ts'],  'Position',[880 460 930 500]);
add_block('simulink/Sinks/To Workspace', [model '/FS_ts'],  'Position',[940 460 990 500]);

%% ================= 控制器模板（sprintf 注入三档差异） =================
tpl = [ ...
'function [D_Q2, D_Q5, mode_out, vlim_out, dp_on, f_short] = BB_LoopCtl(Vout, Iout, Vref_set, Iref_set, Vin)\n' ...
'%% 双环（原版电流环）+ 三模式 + 真0V down-programmer + OVP/ShortOff 兜底\n' ...
'persistent VErr1 VErr2 u1 Integral IErr1 BBFlag dp_state ovp_latch\n' ...
'if isempty(VErr1)\n' ...
'    VErr1 = 0; VErr2 = 0; u1 = 0; IErr1 = 0; BBFlag = 0; dp_state = 0; ovp_latch = 0;\n' ...
'    Integral = 0;\n' ...
'end\n' ...
'%% 定点常量\n' ...
'BUCK_b0 = 5271;  BUCK_b1 = -10363;  BUCK_b2 = 5093;\n' ...
'BOOST_b0 = 8044; BOOST_b1 = -15813; BOOST_b2 = 7772;\n' ...
'ILOOP_KP = 6; ILOOP_KI = 3; ILOOP_KD = 1;\n' ...
'LSB_PER_V = 155.5;  LSB_PER_A = 769.2;  V_PER_LSB = 1/155.5;\n' ...
'PERIOD = 30000;  ADC_MAX = 8190;\n' ...
'DUTY_BUCK_MIN = %g;                        %% A2_full 时 =0，允许 Buck 上管完全关断\n' ...
'DUTY_BUCK_MAX  = 28200/PERIOD;\n' ...
'DUTY_BOOST_MIN = 100/PERIOD;  DUTY_BOOST_MAX = 19500/PERIOD;\n' ...
'DUTY_BOOST_FIX = 1800/PERIOD; DUTY_BUCK_FIX = 28200/PERIOD; DUTY_MIX_FIX = 24000/PERIOD;\n' ...
'U0_BUCK_MAX  = 28200*256/3;  U0_BOOST_MAX = 19500*256/3;\n' ...
'DP_EN = %d;                                  %% down-programmer 使能（original/partial=0）\n' ...
'Ts_ctrl = 1/181333;\n' ...
'Vref_eff = Vref_set;                         %% 参考已含 120V/s 斜坡，无需额外软启动\n' ...
'%% ===== 原版电流环（量纲混用，低压场景不激活，仅保结构完整）=====\n' ...
'IErr_LSB = (Iref_set - Iout) * LSB_PER_A;\n' ...
'i0_LSB = Integral + ILOOP_KP*IErr_LSB + ILOOP_KD*(IErr_LSB - IErr1);\n' ...
'Integral = Integral + ILOOP_KI*IErr_LSB;\n' ...
'if Integral > ADC_MAX, Integral = ADC_MAX; end\n' ...
'if Integral < 0, Integral = 0; end\n' ...
'IErr1 = IErr_LSB;\n' ...
'i0_V = i0_LSB * V_PER_LSB;\n' ...
'Vref_ctrl = Vref_eff + i0_V;\n' ...
'if Vref_ctrl > Vref_eff, Vref_ctrl = Vref_eff; end\n' ...
'if Vref_ctrl < 0, Vref_ctrl = 0; end\n' ...
'Vlim = Vref_ctrl;\n' ...
'%% ===== down-programmer 门控（带滞回，只在低压段启用）=====\n' ...
'if Vref_ctrl < 1.0\n' ...
'    if Vout > Vref_ctrl + 0.02\n' ...
'        dp_state = 1;\n' ...
'    elseif Vout < Vref_ctrl + 0.005\n' ...
'        dp_state = 0;\n' ...
'    end\n' ...
'else\n' ...
'    dp_state = 0;\n' ...
'end\n' ...
'dp_on = dp_state * DP_EN;\n' ...
'%% ===== BBMode 迟滞判定（function.c:1463，场景 Buck 但保留完整逻辑）=====\n' ...
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
'%% ===== 保护兜底（OVP 锁死 / ShortOff 双条件，验证 0V 不误触发）=====\n' ...
'if Vout >= 50, ovp_latch = 1; end\n' ...
'f_short = 0;\n' ...
'if (Iout > 10.1) && (Vout < 0.5), f_short = 1; end   %% 0V 输出 Iout≈0，恒不触发\n' ...
'if ovp_latch == 1\n' ...
'    D_Q2 = 0; D_Q5 = 0;\n' ...
'end\n' ...
'mode_out = BBFlag;\n' ...
'vlim_out = Vlim;\n' ...
'end\n'];

rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','-and','Path',[model '/BB_LoopCtl']);
set_param([model '/BB_LoopCtl'], 'SystemSampleTime', num2str(Ts));
chart.Script = sprintf(tpl, modes(1).duty_buck_min, modes(1).dp_en);  % 先设一次生成 5入6出 端口

%% ---- 功率级参数 ----
set_param([model '/Vin'], 'Amplitude', num2str(Vin));
set_param([model '/Q2'], 'Ron', num2str(Rds_on));
set_param([model '/Q4'], 'Ron', num2str(Rds_on));
set_param([model '/Q3'], 'Ron', num2str(Rds_on));
set_param([model '/Q5'], 'Ron', num2str(Rds_on));
set_param([model '/Q_dp'], 'Ron', num2str(Rds_on));
set_param([model '/L'],  'BranchType','L', 'Inductance', num2str(L));
set_param([model '/Cin'],'BranchType','RC', 'Resistance','0.01', 'Capacitance', num2str(Cin), 'Setx0','on', 'InitialVoltage', num2str(Vin));
set_param([model '/Cout'],'BranchType','RC', 'Resistance','0.03', 'Capacitance', num2str(Cout), 'Setx0','on', 'InitialVoltage', num2str(0)); % 从0软启动，避免启动冲击（坑：直接置12V会与电感0电流不一致产生91A冲击）
set_param([model '/Rload'],'BranchType','R', 'Resistance', num2str(Rload));
set_param([model '/Rdump'],'BranchType','R', 'Resistance', num2str(Rdump));
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
for b = {'Vout_ts','Iout_ts','Vref_ts','DQ2_ts','DP_ts','FS_ts'}
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
add_line(model, 'Im_out/RConn1', 'Q_dp/LConn1', 'autorouting','on');     % down-programmer 接 Vout 节点
add_line(model, 'Q_dp/RConn1', 'Rdump/LConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Q4/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Q5/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Cin/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Cout/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Rload/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Rdump/RConn1', 'autorouting','on');
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
add_line(model, 'BB_LoopCtl/5', 'Q_dp/1', 'autorouting','on');      % dp_on -> Q_dp 门极
add_line(model, 'Vm_out/1', 'Vout_ts/1', 'autorouting','on');
add_line(model, 'Im_out/1', 'Iout_ts/1', 'autorouting','on');
add_line(model, 'Vref/1', 'Vref_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/1', 'DQ2_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/5', 'DP_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/6', 'FS_ts/1', 'autorouting','on');

% Solver
set_param(model, 'SolverType','Variable-step', 'Solver','ode23tb', ...
    'MaxStep','1e-7', 'RelTol','1e-4', 'AbsTol','1e-6');
set_param(model, 'ZeroCrossAlgorithm', 'Adaptive', 'IgnoredZcDiagnostic', 'none');

save_system(model, [outdir '/buckboost_A2.slx']);
disp('=== A2 模型已保存 ===');

%% ================= 逐档仿真 =================
results = struct();
for mi = 1:3
    c = modes(mi);
    fprintf('\n========== 档：%s ==========\n', c.name);

    % 控制器脚本注入本档参数
    chart.Script = sprintf(tpl, c.duty_buck_min, c.dp_en);

    % 参考时间序列：0->12V 软启动(120V/s) -> 12V 稳态 -> vref_slew 斜坡降到 target
    t_ref = (0:1e-3:Tsim)';
    vref = 12*ones(size(t_ref));
    seg_rise = t_ref < 0.1;                            % 软启动上升 0~0.1s（120V/s）
    vref(seg_rise) = 120*t_ref(seg_rise);
    seg = t_ref >= t_drop_start & t_ref < t_drop_end;  % 下降斜坡
    vref(seg) = 12 - vref_slew*(t_ref(seg) - t_drop_start);
    vref(t_ref >= t_drop_end) = c.vref_target;
    vref(vref < c.vref_target) = c.vref_target;        % 斜坡降到 target 即止
    vref_data = [t_ref, vref];
    set_param([model '/Vref'], 'VariableName', 'vref_data');
    set_param(model, 'StopTime', num2str(Tsim));

    simOut = sim(model);

    Vout_t = simOut.get('Vout_ts'); Iout_t = simOut.get('Iout_ts');
    DQ2_t  = simOut.get('DQ2_ts');  DP_t  = simOut.get('DP_ts');
    FS_t   = simOut.get('FS_ts');   Vref_t = simOut.get('Vref_ts');

    results(mi).name = c.name;
    results(mi).Vout = Vout_t;
    results(mi).Iout = Iout_t;
    results(mi).DQ2  = DQ2_t;
    results(mi).DP   = DP_t;
    results(mi).FS   = FS_t;
    results(mi).Vref = Vref_t;

    % 稳态指标（取末尾 0.2s 平均）
    t = Vout_t.Time; v = Vout_t.Data; i = Iout_t.Data;
    tail = t >= (Tsim - 0.2);
    v_ss = mean(v(tail));
    i_ss = mean(i(tail));
    % f_short 是否误触发
    fshort_max = max(FS_t.Data);
    fprintf('  Vout 稳态 = %.4f V（末尾0.2s均值）\n', v_ss);
    fprintf('  Vout 最低 = %.4f V\n', min(v(tail)));
    fprintf('  Iout 稳态 = %.4f A\n', i_ss);
    fprintf('  f_short 是否误触发 = %d（0=正常）\n', fshort_max);
    % down-programmer 是否动作（仅 A2_full）
    if c.dp_en
        dp_active = max(DP_t.Data) > 0.5;
        fprintf('  down-programmer 是否动作 = %d（1=已下拉）\n', dp_active);
    end
end

%% ================= 对比绘图 =================
fig = figure('Name','A2_compare','Visible','off','Position',[60 60 1120 900]);
colors = lines(3);
leg = {};

% 子图1：Vout 对比（对数 y 轴看低位差异）
subplot(4,1,1);
for mi = 1:3
    t = results(mi).Vout.Time*1e3; v = results(mi).Vout.Data;
    plot(t, v, 'Color', colors(mi,:), 'LineWidth', 1.2); hold on;
    leg{end+1} = results(mi).name;
end
ylabel('Vout (V)'); title('A2 创新点对比：输出电压 Vout（原版 0.5V vs 真 0V）'); grid on;
legend(leg, 'Location','northeast');
ylim([-0.1 13]);

% 子图2：Vout 低位放大（0~1V，看清 0.5 / 0.08 / ~0 三档）
subplot(4,1,2);
for mi = 1:3
    t = results(mi).Vout.Time*1e3; v = results(mi).Vout.Data;
    plot(t, v, 'Color', colors(mi,:), 'LineWidth', 1.2); hold on;
end
yline(0.5,'k:','0.5V 软件钳位'); yline(0.08,'m:','0.08V 占空比下限');
ylabel('Vout (V)'); title('低位放大（0~1V）：0.5V 软件钳位 / 0.08V 占空比下限 / ~0V down-programmer'); grid on;
ylim([-0.02 1.0]); xlabel('时间 (ms)');

% 子图3：Iout 对比
subplot(4,1,3);
for mi = 1:3
    t = results(mi).Iout.Time*1e3; i = results(mi).Iout.Data;
    plot(t, i, 'Color', colors(mi,:), 'LineWidth', 1.0); hold on;
end
ylabel('Iout (A)'); title('输出电流 Iout（down-programmer 下拉电流叠加在 A2_full）'); grid on;
legend(leg, 'Location','northeast');

% 子图4：down-programmer 门控 dp_on（仅 A2_full 有）+ 占空比 D_Q2
subplot(4,1,4);
yyaxis left;
t = results(3).DQ2.Time*1e3; d = results(3).DQ2.Data;
plot(t, d, 'Color', colors(3,:), 'LineWidth', 0.8);
ylabel('D_{Q2} (A2\_full)'); ylim([-0.02 0.6]);
yyaxis right;
t = results(3).DP.Time*1e3; dp = results(3).DP.Data;
plot(t, dp, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
ylabel('dp\_on 门控'); ylim([-0.1 1.2]);
title('A2\_full：Buck 上管占空比 D_{Q2}（降到 0 关断）+ down-programmer 门控 dp\_on'); grid on;
xlabel('时间 (ms)');

png = sprintf('%s/A2_compare.png', outdir);
exportgraphics(fig, png, 'Resolution', 150);
fprintf('\n对比波形: %s\n', png);

disp('=== A2 真 0V 输出对比仿真完成 ===');
