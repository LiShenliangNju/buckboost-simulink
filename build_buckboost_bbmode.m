%% ====================================================================
%  四开关 Buck-Boost 三模式自动切换仿真（BBMode 迟滞 + Mix 过渡区）
%  照搬 zeruns function.c:1463 BBMode() + PID.c 双环算法
%  场景：Vin=24V 固定，Vref 从 12V 斜坡扫到 36V（模拟真实旋钮调压）
%        穿越 Buck->Mix->Boost 边界，验证三模式自动切换 + 各段稳压
%  模式编码：0=NA, 1=Buck, 2=Boost, 3=Mix
%
%  【为什么用斜坡而非阶跃】 见建模踩坑记录：2P2Z 补偿器 b1≈-2b0，
%   Vref 阶跃 8V 会让 VErr 跳变 ~1244LSB，b1*VErr1 反向冲击把 u0 打满/打零，
%  占空比 bang-bang 极限环（Vout 甚至冲到 52V / 掉到 -0.3V）。真实硬件 Vref
%  由旋钮/编码器渐变，不存在阶跃工况，故用斜坡扫掠才是忠实复现。
% =====================================================================
close all;

%% ================= 参数（取自 README / 复现手册 / PID.c / function.h） =================
Vin      = 24;      % 输入电压 [V]（理想源，恒定）
L        = 22e-6;   % 电感 22uH
Cin      = 220e-6;  % 输入电容 220uF
Cout     = 690e-6;  % 输出电容 220uF + 470uF
Rds_on   = 6.2e-3;  % MOS 导通电阻 CJAC80SN10 = 6.2mOhm
fsw      = 181333;  % 开关频率 = 实物 181.333kHz（2P2Z 系数按此采样率整定，必须对齐）
Ts       = 1/fsw;   % 采样周期 = 开关周期
deadtime = 0.2e-6;  % 死区 200ns
dz       = deadtime/Ts;

% Vref 斜坡扫掠：12V -> 36V，模拟真实旋钮缓慢调压
% 电压外环带宽~42Hz(时间常数~3.8ms)，斜坡斜率 120V/s 时稳态跟踪误差仅 ~0.45V(≈71LSB)，
% 远小于阶跃冲击，2P2Z 不会进入极限环。
Vref_start = 12;   % 起始（Buck 起步，Cout 从 0 软启动拉起）
Vref_end   = 36;   % 终点（Boost）
t_soft     = 20e-3;   % 前 20ms 保持 12V，让 Buck 软启动充分建立
t_ramp     = 200e-3;  % 斜坡时长 200ms（12->36，斜率 120V/s）
Tsim       = t_soft + t_ramp;
slope      = (Vref_end - Vref_start)/t_ramp;

% From Workspace 斜坡数据（1ms 采样，线性插值）
t_ref   = (0:1e-3:Tsim)';
vref_seq= Vref_start*ones(size(t_ref));
idx_ramp = t_ref > t_soft;
vref_seq(idx_ramp) = Vref_start + slope*(t_ref(idx_ramp)-t_soft);
vref_seq(vref_seq > Vref_end) = Vref_end;
vref_data = [t_ref, vref_seq];

% 理论切换点（迟滞：Buck->Mix @0.85Vin=20.4V，Mix->Boost @1.2Vin=28.8V）
t_b2m = t_soft + (0.85*Vin - Vref_start)/slope;   % Buck->Mix 理论时刻
t_m2b = t_soft + (1.20*Vin - Vref_start)/slope;   % Mix->Boost 理论时刻

Iref_max= 10;      % 电流环参考设大值，恒压环主导（不触发限流）
Rload    = 18;     % 负载 18ohm（Boost 段 36V/18ohm=2A；其余段电流更小）

%% ================= 建模型 =================
model = 'buckboost_bbmode';
if bdIsLoaded(model), bdclose(model); end
new_system(model);
open_system(model);
load_system('powerlib');

% ---- 功率级块（与闭环拓扑一致）----
add_block('powerlib/Electrical Sources/DC Voltage Source', [model '/Vin'],  'Position',[40  160 80  205]);
add_block('powerlib/Power Electronics/Mosfet',                 [model '/Q2'],  'Position',[200 90  245 135]);
add_block('powerlib/Power Electronics/Mosfet',                 [model '/Q4'],  'Position',[200 210 245 255]);
add_block('powerlib/Power Electronics/Mosfet',                 [model '/Q3'],  'Position',[460 90  505 135]);
add_block('powerlib/Power Electronics/Mosfet',                 [model '/Q5'],  'Position',[460 210 505 255]);
add_block('powerlib/Elements/Series RLC Branch',               [model '/L'],   'Position',[330 130 375 175]);
add_block('powerlib/Elements/Series RLC Branch',               [model '/Cin'], 'Position',[120 300 160 345]);
add_block('powerlib/Elements/Series RLC Branch',               [model '/Cout'],'Position',[580 130 625 175]);
add_block('powerlib/Elements/Series RLC Branch',               [model '/Rload'],'Position',[580 300 625 345]);
add_block('powerlib/powergui',                                 [model '/powergui'],'Position',[40 400 120 445]);

% ---- 测量块 ----
add_block('powerlib/Measurements/Voltage Measurement', [model '/Vm_in'], 'Position',[120 380 160 425]);
add_block('powerlib/Measurements/Current Measurement', [model '/Im_in'], 'Position',[80  160 120 205]);
add_block('powerlib/Measurements/Current Measurement', [model '/Im_L'],  'Position',[280 160 320 205]);
add_block('powerlib/Measurements/Voltage Measurement', [model '/Vm_out'],'Position',[640 380 680 425]);
add_block('powerlib/Measurements/Current Measurement', [model '/Im_out'],'Position',[640 160 680 205]);

% ---- 控制器（MATLAB Function，照搬 PID.c + BBMode）----
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/BB_LoopCtl'], 'Position',[820 90 900 230]);

% ---- PWM 生成（锯齿波载波 + 比较 + 死区）----
add_block('simulink/Sources/Repeating Sequence', [model '/saw'], 'Position',[560 20 610 60]);
add_block('simulink/Math Operations/Sum', [model '/Sum_Q2'], 'Position',[820 20 850 50]);
add_block('simulink/Math Operations/Sum', [model '/Sum_Q5'], 'Position',[900 20 930 50]);
add_block('simulink/Sources/Constant', [model '/dz'], 'Position',[840 70 870 100]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q2'], 'Position',[860 20 890 60]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q4'], 'Position',[950 20 980 60]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q5'], 'Position',[860 90 890 130]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q3'], 'Position',[950 90 980 130]);

% ---- 参考输入（Vref 用 From Workspace 斜坡，Iref 恒值）----
add_block('simulink/Sources/From Workspace', [model '/Vref'], 'Position',[700 120 745 155]);
add_block('simulink/Sources/Constant', [model '/Iref'], 'Position',[700 170 740 205]);

% ---- 记录块 ----
add_block('simulink/Sinks/To Workspace', [model '/Vin_ts'], 'Position',[120 450 170 490]);
add_block('simulink/Sinks/To Workspace', [model '/Iin_ts'], 'Position',[80  450 130 490]);
add_block('simulink/Sinks/To Workspace', [model '/IL_ts'],  'Position',[280 450 330 490]);
add_block('simulink/Sinks/To Workspace', [model '/Vout_ts'],'Position',[640 450 690 490]);
add_block('simulink/Sinks/To Workspace', [model '/Iout_ts'],'Position',[700 450 750 490]);
add_block('simulink/Sinks/To Workspace', [model '/Vref_ts'],'Position',[760 450 810 490]);
add_block('simulink/Sinks/To Workspace', [model '/DQ2_ts'], 'Position',[880 450 930 490]);
add_block('simulink/Sinks/To Workspace', [model '/DQ5_ts'], 'Position',[940 450 990 490]);
add_block('simulink/Sinks/To Workspace', [model '/mode_ts'],'Position',[1000 450 1050 490]);

%% ---- 控制器 MATLAB Function 代码（照搬 PID.c + function.c:1463）----
script = sprintf([ ...
'function [D_Q2, D_Q5, mode_out] = BB_LoopCtl(Vout, Iout, Vref_set, Iref_set, Vin)\n' ...
'%% 四开关 Buck-Boost 双环控制器 + BBMode 迟滞切换（照搬 zeruns）\n' ...
'persistent VErr1 VErr2 u1 Integral IErr1 ramp BBFlag softstart_done\n' ...
'if isempty(VErr1)\n' ...
'    VErr1 = 0; VErr2 = 0; u1 = 0; Integral = 0; IErr1 = 0;\n' ...
'    ramp = 0; BBFlag = 0; softstart_done = 0;   %% BBFlag: 0=NA,1=Buck,2=Boost,3=Mix\n' ...
'end\n' ...
'%% 原版定点常量（PID.c / function.h）\n' ...
'BUCK_b0 = 5271;  BUCK_b1 = -10363;  BUCK_b2 = 5093;\n' ...
'BOOST_b0 = 8044; BOOST_b1 = -15813; BOOST_b2 = 7772;\n' ...
'ILOOP_KP = 6; ILOOP_KI = 3; ILOOP_KD = 1;\n' ...
'LSB_PER_V = 155.5;  LSB_PER_A = 769.2;  V_PER_LSB = 1/155.5;\n' ...
'PERIOD = 30000;  ADC_MAX = 8190;\n' ...
'DUTY_BUCK_MAX  = 28200/PERIOD;  DUTY_BUCK_MIN  = 100/PERIOD;\n' ...
'DUTY_BOOST_MAX = 19500/PERIOD;  DUTY_BOOST_MIN = 100/PERIOD;\n' ...
'DUTY_BOOST_FIX = 1800/PERIOD;   %% MIN_BOOST_DUTY1：Buck模式 Boost下管固定6%%\n' ...
'DUTY_BUCK_FIX  = 28200/PERIOD;  %% MAX_BUCK_DUTY：Boost模式 Buck上管固定94%%\n' ...
'DUTY_MIX_FIX   = 24000/PERIOD;  %% MAX_BUCK_DUTY1：Mix模式 Buck上管固定80%%\n' ...
'U0_BUCK_MAX  = 28200*256/3;  U0_BOOST_MAX = 19500*256/3;  %% anti-windup\n' ...
'%% ===== 1. BBMode 迟滞判定（照搬 function.c:1463，用 Vref_set 近似 Vout_ref）=====\n' ...
'PreBBFlag = BBFlag;\n' ...
'switch BBFlag\n' ...
'    case 0   %% NA\n' ...
'        if Vref_set < 0.8*Vin, BBFlag = 1;\n' ...
'        elseif Vref_set > 1.2*Vin, BBFlag = 2;\n' ...
'        else, BBFlag = 3; end\n' ...
'    case 1   %% Buck\n' ...
'        if Vref_set > 1.2*Vin, BBFlag = 2;\n' ...
'        elseif Vref_set > 0.85*Vin, BBFlag = 3; end\n' ...
'    case 2   %% Boost\n' ...
'        if Vref_set < 0.8*Vin, BBFlag = 1;\n' ...
'        elseif Vref_set < 1.15*Vin, BBFlag = 3; end\n' ...
'    case 3   %% Mix\n' ...
'        if Vref_set < 0.8*Vin, BBFlag = 1;\n' ...
'        elseif Vref_set > 1.2*Vin, BBFlag = 2; end\n' ...
'end\n' ...
'%% ===== 2. 软启动 ramp（初始从0升到起始Vref，之后Vref斜坡自然跟随）=====\n' ...
'if ~softstart_done\n' ...
'    ramp = ramp + Vref_set * 0.0011;\n' ...
'    if ramp >= Vref_set, ramp = Vref_set; softstart_done = 1; end\n' ...
'end\n' ...
'if softstart_done, Vref_eff = Vref_set; else, Vref_eff = ramp; end\n' ...
'%% ===== 3. 电流环（增量式 PID，内环）=====\n' ...
'IErr_LSB = (Iref_set - Iout) * LSB_PER_A;\n' ...
'i0_LSB = Integral + ILOOP_KP*IErr_LSB + ILOOP_KD*(IErr_LSB - IErr1);\n' ...
'Integral = Integral + ILOOP_KI*IErr_LSB;\n' ...
'if Integral > ADC_MAX, Integral = ADC_MAX; end\n' ...
'if Integral < 0, Integral = 0; end\n' ...
'IErr1 = IErr_LSB;\n' ...
'i0_V = i0_LSB * V_PER_LSB;\n' ...
'%% CV/CC：电流环输出叠加到电压参考并限幅（恒流时把参考往下拉）\n' ...
'Vref_ctrl = Vref_eff + i0_V;\n' ...
'if Vref_ctrl > Vref_eff, Vref_ctrl = Vref_eff; end\n' ...
'if Vref_ctrl < 0, Vref_ctrl = 0; end\n' ...
'%% 模式切换清零积分项（PID.c:102-108，电流环之后、电压环之前）\n' ...
'if BBFlag ~= PreBBFlag\n' ...
'    u1 = 0; Integral = 0;\n' ...
'end\n' ...
'%% ===== 4. 电压环（2P2Z 二阶差分，外环，按 BBFlag 选系数）=====\n' ...
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
'else   %% Mix（用 Boost 系数，Q2 固定 80%%）\n' ...
'    u0 = u1 + BOOST_b0*VErr_LSB + BOOST_b1*VErr1 + BOOST_b2*VErr2;\n' ...
'    if u0 > U0_BOOST_MAX, u0 = U0_BOOST_MAX; end\n' ...
'    if u0 < 0, u0 = 0; end\n' ...
'    VErr2 = VErr1; VErr1 = VErr_LSB; u1 = u0;\n' ...
'    D_Q2 = DUTY_MIX_FIX;  D_Q5 = floor(u0/256)*3/PERIOD;\n' ...
'    if D_Q5 > DUTY_BOOST_MAX, D_Q5 = DUTY_BOOST_MAX; end\n' ...
'    if D_Q5 < DUTY_BOOST_MIN, D_Q5 = DUTY_BOOST_MIN; end\n' ...
'end\n' ...
'mode_out = BBFlag;\n' ...
'end\n']);
rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','-and','Path',[model '/BB_LoopCtl']);
chart.Script = script;
set_param([model '/BB_LoopCtl'], 'SystemSampleTime', num2str(Ts));

%% ---- 功率级参数 ----
set_param([model '/Vin'], 'Amplitude', num2str(Vin));
set_param([model '/Q2'], 'Ron', num2str(Rds_on));
set_param([model '/Q4'], 'Ron', num2str(Rds_on));
set_param([model '/Q3'], 'Ron', num2str(Rds_on));
set_param([model '/Q5'], 'Ron', num2str(Rds_on));
set_param([model '/L'],  'BranchType','L', 'Inductance', num2str(L));
set_param([model '/Cin'],'BranchType','RC', 'Resistance','0.01', 'Capacitance', num2str(Cin), 'Setx0','on', 'InitialVoltage', num2str(Vin));
set_param([model '/Cout'],'BranchType','RC', 'Resistance','0.03', 'Capacitance', num2str(Cout), 'Setx0','on', 'InitialVoltage','0');
set_param([model '/Rload'],'BranchType','R', 'Resistance', num2str(Rload));

% ---- PWM 生成参数 ----
set_param([model '/saw'], 'rep_seq_t', sprintf('[0 %s]', num2str(Ts)), 'rep_seq_y', '[0 1]');
set_param([model '/dz'], 'Value', num2str(dz));
set_param([model '/Sum_Q2'], 'Inputs', '++');
set_param([model '/Sum_Q5'], 'Inputs', '++');
set_param([model '/Rel_Q2'], 'Operator', '<');
set_param([model '/Rel_Q4'], 'Operator', '>');
set_param([model '/Rel_Q5'], 'Operator', '<');
set_param([model '/Rel_Q3'], 'Operator', '>');

% ---- 参考输入 ----
set_param([model '/Vref'], 'VariableName', 'vref_data');
set_param([model '/Iref'], 'Value', num2str(Iref_max));

% ---- 记录块 ----
for b = {'Vin_ts','Iin_ts','IL_ts','Vout_ts','Iout_ts','Vref_ts','DQ2_ts','DQ5_ts','mode_ts'}
    set_param([model '/' b{1}], 'SaveFormat','Timeseries', 'VariableName', b{1});
end

%% ---- 功率电路连线（物理网络，与闭环一致）----
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
add_line(model, 'Vin/LConn1', 'Q4/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Q5/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Cin/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Cout/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Rload/RConn1', 'autorouting','on');
add_line(model, 'Vm_in/LConn1', 'Im_in/RConn1', 'autorouting','on');
add_line(model, 'Vm_in/LConn2', 'Vin/LConn1', 'autorouting','on');
add_line(model, 'Vm_out/LConn1', 'Im_out/RConn1', 'autorouting','on');
add_line(model, 'Vm_out/LConn2', 'Vin/LConn1', 'autorouting','on');

%% ---- 信号连线 ----
% 反馈：Vout/Iout/Vref/Iref/Vin -> 控制器
add_line(model, 'Vm_out/1', 'BB_LoopCtl/1', 'autorouting','on');   % Vout
add_line(model, 'Im_out/1', 'BB_LoopCtl/2', 'autorouting','on');   % Iout
add_line(model, 'Vref/1', 'BB_LoopCtl/3', 'autorouting','on');     % Vref_set
add_line(model, 'Iref/1', 'BB_LoopCtl/4', 'autorouting','on');     % Iref_set
add_line(model, 'Vm_in/1', 'BB_LoopCtl/5', 'autorouting','on');    % Vin
% 占空比 -> PWM 生成
add_line(model, 'BB_LoopCtl/1', 'Sum_Q2/1', 'autorouting','on');   % D_Q2
add_line(model, 'BB_LoopCtl/2', 'Sum_Q5/1', 'autorouting','on');   % D_Q5
add_line(model, 'dz/1', 'Sum_Q2/2', 'autorouting','on');
add_line(model, 'dz/1', 'Sum_Q5/2', 'autorouting','on');
% 载波 -> 比较器
add_line(model, 'saw/1', 'Rel_Q2/1', 'autorouting','on');
add_line(model, 'saw/1', 'Rel_Q4/1', 'autorouting','on');
add_line(model, 'saw/1', 'Rel_Q5/1', 'autorouting','on');
add_line(model, 'saw/1', 'Rel_Q3/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/1', 'Rel_Q2/2', 'autorouting','on');   % saw < D_Q2
add_line(model, 'Sum_Q2/1', 'Rel_Q4/2', 'autorouting','on');       % saw > D_Q2+dz
add_line(model, 'BB_LoopCtl/2', 'Rel_Q5/2', 'autorouting','on');   % saw < D_Q5
add_line(model, 'Sum_Q5/1', 'Rel_Q3/2', 'autorouting','on');       % saw > D_Q5+dz
% PWM -> MOS 门极
add_line(model, 'Rel_Q2/1', 'Q2/1', 'autorouting','on');
add_line(model, 'Rel_Q4/1', 'Q4/1', 'autorouting','on');
add_line(model, 'Rel_Q5/1', 'Q5/1', 'autorouting','on');
add_line(model, 'Rel_Q3/1', 'Q3/1', 'autorouting','on');
% 记录
add_line(model, 'Vm_in/1', 'Vin_ts/1', 'autorouting','on');
add_line(model, 'Im_in/1', 'Iin_ts/1', 'autorouting','on');
add_line(model, 'Im_L/1', 'IL_ts/1', 'autorouting','on');
add_line(model, 'Vm_out/1', 'Vout_ts/1', 'autorouting','on');
add_line(model, 'Im_out/1', 'Iout_ts/1', 'autorouting','on');
add_line(model, 'Vref/1', 'Vref_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/1', 'DQ2_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/2', 'DQ5_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/3', 'mode_ts/1', 'autorouting','on');

%% Solver 配置
set_param(model, 'SolverType','Variable-step', 'Solver','ode23tb', ...
    'MaxStep','1e-7', 'RelTol','1e-4', 'AbsTol','1e-6', ...
    'StopTime', num2str(Tsim));

outdir = 'C:/Users/ll/Desktop/大创/Simulink';
if ~exist(outdir,'dir'), mkdir(outdir); end
save_system(model, [outdir '/buckboost_bbmode.slx']);
disp('=== 模型已保存，开始三模式斜坡扫掠仿真 ===');

%% ================= 仿真 =================
simOut = sim(model);
Vin_t  = simOut.get('Vin_ts');  Iin_t = simOut.get('Iin_ts');  IL_t  = simOut.get('IL_ts');
Vout_t = simOut.get('Vout_ts'); Iout_t= simOut.get('Iout_ts'); Vref_t= simOut.get('Vref_ts');
DQ2_t  = simOut.get('DQ2_ts');  DQ5_t = simOut.get('DQ5_ts');  mode_t= simOut.get('mode_ts');

%% ================= 结果分析 =================
t = Vout_t.Time;
md = round(mode_t.Data);
mode_label = {'NA','Buck','Boost','Mix'};

% 模式切换时刻（从 mode 数据检测实际跳变沿）
md_t = mode_t.Time;
chg = find(diff(md) ~= 0);
fprintf('\n================ 三模式斜坡扫掠仿真结果 ================\n');
fprintf('Vref: %.0fV -> %.0fV (斜率 %.0fV/s)\n', Vref_start, Vref_end, slope);
fprintf('理论切换点: Buck->Mix @%.1fms (%.1fV), Mix->Boost @%.1fms (%.1fV)\n', ...
    t_b2m*1e3, 0.85*Vin, t_m2b*1e3, 1.20*Vin);
fprintf('--- 实际模式切换序列 ---\n');
for i = 1:numel(chg)
    fprintf('  t=%.2fms : %s -> %s\n', md_t(chg(i))*1e3, ...
        mode_label{md(chg(i))+1}, mode_label{md(chg(i)+1)+1});
end

% 按模式分段统计跟踪误差（Vout vs Vref_set）
fprintf('\n--- 各模式跟踪性能（Vout vs Vref 瞬时目标）---\n');
fprintf('%-6s %10s %10s %10s\n', '模式', '平均Vout', '平均Vref', '跟踪误差');
for m = [1 3 2]   % Buck, Mix, Boost
    seg = md == m;
    if ~any(seg), continue; end
    % 取该模式段的后 60%（跳过切换瞬态）
    seg_t = md_t(seg);
    seg_lo = seg_t(round(numel(seg_t)*0.4));
    seg_hi = seg_t(end);
    idx = (t >= seg_lo) & (t <= seg_hi);
    if ~any(idx), continue; end
    vout_m = mean(Vout_t.Data(idx));
    vref_m = mean(Vref_t.Data(idx));
    err = vout_m - vref_m;
    rip = std(Vout_t.Data(idx))*1000;
    fprintf('%-6s %10.3f %10.3f %10.4f  (纹波%.1fmV)\n', ...
        mode_label{m+1}, vout_m, vref_m, err, rip);
end
fprintf('==================================================\n');

%% ================= 绘图 =================
fig = figure('Name','三模式斜坡扫掠 Buck->Mix->Boost','Visible','off','Position',[60 60 980 760]);

subplot(5,1,1);
plot(t*1e3, Vout_t.Data, 'b', 'LineWidth', 1); hold on;
plot(Vref_t.Time*1e3, Vref_t.Data, 'r--', 'LineWidth', 1.2);
xline(t_b2m*1e3, 'k:', 'LineWidth', 0.7); xline(t_m2b*1e3, 'k:', 'LineWidth', 0.7);
title('输出电压 Vout vs 设定 Vref（斜坡扫掠 Buck→Mix→Boost）');
ylabel('V'); xlabel('ms'); grid on; legend('Vout','Vref');

subplot(5,1,2);
stairs(md_t*1e3, md, 'm', 'LineWidth', 1.4);
yticks([0 1 2 3]); yticklabels({'NA','Buck','Boost','Mix'}); ylim([0.5 3.5]);
title('工作模式 BBFlag（迟滞切换）'); ylabel('模式'); xlabel('ms'); grid on;

subplot(5,1,3);
plot(t*1e3, IL_t.Data, 'g', 'LineWidth', 0.8);
title('电感电流 IL'); ylabel('A'); xlabel('ms'); grid on;

subplot(5,1,4);
plot(DQ2_t.Time*1e3, DQ2_t.Data, 'b', 'LineWidth', 0.8); hold on;
plot(DQ5_t.Time*1e3, DQ5_t.Data, 'r', 'LineWidth', 0.8);
title('占空比 D_{Q2}(Buck上管) / D_{Q5}(Boost下管)'); ylabel('D'); xlabel('ms'); grid on;
legend('D Q2','D Q5'); ylim([-0.05 1.05]);

subplot(5,1,5);
plot(t*1e3, Iout_t.Data, 'c', 'LineWidth', 0.8);
title('输出电流 Iout'); ylabel('A'); xlabel('ms'); grid on;

exportgraphics(fig, [outdir '/buckboost_bbmode_sweep.png'], 'Resolution', 150);
disp(['扫掠波形: ' outdir '/buckboost_bbmode_sweep.png']);
