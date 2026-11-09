%% ====================================================================
%  四开关 Buck-Boost 闭环 PID 仿真（照搬 zeruns PID.c 双环算法）
%  电压环 = 2P2Z 二阶差分补偿器（b0/b1/b2）
%  电流环 = 增量式 PID（KP/KI/KD）
%  双环耦合 = CV/CC 限幅切换，三模式占空比分配 + 软启动
%  量纲换算（精确）：1V=155.5 LSB, 1A=769.2 LSB（13位过采样，满量程8190）
%  验证：Buck(24V->12V) 和 Boost(24V->36V) 恒压闭环，从 0 软启动拉起并稳压
% =====================================================================
close all;

%% ================= 参数（取自 README / 复现手册 / PID.c） =================
Vin      = 24;      % 输入电压 [V]
L        = 22e-6;   % 电感 22uH
Cin      = 220e-6;  % 输入电容 220uF
Cout     = 690e-6;  % 输出电容 220uF + 470uF
Rds_on   = 6.2e-3;  % MOS 导通电阻 CJAC80SN10 = 6.2mOhm
fsw      = 181333;  % 开关频率 = 实物 181.333kHz（2P2Z 系数按此采样率整定，必须对齐）
Ts       = 1/fsw;   % 采样周期 = 开关周期（数字控制，每周期采一次）
deadtime = 0.2e-6;  % 死区 200ns
dz       = deadtime/Ts;   % 死区占载波比例
Tsim     = 50e-3;   % 仿真 50ms（2P2Z 电压外环带宽~42Hz，时间常数~24ms，需充分收敛）

% 两个恒压场景
Vref_buck  = 12;  Rload_buck  = 6;    % Buck: 24V->12V, 2A
Vref_boost = 36;  Rload_boost = 18;   % Boost: 24V->36V, 2A
Iref_max   = 10;  % 电流环参考设大值，让恒压环主导（不触发限流）

%% ================= 建模型 =================
model = 'buckboost_closedloop';
if bdIsLoaded(model), bdclose(model); end
new_system(model);
open_system(model);
load_system('powerlib');

% ---- 功率级块（与开环拓扑一致）----
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

% ---- 控制器（MATLAB Function，照搬 PID.c）----
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/BB_LoopCtl'], 'Position',[820 90 900 210]);

% ---- PWM 生成（锯齿波载波 + 比较 + 死区）----
add_block('simulink/Sources/Repeating Sequence', [model '/saw'], 'Position',[560 20 610 60]);
add_block('simulink/Math Operations/Sum', [model '/Sum_Q2'], 'Position',[820 20 850 50]);
add_block('simulink/Math Operations/Sum', [model '/Sum_Q5'], 'Position',[900 20 930 50]);
add_block('simulink/Sources/Constant', [model '/dz'], 'Position',[840 70 870 100]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q2'], 'Position',[860 20 890 60]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q4'], 'Position',[950 20 980 60]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q5'], 'Position',[860 90 890 130]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [model '/Rel_Q3'], 'Position',[950 90 980 130]);

% ---- 参考输入 ----
add_block('simulink/Sources/Constant', [model '/Vref'], 'Position',[700 120 740 155]);
add_block('simulink/Sources/Constant', [model '/Iref'], 'Position',[700 170 740 205]);
add_block('simulink/Sources/Constant', [model '/mode'], 'Position',[700 220 740 255]);

% ---- 记录块 ----
add_block('simulink/Sinks/To Workspace', [model '/Vin_ts'], 'Position',[120 450 170 490]);
add_block('simulink/Sinks/To Workspace', [model '/Iin_ts'], 'Position',[80  450 130 490]);
add_block('simulink/Sinks/To Workspace', [model '/IL_ts'],  'Position',[280 450 330 490]);
add_block('simulink/Sinks/To Workspace', [model '/Vout_ts'],'Position',[640 450 690 490]);
add_block('simulink/Sinks/To Workspace', [model '/Iout_ts'],'Position',[700 450 750 490]);
add_block('simulink/Sinks/To Workspace', [model '/DQ2_ts'], 'Position',[880 450 930 490]);
add_block('simulink/Sinks/To Workspace', [model '/DQ5_ts'], 'Position',[940 450 990 490]);

%% ---- 控制器 MATLAB Function 代码（照搬 PID.c）----
script = sprintf(['function [D_Q2, D_Q5] = BB_LoopCtl(Vout, Iout, Vref_set, Iref_set, mode)\n' ...
'%% 四开关 Buck-Boost 双环控制器（照搬 zeruns PID.c BuckBoostVILoopCtlPID）\n' ...
'persistent VErr1 VErr2 u1 Integral IErr1 ramp\n' ...
'if isempty(VErr1)\n' ...
'    VErr1 = 0; VErr2 = 0; u1 = 0; Integral = 0; IErr1 = 0;\n' ...
'    if mode == 2, ramp = 24; else, ramp = 0; end\n' ...
'end\n' ...
'%% 原版定点常量（对照 PID.c / function.h）\n' ...
'BUCK_b0 = 5271;  BUCK_b1 = -10363;  BUCK_b2 = 5093;\n' ...
'BOOST_b0 = 8044; BOOST_b1 = -15813; BOOST_b2 = 7772;\n' ...
'ILOOP_KP = 6; ILOOP_KI = 3; ILOOP_KD = 1;\n' ...
'%% 量纲换算（对照 function.c:1035-1036）\n' ...
'LSB_PER_V = 155.5;   %% 1V -> 155.5 LSB（13位过采样）\n' ...
'LSB_PER_A = 769.2;   %% 1A -> 769.2 LSB\n' ...
'V_PER_LSB = 1/155.5;\n' ...
'PERIOD = 30000;\n' ...
'ADC_MAX = 8190;\n' ...
'DUTY_BUCK_MAX  = 28200/PERIOD;   %% 0.94\n' ...
'DUTY_BUCK_MIN  = 100/PERIOD;\n' ...
'DUTY_BOOST_MAX = 19500/PERIOD;   %% 0.65\n' ...
'DUTY_BOOST_MIN = 100/PERIOD;\n' ...
'DUTY_BOOST_FIX = 1800/PERIOD;    %% 0.06（Buck模式 Boost下管固定）\n' ...
'DUTY_BUCK_FIX  = 28200/PERIOD;   %% 0.94（Boost模式 Buck上管固定）\n' ...
'U0_BUCK_MAX  = 28200*256/3;      %% 2406400，anti-windup：u0 上限\n' ...
'U0_BOOST_MAX = 19500*256/3;      %% 1664000，anti-windup：u0 上限\n' ...
'%% 软启动斜坡：5ms 内从 0 升到设定值（Ts=5.5us，约900步）\n' ...
'if ramp < Vref_set\n' ...
'    ramp = ramp + Vref_set * 0.0011;\n' ...
'    if ramp > Vref_set, ramp = Vref_set; end\n' ...
'end\n' ...
'%% 电流环（增量式 PID，内环）\n' ...
'IErr_LSB = (Iref_set - Iout) * LSB_PER_A;\n' ...
'i0_LSB = Integral + ILOOP_KP*IErr_LSB + ILOOP_KD*(IErr_LSB - IErr1);\n' ...
'Integral = Integral + ILOOP_KI*IErr_LSB;\n' ...
'if Integral > ADC_MAX, Integral = ADC_MAX; end\n' ...
'if Integral < 0, Integral = 0; end\n' ...
'IErr1 = IErr_LSB;\n' ...
'i0_V = i0_LSB * V_PER_LSB;\n' ...
'%% CV/CC：电流环输出叠加到电压参考并限幅\n' ...
'Vref_eff = ramp + i0_V;\n' ...
'if Vref_eff > ramp, Vref_eff = ramp; end\n' ...
'if Vref_eff < 0, Vref_eff = 0; end\n' ...
'%% 电压环（2P2Z 二阶差分，外环）\n' ...
'VErr_LSB = (Vref_eff - Vout) * LSB_PER_V;\n' ...
'if mode == 1   %% Buck\n' ...
'    u0 = u1 + BUCK_b0*VErr_LSB + BUCK_b1*VErr1 + BUCK_b2*VErr2;\n' ...
'    if u0 > U0_BUCK_MAX, u0 = U0_BUCK_MAX; end\n' ...
'    if u0 < 0, u0 = 0; end\n' ...
'    VErr2 = VErr1; VErr1 = VErr_LSB; u1 = u0;\n' ...
'    D_Q2 = floor(u0/256) * 3 / PERIOD;   %% (u0>>8)*3 / PERIOD\n' ...
'    D_Q5 = DUTY_BOOST_FIX;\n' ...
'    if D_Q2 > DUTY_BUCK_MAX, D_Q2 = DUTY_BUCK_MAX; end\n' ...
'    if D_Q2 < DUTY_BUCK_MIN, D_Q2 = DUTY_BUCK_MIN; end\n' ...
'elseif mode == 2   %% Boost\n' ...
'    u0 = u1 + BOOST_b0*VErr_LSB + BOOST_b1*VErr1 + BOOST_b2*VErr2;\n' ...
'    if u0 > U0_BOOST_MAX, u0 = U0_BOOST_MAX; end\n' ...
'    if u0 < 0, u0 = 0; end\n' ...
'    VErr2 = VErr1; VErr1 = VErr_LSB; u1 = u0;\n' ...
'    D_Q2 = DUTY_BUCK_FIX;\n' ...
'    D_Q5 = floor(u0/256) * 3 / PERIOD;\n' ...
'    if D_Q5 > DUTY_BOOST_MAX, D_Q5 = DUTY_BOOST_MAX; end\n' ...
'    if D_Q5 < DUTY_BOOST_MIN, D_Q5 = DUTY_BOOST_MIN; end\n' ...
'else   %% Mix（暂按 Boost 处理）\n' ...
'    u0 = u1 + BOOST_b0*VErr_LSB + BOOST_b1*VErr1 + BOOST_b2*VErr2;\n' ...
'    VErr2 = VErr1; VErr1 = VErr_LSB; u1 = u0;\n' ...
'    D_Q2 = 24000/PERIOD;\n' ...
'    D_Q5 = floor(u0/256) * 3 / PERIOD;\n' ...
'    if D_Q5 > DUTY_BOOST_MAX, D_Q5 = DUTY_BOOST_MAX; end\n' ...
'    if D_Q5 < DUTY_BOOST_MIN, D_Q5 = DUTY_BOOST_MIN; end\n' ...
'end\n' ...
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
set_param([model '/Rload'],'BranchType','R');

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
set_param([model '/Vref'], 'Value', num2str(Vref_buck));
set_param([model '/Iref'], 'Value', num2str(Iref_max));
set_param([model '/mode'], 'Value', '1');

% ---- 记录块 ----
for b = {'Vin_ts','Iin_ts','IL_ts','Vout_ts','Iout_ts','DQ2_ts','DQ5_ts'}
    set_param([model '/' b{1}], 'SaveFormat','Timeseries', 'VariableName', b{1});
end

%% ---- 功率电路连线（物理网络，与开环一致）----
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
% 反馈：Vout/Iout -> 控制器
add_line(model, 'Vm_out/1', 'BB_LoopCtl/1', 'autorouting','on');
add_line(model, 'Im_out/1', 'BB_LoopCtl/2', 'autorouting','on');
add_line(model, 'Vref/1', 'BB_LoopCtl/3', 'autorouting','on');
add_line(model, 'Iref/1', 'BB_LoopCtl/4', 'autorouting','on');
add_line(model, 'mode/1', 'BB_LoopCtl/5', 'autorouting','on');
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
add_line(model, 'BB_LoopCtl/1', 'DQ2_ts/1', 'autorouting','on');
add_line(model, 'BB_LoopCtl/2', 'DQ5_ts/1', 'autorouting','on');

%% Solver 配置
set_param(model, 'SolverType','Variable-step', 'Solver','ode23tb', ...
    'MaxStep','1e-7', 'RelTol','1e-4', 'AbsTol','1e-6', ...
    'StopTime', num2str(Tsim));

outdir = 'C:/Users/ll/Desktop/大创/Simulink';
if ~exist(outdir,'dir'), mkdir(outdir); end
save_system(model, [outdir '/buckboost_closedloop.slx']);
disp('=== 模型已保存，开始 Buck 场景仿真 ===');

%% ================= 场景1: Buck 恒压闭环 (24V -> 12V) =================
set_param([model '/Vref'], 'Value', num2str(Vref_buck));
set_param([model '/Iref'], 'Value', num2str(Iref_max));
set_param([model '/mode'], 'Value', '1');
set_param([model '/Rload'],'BranchType','R','Resistance',num2str(Rload_buck));
simOut_b = sim(model);
Vin_b  = simOut_b.get('Vin_ts');  Iin_b = simOut_b.get('Iin_ts');  IL_b = simOut_b.get('IL_ts');
Vout_b = simOut_b.get('Vout_ts'); Iout_b = simOut_b.get('Iout_ts');
DQ2_b  = simOut_b.get('DQ2_ts');  DQ5_b = simOut_b.get('DQ5_ts');

%% ================= 场景2: Boost 恒压闭环 (24V -> 36V) =================
set_param([model '/Vref'], 'Value', num2str(Vref_boost));
set_param([model '/Iref'], 'Value', num2str(Iref_max));
set_param([model '/mode'], 'Value', '2');
set_param([model '/Rload'],'BranchType','R','Resistance',num2str(Rload_boost));
% Boost 从 Vin=24V 起步升压（避免从 0 硬启导致电感电流飙升，对应实物"先Buck预充再Boost"）
set_param([model '/Cout'],'Setx0','on','InitialVoltage','24');
set_param([model '/L'],'SetiL0','on','InitialCurrent','0');
simOut_o = sim(model);
Vin_o  = simOut_o.get('Vin_ts');  Iin_o = simOut_o.get('Iin_ts');  IL_o = simOut_o.get('IL_ts');
Vout_o = simOut_o.get('Vout_ts'); Iout_o = simOut_o.get('Iout_ts');
DQ2_o  = simOut_o.get('DQ2_ts');  DQ5_o = simOut_o.get('DQ5_ts');

%% ================= 结果分析 =================
% 稳态取后 30%
nb = round(numel(Vout_b.Time)*0.7);
no = round(numel(Vout_o.Time)*0.7);
Vout_b_ss = mean(Vout_b.Data(nb:end));
Vout_o_ss = mean(Vout_o.Data(no:end));
Iout_b_ss = mean(Iout_b.Data(nb:end));
Iout_o_ss = mean(Iout_o.Data(no:end));
Pin_b  = Vin_b.Data .* Iin_b.Data;   Pin_b_ss = mean(Pin_b(nb:end));
Pout_b = Vout_b.Data .* Iout_b.Data; Pout_b_ss = mean(Pout_b(nb:end));
Pin_o  = Vin_o.Data .* Iin_o.Data;   Pin_o_ss = mean(Pin_o(nb:end));
Pout_o = Vout_o.Data .* Iout_o.Data; Pout_o_ss = mean(Pout_o(nb:end));
err_b = abs(Vout_b_ss - Vref_buck);
err_o = abs(Vout_o_ss - Vref_boost);

fprintf('\n================ 闭环仿真结果汇总 ================\n');
fprintf('%-8s %8s %8s %8s %8s %8s %8s\n', '模式', 'Vref(V)', 'Vout_ss', 'Iout_ss', '稳态误差', '效率%', 'Vout纹波');
fprintf('%-8s %8.2f %8.3f %8.3f %8.4f %8.2f %8.3f\n', ...
    'Buck', Vref_buck, Vout_b_ss, Iout_b_ss, err_b, Pout_b_ss/Pin_b_ss*100, std(Vout_b.Data(nb:end)));
fprintf('%-8s %8.2f %8.3f %8.3f %8.4f %8.2f %8.3f\n', ...
    'Boost', Vref_boost, Vout_o_ss, Iout_o_ss, err_o, Pout_o_ss/Pin_o_ss*100, std(Vout_o.Data(no:end)));
fprintf('==================================================\n');

%% ================= 绘图 =================
t_b = Vout_b.Time;  t_o = Vout_o.Time;
% Buck
fig1 = figure('Name','Buck 闭环 24V->12V','Visible','off','Position',[80 80 900 700]);
subplot(4,1,1);
plot(t_b*1e3, Vout_b.Data, 'b', 'LineWidth', 1); hold on;
yline(Vref_buck, 'r--', 'LineWidth', 1.2);
title(sprintf('Buck 闭环  Vin=24V -> Vref=12V, 稳态 Vout=%.3fV (误差 %.3fV)', Vout_b_ss, err_b));
ylabel('Vout (V)'); xlabel('时间 ms'); grid on; legend('Vout','Vref');
subplot(4,1,2);
plot(t_b*1e3, IL_b.Data, 'm', 'LineWidth', 1);
title('电感电流 IL (CCM)'); ylabel('A'); xlabel('ms'); grid on;
subplot(4,1,3);
plot(DQ2_b.Time*1e3, DQ2_b.Data, 'b', 'LineWidth', 1); hold on;
plot(DQ5_b.Time*1e3, DQ5_b.Data, 'r', 'LineWidth', 1);
title('占空比 D_{Q2}(Buck主) / D_{Q5}(Boost固定6%)'); ylabel('占空比'); xlabel('ms'); grid on;
legend('D Q2','D Q5');
subplot(4,1,4);
plot(t_b*1e3, Pin_b, 'g', 'LineWidth', 1); hold on; plot(t_b*1e3, Pout_b, 'r', 'LineWidth', 1);
title(sprintf('功率 Pin/Pout, 效率=%.1f%%', Pout_b_ss/Pin_b_ss*100)); ylabel('W'); xlabel('ms'); grid on;
exportgraphics(fig1, [outdir '/buckboost_closed_buck.png'], 'Resolution', 150);

% Boost
fig2 = figure('Name','Boost 闭环 24V->36V','Visible','off','Position',[80 80 900 700]);
subplot(4,1,1);
plot(t_o*1e3, Vout_o.Data, 'b', 'LineWidth', 1); hold on;
yline(Vref_boost, 'r--', 'LineWidth', 1.2);
title(sprintf('Boost 闭环  Vin=24V -> Vref=36V, 稳态 Vout=%.3fV (误差 %.3fV)', Vout_o_ss, err_o));
ylabel('Vout (V)'); xlabel('时间 ms'); grid on; legend('Vout','Vref');
subplot(4,1,2);
plot(t_o*1e3, IL_o.Data, 'm', 'LineWidth', 1);
title('电感电流 IL (CCM)'); ylabel('A'); xlabel('ms'); grid on;
subplot(4,1,3);
plot(DQ2_o.Time*1e3, DQ2_o.Data, 'b', 'LineWidth', 1); hold on;
plot(DQ5_o.Time*1e3, DQ5_o.Data, 'r', 'LineWidth', 1);
title('占空比 D_{Q2}(Buck固定94%) / D_{Q5}(Boost主)'); ylabel('占空比'); xlabel('ms'); grid on;
legend('D Q2','D Q5');
subplot(4,1,4);
plot(t_o*1e3, Pin_o, 'g', 'LineWidth', 1); hold on; plot(t_o*1e3, Pout_o, 'r', 'LineWidth', 1);
title(sprintf('功率 Pin/Pout, 效率=%.1f%%', Pout_o_ss/Pin_o_ss*100)); ylabel('W'); xlabel('ms'); grid on;
exportgraphics(fig2, [outdir '/buckboost_closed_boost.png'], 'Resolution', 150);

disp(['Buck 波形: ' outdir '/buckboost_closed_buck.png']);
disp(['Boost 波形: ' outdir '/buckboost_closed_boost.png']);
