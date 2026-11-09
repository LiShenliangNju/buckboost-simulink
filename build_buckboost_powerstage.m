%% ====================================================================
%  四开关 Buck-Boost 功率级仿真（含功率计）
%  基于 zeruns 四开关同步整流 Buck-Boost 数字电源拓扑
%  拓扑: Vin -[Q2]- 节点A -[L]- 节点B -[Q3]- Vout
%         Q4: 节点A -> GND ;  Q5: 节点B -> GND
%  功率计: Pin = Vin*Iin, Pout = Vout*Iout, 效率 = Pout/Pin
%  验证: 用 PWM 驱动四开关, 分别验证 Buck(24V->12V) 和 Boost(24V->36V)
% =====================================================================
close all;

%% 参数（取自 README / 复现手册）
Vin      = 24;      % 输入电压 [V]
L        = 22e-6;   % 电感 22uH
Cin      = 220e-6;  % 输入电容 220uF
Cout     = 690e-6;  % 输出电容 220uF + 470uF
Rds_on   = 6.2e-3;  % MOS 导通电阻 CJAC80SN10 = 6.2mOhm
fsw      = 100e3;   % 仿真开关频率 100kHz（实物 181.333kHz，折中以减小纹波避免 DCM）
Ts       = 1/fsw;
deadtime = 0.2e-6;  % 死区时间 200ns（防桥臂直通，等效 EG3112 内置死区）
Tsim     = 5e-3;    % 仿真 5ms（让 LC 谐振启动瞬态充分衰减）

% Buck 工作点: 24V -> 12V, 负载 2A => 6ohm, 占空比 0.5
Vout_set_buck = 12;  Rload_buck = 6;   D_buck = Vout_set_buck/Vin;
% Boost 工作点: 24V -> 36V, 负载 2A => 18ohm, 占空比 1/3
Vout_set_boost= 36;  Rload_boost= 18;  D_boost = 1 - Vin/Vout_set_boost;

%% 建模型
model = 'buckboost_powerstage';
if bdIsLoaded(model), bdclose(model); end
new_system(model);
open_system(model);

% ---- 功率级块 ----
add_block('powerlib/Electrical Sources/DC Voltage Source', [model '/Vin'],  'Position',[40  150 80  195]);
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
add_block('powerlib/Measurements/Voltage Measurement',  [model '/Vm_in'], 'Position',[120 380 160 425]);
add_block('powerlib/Measurements/Current Measurement',  [model '/Im_in'], 'Position',[80  150 120 195]);
add_block('powerlib/Measurements/Current Measurement',  [model '/Im_L'],  'Position',[280 150 320 195]);
add_block('powerlib/Measurements/Voltage Measurement',  [model '/Vm_out'],'Position',[640 380 680 425]);
add_block('powerlib/Measurements/Current Measurement',  [model '/Im_out'],'Position',[640 150 680 195]);

% ---- PWM 信号源 ----
add_block('simulink/Sources/Pulse Generator', [model '/PWM_Q2'],'Position',[200 20 245 60]);
add_block('simulink/Sources/Pulse Generator', [model '/PWM_Q4'],'Position',[260 20 305 60]);
add_block('simulink/Sources/Pulse Generator', [model '/PWM_Q3'],'Position',[460 20 505 60]);
add_block('simulink/Sources/Pulse Generator', [model '/PWM_Q5'],'Position',[520 20 565 60]);

% ---- 记录块 (To Workspace) ----
add_block('simulink/Sinks/To Workspace', [model '/Vin_ts'], 'Position',[120 450 170 490]);
add_block('simulink/Sinks/To Workspace', [model '/Iin_ts'], 'Position',[80  450 130 490]);
add_block('simulink/Sinks/To Workspace', [model '/IL_ts'],  'Position',[280 450 330 490]);
add_block('simulink/Sinks/To Workspace', [model '/Vout_ts'],'Position',[640 450 690 490]);
add_block('simulink/Sinks/To Workspace', [model '/Iout_ts'],'Position',[700 450 750 490]);

% ---- 功率级参数 ----
set_param([model '/Vin'], 'Amplitude', num2str(Vin));
set_param([model '/Q2'], 'Ron', num2str(Rds_on));
set_param([model '/Q4'], 'Ron', num2str(Rds_on));
set_param([model '/Q3'], 'Ron', num2str(Rds_on));
set_param([model '/Q5'], 'Ron', num2str(Rds_on));
set_param([model '/L'],  'BranchType','L', 'Inductance', num2str(L));
set_param([model '/Cin'],'BranchType','RC', 'Resistance','0.01', 'Capacitance', num2str(Cin), 'Setx0','on', 'InitialVoltage', num2str(Vin));
set_param([model '/Cout'],'BranchType','C', 'Capacitance', num2str(Cout));

for b = {'Vin_ts','Iin_ts','IL_ts','Vout_ts','Iout_ts'}
    set_param([model '/' b{1}], 'SaveFormat','Timeseries', 'VariableName', b{1});
end

%% ---- 功率电路连线（物理网络） ----
% 输入回路: Vin+ -> Im_in -> Q2.m ; Cin 并接在 Im_in 之后
add_line(model, 'Vin/RConn1', 'Im_in/LConn1', 'autorouting','on');
add_line(model, 'Im_in/RConn1', 'Q2/LConn1', 'autorouting','on');
add_line(model, 'Im_in/RConn1', 'Cin/LConn1', 'autorouting','on');

% 节点A: Q2源 = Q4漏 = 电感电流入口
add_line(model, 'Q2/RConn1', 'Q4/LConn1', 'autorouting','on');
add_line(model, 'Q2/RConn1', 'Im_L/LConn1', 'autorouting','on');
add_line(model, 'Im_L/RConn1', 'L/LConn1', 'autorouting','on');

% 节点B: 电感右端 = Q3源 = Q5漏
add_line(model, 'L/RConn1', 'Q3/RConn1', 'autorouting','on');
add_line(model, 'L/RConn1', 'Q5/LConn1', 'autorouting','on');

% 输出回路: Q3漏 -> Im_out -> Cout/Rload
add_line(model, 'Q3/LConn1', 'Im_out/LConn1', 'autorouting','on');
add_line(model, 'Im_out/RConn1', 'Cout/LConn1', 'autorouting','on');
add_line(model, 'Im_out/RConn1', 'Rload/LConn1', 'autorouting','on');

% 地: 全部负端连到 Vin 负端（电路参考地）
add_line(model, 'Vin/LConn1', 'Q4/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Q5/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Cin/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Cout/RConn1', 'autorouting','on');
add_line(model, 'Vin/LConn1', 'Rload/RConn1', 'autorouting','on');

% 电压测量: Vm_in 测输入, Vm_out 测输出
add_line(model, 'Vm_in/LConn1', 'Im_in/RConn1', 'autorouting','on');
add_line(model, 'Vm_in/LConn2', 'Vin/LConn1', 'autorouting','on');
add_line(model, 'Vm_out/LConn1', 'Im_out/RConn1', 'autorouting','on');
add_line(model, 'Vm_out/LConn2', 'Vin/LConn1', 'autorouting','on');

% ---- 信号连线 ----
add_line(model, 'PWM_Q2/1', 'Q2/1', 'autorouting','on');
add_line(model, 'PWM_Q4/1', 'Q4/1', 'autorouting','on');
add_line(model, 'PWM_Q3/1', 'Q3/1', 'autorouting','on');
add_line(model, 'PWM_Q5/1', 'Q5/1', 'autorouting','on');
add_line(model, 'Vm_in/1', 'Vin_ts/1', 'autorouting','on');
add_line(model, 'Im_in/1', 'Iin_ts/1', 'autorouting','on');
add_line(model, 'Im_L/1', 'IL_ts/1', 'autorouting','on');
add_line(model, 'Vm_out/1', 'Vout_ts/1', 'autorouting','on');
add_line(model, 'Im_out/1', 'Iout_ts/1', 'autorouting','on');

%% Solver 配置
set_param(model, 'SolverType','Variable-step', 'Solver','ode23tb', ...
    'MaxStep','1e-7', 'RelTol','1e-4', 'AbsTol','1e-6', ...
    'StopTime', num2str(Tsim));

% 保存模型
outdir = 'C:/Users/ll/Desktop/大创/Simulink';
if ~exist(outdir,'dir'), mkdir(outdir); end
save_system(model, [outdir '/buckboost_powerstage.slx']);

%% ================= 场景1: Buck 模式 (24V -> 12V) =================
% Q2=D(50%), Q4=~Q2(相位180度), Q3=恒1(100%), Q5=恒0
set_param([model '/PWM_Q2'],'Amplitude','1','Period',num2str(Ts),'PulseWidth',num2str(D_buck*100),'PhaseDelay','0');
set_param([model '/PWM_Q4'],'Amplitude','1','Period',num2str(Ts),'PulseWidth',num2str((1-D_buck)*100 - 2*deadtime/Ts*100),'PhaseDelay',num2str(D_buck*Ts + deadtime));
set_param([model '/PWM_Q3'],'Amplitude','1','Period',num2str(Ts),'PulseWidth','99.9','PhaseDelay','0');
set_param([model '/PWM_Q5'],'Amplitude','0');
set_param([model '/Rload'],'BranchType','R','Resistance',num2str(Rload_buck));
set_param([model '/Cout'],'Setx0','on','InitialVoltage',num2str(Vout_set_buck));
set_param([model '/L'],'SetiL0','on','InitialCurrent','2');
simOut_b = sim(model);
Vin_b  = simOut_b.get('Vin_ts');  Iin_b = simOut_b.get('Iin_ts');  IL_b = simOut_b.get('IL_ts');
Vout_b = simOut_b.get('Vout_ts');  Iout_b = simOut_b.get('Iout_ts');

%% ================= 场景2: Boost 模式 (24V -> 36V) =================
% Q5=D(33.3%), Q3=~Q5(相位180度), Q2=恒1(100%), Q4=恒0
set_param([model '/PWM_Q5'],'Amplitude','1','Period',num2str(Ts),'PulseWidth',num2str(D_boost*100),'PhaseDelay','0');
set_param([model '/PWM_Q3'],'Amplitude','1','Period',num2str(Ts),'PulseWidth',num2str((1-D_boost)*100 - 2*deadtime/Ts*100),'PhaseDelay',num2str(D_boost*Ts + deadtime));
set_param([model '/PWM_Q2'],'Amplitude','1','Period',num2str(Ts),'PulseWidth','99.9','PhaseDelay','0');
set_param([model '/PWM_Q4'],'Amplitude','0');
set_param([model '/Rload'],'BranchType','R','Resistance',num2str(Rload_boost));
set_param([model '/Cout'],'Setx0','on','InitialVoltage',num2str(Vout_set_boost));
set_param([model '/L'],'SetiL0','on','InitialCurrent','3');
simOut_o = sim(model);
Vin_o  = simOut_o.get('Vin_ts');  Iin_o = simOut_o.get('Iin_ts');  IL_o = simOut_o.get('IL_ts');
Vout_o = simOut_o.get('Vout_ts');  Iout_o = simOut_o.get('Iout_ts');

%% ================= 结果分析（功率计） =================
t_b  = Vout_b.Time;
Pin_b  = Vin_b.Data .* Iin_b.Data;
Pout_b = Vout_b.Data .* Iout_b.Data;
nb = round(numel(t_b)*0.5);
Vin_b_avg  = mean(Vin_b.Data(nb:end));
Iin_b_avg  = mean(Iin_b.Data(nb:end));
Vout_b_avg = mean(Vout_b.Data(nb:end));
Iout_b_avg = mean(Iout_b.Data(nb:end));
Pin_b_avg  = mean(Pin_b(nb:end));
Pout_b_avg = mean(Pout_b(nb:end));
eff_b = Pout_b_avg/Pin_b_avg*100;

t_o  = Vout_o.Time;
Pin_o  = Vin_o.Data .* Iin_o.Data;
Pout_o = Vout_o.Data .* Iout_o.Data;
no = round(numel(t_o)*0.5);
Vin_o_avg  = mean(Vin_o.Data(no:end));
Iin_o_avg  = mean(Iin_o.Data(no:end));
Vout_o_avg = mean(Vout_o.Data(no:end));
Iout_o_avg = mean(Iout_o.Data(no:end));
Pin_o_avg  = mean(Pin_o(no:end));
Pout_o_avg = mean(Pout_o(no:end));
eff_o = Pout_o_avg/Pin_o_avg*100;

%% ================= 绘图 =================
% ---- Buck ----
fig1 = figure('Name','Buck 24V->12V','Visible','off','Position',[80 80 900 640]);
subplot(3,1,1);
plot(t_b*1e3, Vout_b.Data, 'b', 'LineWidth', 1);
hold on; yline(Vout_b_avg, 'r--', 'LineWidth', 1.2);
title(sprintf('Buck 模式  Vin=24V -> Vout≈%.2fV  (D=%.0f%%)', Vout_b_avg, D_buck*100));
ylabel('输出电压 V'); xlabel('时间 ms'); grid on;
legend('Vout', sprintf('平均 %.2fV', Vout_b_avg), 'Location','best');
subplot(3,1,2);
plot(t_b*1e3, IL_b.Data, 'm', 'LineWidth', 1);
title('电感电流 (CCM 三角波)'); ylabel('电流 A'); xlabel('时间 ms'); grid on;
subplot(3,1,3);
plot(t_b*1e3, Pin_b, 'g', 'LineWidth', 1); hold on;
plot(t_b*1e3, Pout_b, 'r', 'LineWidth', 1);
yline(Pin_b_avg, 'g--'); yline(Pout_b_avg, 'r--');
title(sprintf('功率计: Pin≈%.1fW, Pout≈%.1fW, 效率≈%.1f%%', Pin_b_avg, Pout_b_avg, eff_b));
ylabel('功率 W'); xlabel('时间 ms'); grid on;
legend('Pin', 'Pout', 'Location','best');
exportgraphics(fig1, [outdir '/buckboost_buck_wave.png'], 'Resolution', 150);

% ---- Boost ----
fig2 = figure('Name','Boost 24V->36V','Visible','off','Position',[80 80 900 640]);
subplot(3,1,1);
plot(t_o*1e3, Vout_o.Data, 'b', 'LineWidth', 1);
hold on; yline(Vout_o_avg, 'r--', 'LineWidth', 1.2);
title(sprintf('Boost 模式  Vin=24V -> Vout≈%.2fV  (D=%.0f%%)', Vout_o_avg, D_boost*100));
ylabel('输出电压 V'); xlabel('时间 ms'); grid on;
legend('Vout', sprintf('平均 %.2fV', Vout_o_avg), 'Location','best');
subplot(3,1,2);
plot(t_o*1e3, IL_o.Data, 'm', 'LineWidth', 1);
title('电感电流 (CCM 三角波)'); ylabel('电流 A'); xlabel('时间 ms'); grid on;
subplot(3,1,3);
plot(t_o*1e3, Pin_o, 'g', 'LineWidth', 1); hold on;
plot(t_o*1e3, Pout_o, 'r', 'LineWidth', 1);
yline(Pin_o_avg, 'g--'); yline(Pout_o_avg, 'r--');
title(sprintf('功率计: Pin≈%.1fW, Pout≈%.1fW, 效率≈%.1f%%', Pin_o_avg, Pout_o_avg, eff_o));
ylabel('功率 W'); xlabel('时间 ms'); grid on;
legend('Pin', 'Pout', 'Location','best');
exportgraphics(fig2, [outdir '/buckboost_boost_wave.png'], 'Resolution', 150);

%% ================= 数值结果汇总 =================
fprintf('\n================ 仿真结果汇总 ================\n');
fprintf('%-10s %10s %10s %10s %10s %10s %10s\n', '模式', 'Vin(V)', 'Iin(A)', 'Pin(W)', 'Vout(V)', 'Iout(A)', '效率(%)');
fprintf('%-10s %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f\n', ...
    'Buck', Vin_b_avg, Iin_b_avg, Pin_b_avg, Vout_b_avg, Iout_b_avg, eff_b);
fprintf('%-10s %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f\n', ...
    'Boost', Vin_o_avg, Iin_o_avg, Pin_o_avg, Vout_o_avg, Iout_o_avg, eff_o);
fprintf('==============================================\n');
disp(['模型已保存: ' outdir '/buckboost_powerstage.slx']);
disp(['Buck 波形: ' outdir '/buckboost_buck_wave.png']);
disp(['Boost 波形: ' outdir '/buckboost_boost_wave.png']);
