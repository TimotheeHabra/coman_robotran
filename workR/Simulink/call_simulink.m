%% Initialization
clc;
close all;
clear all;

%% Generate mbs_data

mbs_data = generate_mbs_data();
mbs_data_closed = mbs_data;

%% Parameters to check

% -- Simulation model parameters [s] -- %
start_time  = 0.0;
finish_time = 60.0;
step_size   = 5.0e-4;


%% Simulink

model_name = 'dirdyna_coman_robotran';

full_model_name = sprintf('%s.mdl', model_name);

open_system(full_model_name);

set_param(model_name, 'Solver','ode4',...
         'StartTime',num2str(start_time),...
         'StopTime',num2str(finish_time),...
         'FixedStep',num2str(step_size));
     
% mbs_data.DonePart=1;

ud =  get_param([model_name '/S-Function_dirdynared'], 'UserData');
ud.mbs_data = mbs_data;
set_param([model_name '/S-Function_dirdynared'], 'UserData', ud);

tic
sim(full_model_name); % launch simulation
toc

%% Outputs -> use this section to plot graphs, analyze the results... using the Matlab framework

t    = tsim.signals.values;
out  = out.signals.values;

% Graphical Results: position tracking
figure;
hold on;

% right arm: reference and actual position for the elbow [rad]
plot(t,out(:,1),'b'); % reference
plot(t,out(:,2),'r'); % actual position

% left arm: reference and actual position for the elbow [rad]
plot(t,out(:,3),'Color',[33 75 31]./255);   % reference
plot(t,out(:,4),'Color',[131 15 246]./255); % actual position

ylim([-1.3 0.25]);
xlabel('time [s]');
ylabel('position [rad]');
title('Position tracking');
legend('right ref [rad]','right pos [rad]','left ref [rad]','left pos [rad]');
