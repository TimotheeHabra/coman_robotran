%
% Synchronize other versions of the code with the Simulink one
% (files are copied from the Simulink environment)
%
% Developed by Nicolas Van der Noot - 2014
%

clc;
clear all;
close all;

%% Flags

stand_state_flag = 0; % 1 to synchronize 'simu_variables.m' and 'control_variables.m'
                      % with the StandaloneC version
xml_flag         = 1; % 1 to generate the xml file for the StandaloneC version


%% Path of the original version (Simulink in this script)


%% Generate the .txt files used in the StandaloneC version for the state variables

if stand_state_flag
    
    [simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct] = simu_variables();  
    [controllers_struct] = control_variables();
    
    addpath('make_generate/');
    
    gen_control_variables_txt(controllers_struct);
    gen_simu_variables_txt(simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct);
    
    rmpath('make_generate/');

end

%% Generate the xml file (.mbsdata) for the StandaloneC version

if xml_flag
    
    fprintf('\n>>> Creating the xml file for the StandaloneC version...\n');
   
    addpath('standalone/');

    generate_xml();

    rmpath('standalone/');
end

