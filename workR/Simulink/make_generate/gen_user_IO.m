% Function to be launched from workR !!
% 
% This function generates the files user_sf_IO.c and user_sf_IO.h, which
% define the I/O ports of the Sfunction. This function is a cleaner
% alternative to the procedure described in the tutorial file
% RobotranLearning_CH06_SFunctions, which proposes to modify directly the
% .mbs file of the project to include custom IO's.

function gen_user_IO(user_vars, prjname)

% Project loading
[mbs_data, mbs_info] = mbs_load(prjname,'default');

% Processing user_vars
for i=1:size(user_vars,1),
    mbs_info.user_variables.(user_vars{i,1}).varname = user_vars{i,1};  % varname
    mbs_info.user_variables.(user_vars{i,1}).vartype = user_vars{i,2};  % vartype 
    mbs_info.user_variables.(user_vars{i,1}).size = user_vars{i,3};     % size
    mbs_info.user_variables.(user_vars{i,1}).type = user_vars{i,4};     % type
end

% Generates user_sf_IO.c and user_sf_IO.h
path_to_src_user = '../../StandaloneC/src/project/user_files';

overwrite = 1; % to erase previous files user_sf_IO
mbs_sf_user_controller(path_to_src_user, mbs_info, overwrite, prjname);

end




function mbs_sf_user_controller(pathname,MBS_info,overwrite,prjname)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_s 1.7.a
%
% Creation : 2008 by JF Collard
% Last update : 01/10/08
% Adapted by Nicolas Van der Noot: 18/09/13
% --------------------------
% MBS_SF_USER Creates the user_sf_IO.c and user_sf_IO.h files for S-function project.
%
%   MBS_SF_USER(PATHNAME,MBS_info,OVERWRITE)
%
%
%   See also mbs_sf_create_project.
%
%   Gestion via Bugzilla :
%   03/10/08 : JFC : Bug n?46
%   16/10/08 : JFC : Bug n?51
%
%---
if nargin<3
    overwrite=0;
end

% Read IO parameters
[Ninput,input_name,input_size,Noutput,output_name,output_size] = mbs_get_UserVar_InOut(MBS_info);

%%%%%%%%%%% user_sf_IO.c %%%%%%%%%%%
filename = 'user_sf_IO.c';
fname = fullfile(pathname,filename);

if exist(fname,'file') == 2 && ~overwrite
    button = questdlg(['Would you like to replace ' filename],'File already exist');
    switch button
        case {'No', 'Cancel'}
            disp(['mbs >>> s-function file: ' filename ' not created']);
            return
    end
end

fid = fopen(fname,'w');

% Header
fprintf(fid,mbs_sf_header(filename,MBS_info.mbsname));

% Begin file
fprintf(fid,[...
    '#include "MBSfun.h" \r\n' ...
    '#include "user_sf_IO.h" \r\n' ...
    '#include "sfdef.h" \r\n'...
    '#include "userDef.h"\r\n'...
    '#include "ControllersStruct.h"\r\n'...
    '\r\n']);

fprintf(fid,[''...
'\r\n'...
'UserIOStruct * initUserIO(MBSdataStruct *s)\r\n'...
'{\r\n'...
'    UserIOStruct *uvs;\r\n\r\n']);

if isfield(MBS_info,'user_variables')
    user_var_name = fieldnames(MBS_info.user_variables);
    Nuser_var = length(user_var_name);
    
    found_sigle_tab = 0;
    found_multi_tab = 0;
    for i = 1:Nuser_var
        size = MBS_info.user_variables.(user_var_name{i}).size;
        if length(size) > 1
            found_multi_tab = 1;
            break;
        elseif (length(size) == 1) && (size > 1)
            found_sigle_tab = 1;
        end        
    end

    if found_multi_tab
        fprintf(fid,'    int i, j;\r\n\r\n');
    elseif found_sigle_tab
        fprintf(fid,'    int i;\r\n\r\n');
    end

    fprintf(fid,[''...
    '    uvs = (UserIOStruct*) malloc(sizeof(UserIOStruct));\r\n'...
    '\r\n']);

    for i=1:Nuser_var

        name = MBS_info.user_variables.(user_var_name{i}).varname;
        type = MBS_info.user_variables.(user_var_name{i}).type;
        size = MBS_info.user_variables.(user_var_name{i}).size;
        vartype = MBS_info.user_variables.(user_var_name{i}).vartype;
        
        
        
        if (length(size) == 2) % 2-entries tabular
            
            if strcmp(type,'structure')
                fprintf(fid,'    for (i=1;i<=%d;i++)\n', size(1));
                fprintf(fid,'    {\n');
                fprintf(fid,'        for (j=1;j<=%d;j++)\n', size(2));
                fprintf(fid,'        {\n');
            else
                fprintf(fid,'    for (i=1;i<=%d;i++)\n', size(1));
                fprintf(fid,'    {\n');
                fprintf(fid,'        for (j=1;j<=%d;j++)\n', size(2));
                fprintf(fid,'        {\n');
            end
            
            if strcmp(vartype,'int')
            
                fprintf(fid,'            uvs->%s[i][j] = 0;\n', name);
            
            elseif strcmp(vartype,'double')
            
                fprintf(fid,'            uvs->%s[i][j] = 0.0;\n', name);
            
            elseif strcmp(type,'structure')
            
                fprintf(fid,'            uvs->%s[i][j] = init_%s();\n', name, vartype);
            end

            fprintf(fid,'        }\n');
            fprintf(fid,'    }\n\n');
            
            
        elseif (size == 1) % single variables
            
            if strcmp(vartype,'int')
            
                fprintf(fid,'    uvs->%s = 0;\n\n', name); 
            
            elseif strcmp(vartype,'double')
            
                fprintf(fid,'    uvs->%s = 0.0;\n\n', name); 
            
            elseif strcmp(type,'structure')
            
                fprintf(fid,'    uvs->%s = init_%s();\n\n', name, vartype);
            end
            
        elseif (size > 1) % single-entry tabular
            
            if strcmp(type,'structure')
                fprintf(fid,'    for (i=1;i<=%d;i++)\n', size);
                fprintf(fid,'    {\n');
            else
                fprintf(fid,'    for (i=1;i<=%d;i++)\n', size);
                fprintf(fid,'    {\n');
            end

            if ( strcmp(vartype, 'int') )
            
                fprintf(fid,'        uvs->%s[i] = 0;\n', name);
            
            elseif ( strcmp(vartype, 'double') )
            
                fprintf(fid,'        uvs->%s[i] = 0.0;\n', name);
            else
                fprintf(fid,'        uvs->%s[i] = init_%s();\n', name, vartype);
            end
            fprintf(fid,'    }\n\n');
        end   
    end
else
    %warning('No user_variables field defined in MBS_info');
end



fprintf(fid,[''...
'    // simbodyStruct //\r\n'...
'    #ifdef SIMBODY\r\n'...
'    uvs->simbodyStruct = init_SimbodyStruct();\r\n'...
'    #endif\r\n'...
'\r\n'...
'    return uvs;\r\n'...
'}\r\n'...
'\r\n\r\n'...
'void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)\r\n'...
'{\r\n']);


if isfield(MBS_info,'user_variables')
    user_var_name = fieldnames(MBS_info.user_variables);
    Nuser_var = length(user_var_name);
    
    found_sigle_tab = 0;
    found_multi_tab = 0;
    for i = 1:Nuser_var        
        vartype = MBS_info.user_variables.(user_var_name{i}).vartype;
        size = MBS_info.user_variables.(user_var_name{i}).size;        
        if ( ~strcmp(vartype, 'int') ) && ( ~strcmp(vartype, 'double') )            
            if length(size) > 1
                found_multi_tab = 1;
                break;
            elseif (length(size) == 1) && (size > 1)
                found_sigle_tab = 1;
            end     
        end
    end

    if found_multi_tab
        fprintf(fid,'    int i, j;\r\n');
    elseif found_sigle_tab
        fprintf(fid,'    int i;\r\n');
    end

    for i=1:Nuser_var

        name = MBS_info.user_variables.(user_var_name{i}).varname;
        type = MBS_info.user_variables.(user_var_name{i}).type;
        size = MBS_info.user_variables.(user_var_name{i}).size;
        
        if strcmp(type,'structure')
            nameStructType = MBS_info.user_variables.(user_var_name{i}).vartype;
            
            fprintf(fid,['\r\n'...
                '    // ' nameStructType ': ' name ' //\r\n']);
            
            if (length(size) == 2) % 2-entries tabular
                
                fprintf(fid,'    for (i=1;i<=%d;i++)\n', size(1));
                fprintf(fid,'    {\n');
                fprintf(fid,'        for (j=1;j<=%d;j++)\n', size(2));
                fprintf(fid,'        {\n');                
                fprintf(fid,['            free_' nameStructType '(uvs->' name '[i][j]);\r\n']);                
                fprintf(fid,'        }\n');
                fprintf(fid,'    }\n\n');
                
            elseif (size == 1) % single variables
                
                fprintf(fid,['    free_' nameStructType '(uvs->' name ');\r\n']);    
                
            elseif (size > 1) % single-entry tabular
                
                fprintf(fid,['    for (i=1;i<=' num2str(size) ';i++)\r\n'...
                             '    {\r\n'...
                             '        free_' nameStructType '(uvs->' name '[i]);\r\n'...
                             '    }\r\n']);
            end
        end
    end
end


fprintf(fid,[
'    // SimbodyStruct: simbodyStruct //\r\n'...
'    #ifdef SIMBODY\r\n'...
'    free_SimbodyStruct(uvs->simbodyStruct);\r\n'...
'    #endif\r\n'...
'\r\n'...
'    free(uvs);\r\n'...
'}\r\n\r\n']);

fprintf(fid,'#ifndef CMEX \r\n \r\n');

    % Input sizes
    fprintf(fid,[...
        'void sf_set_user_input_sizes(SimStruct *S, MBSdataStruct *MBSdata, int sf_ninput) \r\n'...
        '{ \r\n'...
        '   if (SF_N_USER_INPUT > 0) { // warning: index starts at sf_ninput \r\n'...
        '        // example: ssSetInputPortWidth(S,sf_ninput,10); \r\n']);
    for i=1:Ninput,
        fprintf(fid,[...
            ' \r\n'...
            '       /* User input port' num2str(i-1) ' : ' input_name{i} ' */ \r\n']);
        if i==1,
            fprintf(fid,[...
                '       ssSetInputPortWidth(S, sf_ninput, ' num2str(input_size(i)) '); \r\n'...
                '       ssSetInputPortDirectFeedThrough(S, sf_ninput, 1); \r\n']);
        else
            fprintf(fid,[...
                '       ssSetInputPortWidth(S,sf_ninput+' num2str(i-1) ',' num2str(input_size(i)) '); \r\n'...
                '       ssSetInputPortDirectFeedThrough(S, sf_ninput+' num2str(i-1) ', 1); \r\n']);
        end
    end
    fprintf(fid,[...
        '   } \r\n'...
        '} \r\n']);

    % Output sizes
    fprintf(fid,[...
        '\r\n'...
        'void sf_set_user_output_sizes(SimStruct *S, MBSdataStruct *MBSdata) \r\n'...
        '        // example: ssSetOutputPortWidth(S, SF_NOUTPUT, 10); \r\n'...
        '{ \r\n'...
        '   if (SF_N_USER_OUTPUT > 0) { // warning: index starts at SF_NOUTPUT \r\n']);
    for i=1:Noutput,
        fprintf(fid,[...
            '\r\n'...
            '       /* User output port' num2str(i-1) ' : ' output_name{i} ' */ \r\n']);
        if i==1,
            fprintf(fid,[...
                '       ssSetOutputPortWidth(S, SF_NOUTPUT, ' num2str(output_size(i)) '); \r\n']);
        else
            fprintf(fid,[...
                '       ssSetOutputPortWidth(S, SF_NOUTPUT+' num2str(i-1) ', ' num2str(output_size(i)) '); \r\n']);
        end
    end
    fprintf(fid,[...
        '   } \r\n'...
        '} \r\n']);

    % Input update
    fprintf(fid,[...
        '\r\n'...
        'void sf_get_user_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds, int sf_ninput) \r\n'...
        '{ \r\n'...
        '    // warning: index starts at sf_ninput\r\n'...
        '    // example: InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,sf_ninput);\r\n'...
        '    //          MBSdata->user_IO->var1 = *uPtrs0[0];\r\n']);
    if any(input_size>1),
        fprintf(fid,...
        '    int i;\r\n');
    end
    for i=1:Ninput,
        if i==1,
            fprintf(fid,['  InputRealPtrsType uPtrs' num2str(i-1) ' = ssGetInputPortRealSignalPtrs(S,sf_ninput); \r\n']);
        else
            fprintf(fid,['  InputRealPtrsType uPtrs' num2str(i-1) ' = ssGetInputPortRealSignalPtrs(S,sf_ninput+' num2str(i-1) '); \r\n']);
        end
    end
    for i=1:Ninput,
        fprintf(fid,[...
            '\r\n'...
            '   /* User input port' num2str(i-1) ' : ' input_name{i} ' */ \r\n']);
        if i==1,
            fprintf(fid,'   if (ssGetInputPortConnected(S,sf_ninput)) \r\n');
        else
            fprintf(fid,['   if (ssGetInputPortConnected(S,sf_ninput+' num2str(i-1) ')) \r\n']);
        end
        if input_size(i)==1,
            fprintf(fid,['      MBSdata->user_IO->' input_name{i} ' = *uPtrs' num2str(i-1) '[0]; \r\n']);
        else
            fprintf(fid,['      for (i=1;i<=' num2str(input_size(i)) ';i++)\r\n'...
                         '          MBSdata->user_IO->' input_name{i} '[i] = *uPtrs' num2str(i-1) '[i-1]; \r\n']);
        end
    end
    fprintf(fid,'} \r\n');

    % Output update
    fprintf(fid,[...
        '\r\n'...
        'void sf_set_user_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) \r\n'...
        '{ \r\n'...
        '    // warning: index starts at SF_NOUTPUT  \r\n'...
        '    // example: real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT); \r\n'...
        '    //          *y0 = MBSdata->user_IO->var1;  \r\n']);
    if any(output_size>1),
        fprintf(fid,...
        '    int i;\r\n');
    end
    for i=1:Noutput,
        if i==1,
            fprintf(fid,['  real_T *y' num2str(i-1) ' = ssGetOutputPortRealSignal(S,SF_NOUTPUT); \r\n']);
        else
            fprintf(fid,['  real_T *y' num2str(i-1) ' = ssGetOutputPortRealSignal(S,SF_NOUTPUT+' num2str(i-1) '); \r\n']);
        end
    end
    for i=1:Noutput,
        fprintf(fid,[...
            '\r\n'...
            '   /* User output port' num2str(i-1) ' : ' output_name{i} ' */ \r\n']);
        if i==1,
            fprintf(fid,'   if (ssGetOutputPortConnected(S,SF_NOUTPUT)) \r\n');
        else
            fprintf(fid,['   if (ssGetOutputPortConnected(S,SF_NOUTPUT+' num2str(i-1) ')) \r\n']);
        end
        if output_size(i)==1,
            fprintf(fid,['      *y' num2str(i-1) ' = MBSdata->user_IO->' output_name{i} '; \r\n']);
        else
            fprintf(fid,['      for (i=1;i<=' num2str(output_size(i)) ';i++)\r\n'...
                         '          y' num2str(i-1) '[i-1] = MBSdata->user_IO->' output_name{i} '[i]; \r\n']);
        end
    end
    fprintf(fid,'} \r\n');

fprintf(fid,'\r\n#endif \r\n');

fclose(fid);
disp(['mbs >>> s-function file: ' filename ' created']);


%%%%%%%%%%% user_sf_IO.h %%%%%%%%%%%

filename = 'user_sf_IO.h';
fname = fullfile(pathname,filename);

if exist(fname,'file') == 2 && ~overwrite
    button = questdlg(['Would you like to replace ' filename],'File already exist');
    switch button
        case {'No', 'Cancel'}
            disp(['mbs >>> s-function file: ' filename ' not created']);
            return
    end
end

fid = fopen(fname,'w');

% Header
fprintf(fid,mbs_sf_header(filename,MBS_info.mbsname));

% Begin file
fprintf(fid,[''...
'#ifndef UsersfIO_h\r\n'...
'#define UsersfIO_h\r\n'...
'/*--------------------*/\r\n \r\n']);

fprintf(fid,[...
    '#ifdef ACCELRED \r\n'...
    '#define S_FUNCTION_NAME  mbs_sf_accelred_' prjname ' \r\n'...
    '#elif defined DIRDYNARED \r\n'...
    '#define S_FUNCTION_NAME  mbs_sf_dirdynared_' prjname ' \r\n'...
    '#elif defined INVDYNARED \r\n'...
    '#define S_FUNCTION_NAME  mbs_sf_invdynared_' prjname ' \r\n'...
    '#elif defined SENSORKIN \r\n'...
    '#define S_FUNCTION_NAME  mbs_sf_sensorkin_' prjname ' \r\n'...
    '#endif \r\n'...
    ' \r\n'...
    '#define SF_N_USER_INPUT ' num2str(Ninput) ' \r\n'...
    '#define SF_N_USER_OUTPUT ' num2str(Noutput) ' \r\n'...
    '\r\n'...
    '#include "userDef.h"\r\n'...
    '#include "ControllersStruct.h"\r\n'...
    ' \r\n']);


if isfield(MBS_info,'user_variables')
    user_var_name = fieldnames(MBS_info.user_variables);
    Nuser_var = length(user_var_name);
else
    warning('No user_variables field defined in MBS_info');
    user_var_name = [];
    Nuser_var = 0;

    fprintf(fid,'/*\r\n');

end

fprintf(fid,[''...
'typedef struct UserIOStruct \r\n'...
'{\r\n']);

for i=1:Nuser_var

    name = user_var_name{i};
    type = MBS_info.user_variables.(user_var_name{i}).type;
    size    = MBS_info.user_variables.(user_var_name{i}).size;
    vartype = MBS_info.user_variables.(user_var_name{i}).vartype;
    
    
    if ( ( strcmp(vartype, 'int') ) || ( strcmp(vartype, 'double') ) )
        
        if (length(size) == 2) % 2-entries tabular
            fprintf(fid,'    %s %s[%d+1][%d+1];\n', vartype, name, size(1), size(2));
        
        elseif (size == 1) % single variables
            fprintf(fid,'    %s %s;\n', vartype, name);
        
        elseif (size > 1) % single-entry tabular (vector)
            fprintf(fid,'    %s %s[%d+1];\n', vartype, name, size);
        end
    else
        if (length(size) == 2) % 2-entries tabular
            fprintf(fid,'    struct %s *%s[%d+1][%d+1];\n', vartype, name, size(1), size(2));

        elseif (size == 1) % single variables
            fprintf(fid,'    struct %s *%s;\n', vartype, name);

        elseif (size > 1) % single-entry tabular (vector)
            fprintf(fid,'    struct %s *%s[%d+1];\n', vartype, name, size);        
        end
    end
    
    if (size <= -1) % pointers
        fprintf(fid,'    %s ', vartype);
        for j= size+1:0

            fprintf(fid,'*');
        end
        fprintf(fid,'%s;\n', name);
    end
end



fprintf(fid,[
'\r\n'...
'    #ifdef SIMBODY\r\n'...
'    SimbodyStruct *simbodyStruct;\r\n'...
'    #endif\r\n'...
'\r\n'...
'} UserIOStruct;\r\n']);

if ~isfield(MBS_info,'user_variables')
    fprintf(fid,['*/\r\n'...
        'typedef void* UserIOStruct;\r\n']);
end


fprintf(fid,['\r\n'...
'/*--------------------*/\r\n'...
'#endif\r\n']);

fclose(fid);
disp(['mbs >>> s-function file: ' filename ' created']);

end
