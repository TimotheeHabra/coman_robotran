function gen_controller_struct(controllers_struct)

[nb_controllers, ~] = size(controllers_struct);


%% ControllerStruct.c

filename = 'ControllersStruct.c';
pathname = '../../StandaloneC/src/project/controller_files';
fname = fullfile(pathname,filename);

fid = fopen(fname,'w');
fprintf(fid,[...
'//---------------------------\r\n' ...
'// Nicolas Van der Noot\r\n' ...
'//\r\n' ...
'// Creation : 19-Sep-2013\r\n' ...
'// Last update : ' date '\r\n' ...
'//---------------------------\r\n\r\n' ...
'#include <stdlib.h>\r\n\r\n' ...
'#include "ControllersStruct.h"\r\n' ...
'\r\n\r\n']);

fprintf(fid,[...
'// ---- Controlleres initialization ---- //\r\n' ...
'\r\n']);

% init
for k = 1:nb_controllers
    
    control_name = controllers_struct{k,1};
    control_vars = controllers_struct{k,2};
    
    [nb_variables, ~] = size(control_vars);

    fprintf(fid,[...
    '// ' control_name 'Struc\r\n' ...
    '' control_name ' * init_' control_name '(void)\r\n' ...
    '{\r\n' ...
    '    ' control_name ' *cvs;\r\n\r\n']);
    
    found_sigle_tab = 0;
    found_multi_tab = 0;
    for i = 1:nb_variables        
        if length(control_vars{i,3}) > 1
            found_multi_tab = 1;
            break;
        elseif (length(control_vars{i,3}) == 1) && (control_vars{i,3} > 1)
            found_sigle_tab = 1;
        end        
    end

    if found_multi_tab
        fprintf(fid,'    int i, j;\r\n\r\n');
    elseif found_sigle_tab
        fprintf(fid,'    int i;\r\n\r\n');
    end
    
    fprintf(fid,[...
    '    cvs = (' control_name '*) malloc(sizeof(' control_name '));\r\n' ... 
    '\r\n']);

    for i = 1:nb_variables

        if (length(control_vars{i,3}) == 2) % 2-entries tabular

            tab_size = control_vars{i,3};
        
            fprintf(fid,'    for (i=0;i<%d;i++)\n', tab_size(1));
            fprintf(fid,'    {\n');
            fprintf(fid,'        for (j=0;j<%d;j++)\n', tab_size(2));
            fprintf(fid,'        {\n');

            if strcmp(control_vars{i,2},'int')            
                fprintf(fid,'            cvs->%s[i][j] = 0;\n', control_vars{i,1});
            
            elseif strcmp(control_vars{i,2},'double')            
                fprintf(fid,'            cvs->%s[i][j] = 0.0;\n', control_vars{i,1});
            
            else            
                fprintf(fid,'            cvs->%s[i][j] = init_%s();\n', control_vars{i,1}, control_vars{i,2});
            end

            fprintf(fid,'        }\n');
            fprintf(fid,'    }\n\n');
        
        elseif (control_vars{i,3} == 1) % single variables
        
            if strcmp(control_vars{i,2},'int')            
                fprintf(fid,'    cvs->%s = 0;\n\n', control_vars{i,1}); 
            
            elseif strcmp(control_vars{i,2},'double')            
                fprintf(fid,'    cvs->%s = 0.0;\n\n', control_vars{i,1}); 
            
            else            
                fprintf(fid,'    cvs->%s = init_%s();\n\n', control_vars{i,1}, control_vars{i,2});
            end
        
        elseif (control_vars{i,3} > 1) % single-entry tabular
        
            fprintf(fid,'    for (i=0;i<%d;i++)\n', control_vars{i,3});
            fprintf(fid,'    {\n');

            if ( strcmp(control_vars{i,2}, 'int') )            
                fprintf(fid,'        cvs->%s[i] = 0;\n', control_vars{i,1});
            
            elseif ( strcmp(control_vars{i,2}, 'double') )            
                fprintf(fid,'        cvs->%s[i] = 0.0;\n', control_vars{i,1});
            
            else            
                fprintf(fid,'        cvs->%s[i] = init_%s();\n', control_vars{i,1}, control_vars{i,2});
            end

            fprintf(fid,'    }\n\n');
        end
        
    end

    fprintf(fid,[...
    '    return cvs;\r\n' ...
    '}\r\n\r\n']);

end

fprintf(fid,[...
'// ---- Controllers: free ---- //\r\n' ...
'\r\n']);

% free
for k = 1:nb_controllers
    
    control_name = controllers_struct{k,1};
    control_vars = controllers_struct{k,2};
    
    [nb_variables, ~] = size(control_vars);

    fprintf(fid,[...
    '// ' control_name '\r\n' ...
    'void free_' control_name '(' control_name ' *cvs)\r\n' ...
    '{\r\n']);

    found_sigle_tab = 0;
    found_multi_tab = 0;
    for i = 1:nb_variables    
        if ( ~strcmp(control_vars{i,2}, 'int') ) && ( ~strcmp(control_vars{i,2}, 'double') )               
            if length(control_vars{i,3}) > 1
                found_multi_tab = 1;
                break;
            elseif (length(control_vars{i,3}) == 1) && (control_vars{i,3} > 1)
                found_sigle_tab = 1;
            end              
        end
    end

    if found_multi_tab
        fprintf(fid,'    int i, j;\r\n\r\n');
    elseif found_sigle_tab
        fprintf(fid,'    int i;\r\n\r\n');
    end


    for i = 1:nb_variables

        if ( ~strcmp(control_vars{i,2}, 'int') ) && ( ~strcmp(control_vars{i,2}, 'double') ) % structure
        
            if (length(control_vars{i,3}) == 2) % 2-entries tabular

                tab_size = control_vars{i,3};
            
                fprintf(fid,'    for (i=0;i<%d;i++)\n', tab_size(1));
                fprintf(fid,'    {\n');
                fprintf(fid,'        for (j=0;j<%d;j++)\n', tab_size(2));
                fprintf(fid,'        {\n');
                fprintf(fid,'            free_%s(cvs->%s[i][j]);\n', control_vars{i,2}, control_vars{i,1});
                fprintf(fid,'        }\n');
                fprintf(fid,'    }\n\n');
            
            elseif (control_vars{i,3} == 1) % single variables
            
                fprintf(fid,'    free_%s(cvs->%s);\n\n', control_vars{i,2}, control_vars{i,1});
            
            elseif (control_vars{i,3} > 1) % single-entry tabular
            
                fprintf(fid,'    for (i=0;i<%d;i++)\n', control_vars{i,3});
                fprintf(fid,'    {\n');
                fprintf(fid,'        free_%s(cvs->%s[i]);\n', control_vars{i,2}, control_vars{i,1});
                fprintf(fid,'    }\n\n');
            end
        end  
    end

    fprintf(fid,[...
    '    free(cvs);\r\n' ...
    '}\r\n\r\n']);
end

fclose(fid);

fprintf('ControllerStruct.c created\r\n');

%% ControllerStruct.h

filename = 'ControllersStruct.h';
pathname = '../../StandaloneC/src/project/controller_files';
fname = fullfile(pathname,filename);

fid = fopen(fname,'w');
fprintf(fid,[...
'//---------------------------\r\n' ...
'// Nicolas Van der Noot\r\n' ...
'//\r\n' ...
'// Creation : 19-Sep-2013\r\n' ...
'// Last update : ' date '\r\n' ...
'//---------------------------\r\n\r\n' ...
'#ifndef ControllerStruct_h\r\n' ...
'#define ControllerStruct_h\r\n' ...
'\r\n\r\n']);

fprintf(fid,[...
'// ---- Structures definitions (typedef) ---- //\r\n' ...
'\r\n']);

% typedef
for k = 1:nb_controllers
    
    control_name = controllers_struct{k,1};
    control_vars = controllers_struct{k,2};
    
    [nb_variables, ~] = size(control_vars);

    fprintf(fid,[...
    '// ' control_name 'Struc\r\n' ...
    'typedef struct ' control_name '\r\n' ...
    '{\r\n']);

    for i = 1:nb_variables
        
        if ( ( strcmp(control_vars{i,2}, 'int') ) || ( strcmp(control_vars{i,2}, 'double') ) )
            
            if (length(control_vars{i,3}) == 2) % 2-entries tabular
                tab_size = control_vars{i,3};            
                fprintf(fid,'    %s %s[%d][%d];\n', control_vars{i,2}, control_vars{i,1}, tab_size(1), tab_size(2));
            
            elseif (control_vars{i,3} == 1) % single variables            
                fprintf(fid,'    %s %s;\n', control_vars{i,2}, control_vars{i,1});
            
            elseif (control_vars{i,3} > 1) % single-entry tabular (vector)            
                fprintf(fid,'    %s %s[%d];\n', control_vars{i,2}, control_vars{i,1}, control_vars{i,3});
            end
        
        else
        
            if (length(control_vars{i,3}) == 2) % 2-entries tabular
                tab_size = control_vars{i,3};            
                fprintf(fid,'    struct %s *%s[%d][%d];\n', control_vars{i,2}, control_vars{i,1}, tab_size(1), tab_size(2));
            
            elseif (control_vars{i,3} == 1) % single variables            
                fprintf(fid,'    struct %s *%s;\n', control_vars{i,2}, control_vars{i,1});
            
            elseif (control_vars{i,3} > 1) % single-entry tabular (vector)            
                fprintf(fid,'    struct %s *%s[%d];\n', control_vars{i,2}, control_vars{i,1}, control_vars{i,3});
            end
        end

        if (control_vars{i,3} <= -1) % pointers
        
            fprintf(fid,'    %s ', control_vars{i,2});
            for j= control_vars{i,3}+1:0
            
                fprintf(fid,'*');
            end
            fprintf(fid,'%s;\n', control_vars{i,1});
        end
    end

    fprintf(fid,[...
    '\r\n} ' control_name ';\r\n\r\n' ...
    '\r\n']);
end

fprintf(fid,[...
'// ---- Init and free functions: declarations ---- //\r\n' ...
'\r\n']);

% functions declaration
for k = 1:nb_controllers 
    control_name = controllers_struct{k,1};
    fprintf(fid,[control_name ' * init_' control_name '(void);\r\n']);
    fprintf(fid,['void free_' control_name '(' control_name ' *cvs);\r\n\r\n']);
end

fprintf(fid,[...
'/*--------------------*/\r\n' ...
'#endif\r\n' ...
'\r\n']);

fclose(fid);

fprintf('ControllerStruct.h created\r\n');

end