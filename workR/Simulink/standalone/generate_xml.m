function [ ] = generate_xml()

%% Generate mbs_data
mbs_data = generate_mbs_data();

%% Generate the .mbsdata

filename = 'Model_standalone.mbsdata';
pathname = '../../StandaloneC/src/project/';
    
MBSdata_extract(mbs_data, filename, pathname);

end
