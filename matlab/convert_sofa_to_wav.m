%% this function will conbvert from SOFA file to WAV file
%  Reference: https://www.york.ac.uk/sadie-project/database.html

function convert_sofa_to_wav (input_sofa_file, output_wav_dir, sample_rate, BitsPerSample, output_ref_dir)

%% automatically go to the dir, where this script is located
delim = filesep;
this_script = mfilename('fullpath');
[pathstr,~,~] = fileparts(this_script);
cd(pathstr);

%% add necessary paths
addpath(genpath(pathstr));

%% input arguments
if(~exist('input_sofa_file','var'))
    input_sofa_file = './SADIE_hrtf_database/SOFA/D1_HRIR_SOFA/D1_44K_16bit_256tap_FIR_SOFA.sofa';
end
if(~exist('output_wav_dir','var'))
    output_wav_dir = './SADIE_hrtf_database/WAV/D1_HRIR_44K/';
end
if(~exist('sample_rate','var'))
    sample_rate = 44100;
end
if(~exist('BitsPerSample','var'))
    BitsPerSample = 16;
end
    
%% Read a SOFA file
sofa_content = SOFAload(input_sofa_file);

%% create output dir if needed
if(~exist('NAME','dir'))
    mkdir(output_wav_dir);
end

%% extract hrir for each az and el and store in wav format
all_az_el = sofa_content.SourcePosition;
for ii = 1:length(all_az_el)
    fprintf('running index %d out of %d\n', ii, length(all_az_el));
    hrir_l = squeeze(sofa_content.Data.IR(ii,1,:));
    hrir_r = squeeze(sofa_content.Data.IR(ii,2,:));
    hrir = [hrir_l hrir_r];
    az = double(all_az_el(ii,1));
    el = double(all_az_el(ii,2));
    az_str = num2str(az,'%.1f');
    %az_str = strrep(az_str,'.',',');
    el_str = num2str(el,'%.1f');
    %el_str = strrep(el_str,'.',',');
    %fname = ['azi_' az_str '_ele_' el_str '.wav'];
    fname = ['E' el_str '_A' az_str '_D1.wav']; % naming scheme for Resonance SH calc
    fname = [output_wav_dir delim fname];
    audiowrite(fname,hrir,sample_rate,'BitsPerSample',BitsPerSample);
end

%% compare data in output_ref_dir to output_wav_dir
if(exist('output_ref_dir','var'))
    listings_in_refdir = dir(output_ref_dir);
    listings_in_outdir = dir(output_wav_dir);
    missing_count = 0;
    if(length(listings_in_refdir) > length(listings_in_outdir))
        for jj = 1:length(listings_in_refdir)
            [~,f_ref,EXT_ref] = fileparts(listings_in_refdir(jj).name);
            f_ref_fullpath = [output_ref_dir delim f_ref EXT_ref];   % ref file
            f_out_fullpath = [output_wav_dir delim f_ref EXT_ref];   % out file

            if (~exist(f_out_fullpath,'file'))
                fprintf('output file does not exit: %s\n', f_ref);
                missing_count = missing_count + 1;
                continue
            end

            if (~listings_in_refdir(jj).isdir && strcmp(EXT_ref, '.wav'))
                hrir_ref = audioread(f_ref_fullpath);
                hrir_out = audioread(f_out_fullpath);
                err_ref_out = sum(sum(abs(hrir_ref-hrir_out)));
                if (err_ref_out ~= 0)
                    error('ref file %s is not same as output file\n', f_ref);
                end
            end
        end
        fprintf('total number of missing files from out dir: %d\n', missing_count);
    else
        for jj = 1:length(listings_in_outdir)
            [~,f_out,EXT_out] = fileparts(listings_in_outdir(jj).name);
            f_ref_fullpath = [output_ref_dir delim f_out EXT_out];   % ref file
            f_out_fullpath = [output_wav_dir delim f_out EXT_out];   % out file

            if (~exist(f_ref_fullpath,'file'))
                fprintf('ref file does not exit: %s\n', f_out);
                missing_count = missing_count + 1;
                continue
            end

            if (~listings_in_outdir(jj).isdir && strcmp(EXT_ref, '.wav'))
                hrir_ref = audioread(f_ref_fullpath);
                hrir_out = audioread(f_out_fullpath);
                err_ref_out = sum(sum(abs(hrir_ref-hrir_out)));
                if (err_ref_out ~= 0)
                    error('ref file %s is not same as output file\n', f_out);
                end
            end
        end
        fprintf('total number of missing files from ref dir: %d\n', missing_count);
    end
end