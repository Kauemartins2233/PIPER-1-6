function fix_sample_time()
% Forca SampleTime=0.01 no MATLAB Function block hil_io do
% modelo_HIL_guiagem para impedir que ODE4 o chame nos 4 sub-passos do RK4.
mdl = 'modelo_HIL_guiagem';
dst_dir = fileparts(fileparts(mfilename('fullpath')));
dst = fullfile(dst_dir, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(dst);
rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',[mdl '/hil_io']);
chart.ChartUpdate = 'DISCRETE';
chart.SampleTime = '0.01';
save_system(mdl);
close_system(mdl);
fprintf('hil_io SampleTime = 0.01 (discrete)\n');
end
