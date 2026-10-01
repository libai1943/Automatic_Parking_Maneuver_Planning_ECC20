% ==============================================================================
%  MATLAB Source Codes in association with the paper entitled "Maneuver
%  Planning for Automatic Parking with Safe Travel Corridors: A Numerical
%  Optimal Control Approach" published by European Control Conference (ECC)
%  2020.
%  Copyright (C) 2020 Bai Li
%  2020.02.15
% ==============================================================================
% The codes are licensed under the GNU General Public License v3.0.
% Users need to get a licensed version of AMPL(free for 30 days) from the
% official website https://ampl.com/try-ampl/request-a-full-trial/
% Users are suggested to cite the following articles in association with
% the codes:
% (1) Li, B., & Shao, Z. (2015). A unified motion planning method for
% parking an autonomous vehicle in the presence of irregularly placed
% obstacles. Knowledge-Based Systems, 86, pp. 11-20.
% (2) Li, B. et al. (2020). Maneuver planning for automatic parking
% with safe travel corridors: A numerical optimal control approach. In
% Proc. 2020 European Control Conference (ECC), pp. 1993-1998.
% ==============================================================================
function result = RunMe()
%RUNME Run the original ECC 2020 STC example from any working directory.
% The bundled AMPL executable is selected explicitly. No MATLAB AMPL API.
close all; clc;
project_dir=fileparts(mfilename('fullpath'));
previous_dir=pwd; previous_path=path; previous_env=getenv('PATH');
cleanup=onCleanup(@() RestoreState(previous_dir,previous_path,previous_env)); %#ok<NASGU>
addpath(project_dir); setenv('PATH',[project_dir,pathsep,previous_env]);
global vehicle_TPBV_ obstacle_vertexes_ Nobs vehicle_geometrics_ optimization_
load(fullfile(project_dir,'Case1.mat'));
InitParams();
planning_clock=tic;
[x,y,theta,~,complete]=SearchHybridAStarPath();
assert(complete && numel(x)>1,'Hybrid A* did not return a complete path.');
coarse_path=struct('x',x,'y',y,'theta',theta);
[x,y,theta,v,a,phy,w,tf]=ResamplePath(x,y,theta);
output_dir=fullfile(project_dir,'results');
if ~isfolder(output_dir),mkdir(output_dir);end
cd(output_dir);
[rear_boxes,front_boxes,xr,yr,xf,yf]=SpecifyLocalBoxes(x,y,theta);
WriteInitialGuess(x,y,theta,xr,yr,xf,yf,v,a,phy,w,tf);
WriteBoundaryValues();
copyfile(fullfile(project_dir,'NLP.mod'),'NLP.mod');
copyfile(fullfile(project_dir,'rr.run'),'rr.run');
copyfile(fullfile(project_dir,'ipopt.opt'),'ipopt.opt');
fid=fopen('solve_status.txt','w');fprintf(fid,'999\n');fclose(fid);
executable=fullfile(project_dir,'ampl.exe');
assert(isfile(executable),'AMPL executable is missing from the repository directory.');
[exit_code,log_text]=system(sprintf('"%s" rr.run',executable));
fid=fopen('solver.log','w');fprintf(fid,'%s',log_text);fclose(fid);
fid=fopen('solve_status.txt','r');status=fscanf(fid,'%f');fclose(fid);
assert(exit_code==0 && numel(status)==1 && status(1)<100, ...
    'AMPL/IPOPT did not solve the problem. See results/solver.log.');
fid=fopen('trajectory.txt','r');values=fscanf(fid,'%f',[7,inf]).';fclose(fid);
fid=fopen('terminal_time.txt','r');duration=fscanf(fid,'%f');fclose(fid);
assert(isequal(size(values),[optimization_.Nfe,7]) && all(isfinite(values(:))), ...
    'Incomplete numerical result. See results/solver.log.');
result=struct('x',values(:,1),'y',values(:,2),'theta',values(:,3), ...
    'v',values(:,4),'a',values(:,5),'phi',values(:,6),'omega',values(:,7), ...
    'duration',duration,'coarse_path',coarse_path,'rear_boxes',rear_boxes, ...
    'front_boxes',front_boxes,'planning_seconds',toc(planning_clock));
result.validation=ValidateParkingResult(result);
assert(result.validation.passed,'The computed trajectory failed final validation.');
save('result.mat','result');
PlotParkingResult(result);
fprintf('ECC 2020: T = %.3f s, planning = %.3f s; validation passed.\n',duration,result.planning_seconds);
end

function RestoreState(folder,matlab_path,environment_path)
cd(folder);path(matlab_path);setenv('PATH',environment_path);
end
