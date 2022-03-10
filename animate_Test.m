% Animates quadcopter trajectory using PX4 log file (ulog format)

% Quadcopter animation files modified from https://github.com/gibiansky/experiments/tree/master/quadcopter/matlab

% 3D arrow program obtained from https://www.mathworks.com/matlabcentral/fileexchange/71994-arrow-3d

close all
clear all
clc

filename = 'TestFlight1.ulg';
ulog = ulogreader(filename);
limLat = 2; % Meters added to displayed space in x and y (horizontal) directions. 
limAlt = 1; % Meters added to displayed space in z (vertical) direction. 
dk = 5;     % Log steps skipped per animated frame (downsampling). 
            % dk = 1 when all log entries need to be visualized.
scale = 0.2;% Scale for quadcopter visualization.
initYaw = pi; %Sometimes, initial yaw may be incorrect. Can add an initial yaw ((-pi, pi]) to correct this.

S = vartype('float');

% Reading vehicle attitude
att = readTopicMsgs(ulog,'TopicNames',{'vehicle_attitude'});
att_TT = att.TopicMessages{1,1};
att_TT_num = att_TT(:,S);

% Reading vehicle local position
local_pos = readTopicMsgs(ulog,'TopicNames',{'vehicle_local_position'});
local_pos_TT = local_pos.TopicMessages{1,1};
local_pos_TT_num = local_pos_TT(:,S);

% Reading vehicle local position setpoints (NOT mission setpoints)
local_pos_sp = readTopicMsgs(ulog,'TopicNames',{'vehicle_local_position_setpoint'});
local_pos_sp_TT = local_pos_sp.TopicMessages{1,1};
local_pos_sp_TT_num = local_pos_sp_TT(:,S);

% Coordinate correction
xx =  local_pos_TT_num.x;
yy =  -local_pos_TT_num.y;
zz =  -local_pos_TT_num.z;
xx_sp =  local_pos_sp_TT_num.x;
yy_sp =  -local_pos_sp_TT_num.y;
zz_sp =  -local_pos_sp_TT_num.z;
qq =  att_TT_num.q;
qq = qq(1:2:end,:); % Attitude is sampled twice as fast as position messages.

minL = min([length(xx),length(xx_sp),length(qq(:,1))]);

% Using dk to downsample log entries

xx = xx(1:dk:minL);
yy = yy(1:dk:minL);
zz = zz(1:dk:minL);

xx_sp = xx_sp(1:dk:minL);
yy_sp = yy_sp(1:dk:minL);
zz_sp = zz_sp(1:dk:minL);

qq = qq(1:dk:minL,:);

% Create quadcopter object
hh = quadcopter(scale);

% Create setpoint object (sphere)
[Xsp, Ysp, Zsp] = sphere;

% Uncomment instructions below to record simulation
%vidObj = VideoWriter('QuadSim.mp4');
%vidObj.FrameRate = 20; 
%open(vidObj);

set(gca,'nextplot','replacechildren');

% Setting displayed maximum and minimum coordinates.
minX = min(xx) - limLat;
maxX = max(xx) + limLat;
minY = min(yy) - limLat;
maxY = max(yy) + limLat;
minZ = min(zz) - limAlt;
maxZ = max(zz) + limAlt;

mZZ  = min(zz);

% Initial yaw
theta0 = quat2eul(qq(1,:),'XYZ');
theta0(3) = theta0(3)- initYaw;

for ii = 1:length(xx)
    % Correcting attitude angles.
    theta = [0 0 -1].*(quat2eul(qq(ii,:),'XYZ') - theta0);
    % Moving and rotating quadcopter animation
    animateR(hh, [xx(ii,:);yy(ii,:);zz(ii,:)], theta');
    % Plots current local setpoint as a red sphere is it is not NAN
    if ~isnan(xx(ii,:))
        zz_sp_plot = zz_sp(ii,:);
        if zz_sp_plot < mZZ
            zz_sp_plot = mZZ;
        end
        hold on
        surf(0.9*scale*Xsp + xx_sp(ii,:), 0.9*scale*Ysp + yy_sp(ii,:), 0.9*scale*0.5*Zsp + zz_sp_plot, 'EdgeColor', 'none', 'FaceColor', 'r');
        hold off
    end
    xlabel('$x$ (m)','interpreter','latex')
    ylabel('$y$ (m)','interpreter','latex')
    zlabel('$z$ (m)','interpreter','latex')
    axis([minX maxX minY maxY minZ maxZ])
    
    ff = gcf;
    ff.Position = [400,100,1000,800];
    
    % Uncomment instructions to record video
    %currFrame = getframe(gcf);
    %writeVideo(vidObj,currFrame);
    
    % Comment instructions to record video
    drawnow
    pause(0.1)
    
    % Plots previous local setpoint as a green sphere is it is not NAN
    if (~isnan(xx(ii,:)) && ii>1)
        zz_sp_plot = zz_sp(ii-1,:);
        if zz_sp_plot < mZZ
            zz_sp_plot = mZZ;
        end
        hold on
        surf(scale*Xsp + xx_sp(ii-1,:), scale*Ysp + yy_sp(ii-1,:), scale*0.5*Zsp + zz_sp_plot, 'EdgeColor', 'none', 'FaceColor', 'g');
        hold off
    end
end

% Uncomment instructions to record video
%writeVideo(vidObj, movieVector);    
%close(vidObj);