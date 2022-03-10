% Animates quadcopter trajectory using PX4 log file (ulog format)
% Also, shows evolution of RCAC theta parameters [1].

% [1] Spencer, J., Lee, J., Paredes, J. A., Goel, A., & Bernstein, D. (2021).
% An adaptive PID autotuner for multicopters with experimental results.
% arXiv preprint arXiv:2109.12797, accepted to ICRA 2022

% Quadcopter animation files modified from https://github.com/gibiansky/experiments/tree/master/quadcopter/matlab

% 3D arrow program obtained from https://www.mathworks.com/matlabcentral/fileexchange/71994-arrow-3d

close all
clear all
clc

filename = 'TestFlight2.ulg';
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

% Reading vehicle local position setpoint
local_pos_sp = readTopicMsgs(ulog,'TopicNames',{'vehicle_local_position_setpoint'});
local_pos_sp_TT = local_pos_sp.TopicMessages{1,1};
local_pos_sp_TT_num = local_pos_sp_TT(:,S);

% Reading RCAC Attitude Variables
rcac_att_theta_msg = readTopicMsgs(ulog,'TopicNames',{'rcac_att_variables'});
rcac_att_theta_TT = rcac_att_theta_msg.TopicMessages{1,1};
rcac_att_theta_TT_num = rcac_att_theta_TT(:,S);

% Reading RCAC Angular Rate Variables
rcac_rate_theta_msg = readTopicMsgs(ulog,'TopicNames',{'rcac_rate_variables'});
rcac_rate_theta_TT = rcac_rate_theta_msg.TopicMessages{1,1};
rcac_rate_theta_TT_num = rcac_rate_theta_TT(:,S);

% Reading RCAC Position and Velocity Variables
rcac_pos_vel_theta_msg = readTopicMsgs(ulog,'TopicNames',{'rcac_pos_vel_variables'});
rcac_pos_vel_theta_TT = rcac_pos_vel_theta_msg.TopicMessages{1,1};
rcac_pos_vel_theta_TT_num = rcac_pos_vel_theta_TT(:,S);

% Coordinate correction
xx =  local_pos_TT_num.x;
yy =  -local_pos_TT_num.y;
zz =  -local_pos_TT_num.z;
xx_sp =  local_pos_sp_TT_num.x;
yy_sp =  -local_pos_sp_TT_num.y;
zz_sp =  -local_pos_sp_TT_num.z;
qq =  att_TT_num.q;
qq = qq(1:2:end,:); % Attitude is sampled twice as fast as position messages.

rcac_att_theta = rcac_att_theta_TT_num.rcac_att_theta;
rcac_rate_theta = rcac_rate_theta_TT_num.rcac_rate_theta;
rcac_pos_theta = rcac_pos_vel_theta_TT_num.rcac_pos_theta;
rcac_vel_theta = rcac_pos_vel_theta_TT_num.rcac_vel_theta;

timeList = rcac_att_theta_TT_num.timestamp;
tt_RCAC = seconds(timeList - timeList(1));

minL = min([length(xx),length(xx_sp),length(qq(:,1))]);

% Using dk to downsample log entries

xx = xx(1:dk:minL);
yy = yy(1:dk:minL);
zz = zz(1:dk:minL);

xx_sp = xx_sp(1:dk:minL);
yy_sp = yy_sp(1:dk:minL);
zz_sp = zz_sp(1:dk:minL);

qq = qq(1:dk:minL,:);

rcac_att_theta = rcac_att_theta(1:dk:minL,:);
rcac_rate_theta = rcac_rate_theta(1:dk:minL,:);
rcac_pos_theta = rcac_pos_theta(1:dk:minL,:);
rcac_vel_theta = rcac_vel_theta(1:dk:minL,:);

tt_RCAC = tt_RCAC(1:dk:minL,:);

% Create figure and subplot objects to enable separate plotting
figure; plots = [subplot(4, 6, [1:4 7:10 13:16 19:22]), subplot(4, 6, 5:6),...
    subplot(4, 6, 11:12), subplot(4, 6, 17:18), subplot(4, 6, 23:24)];

subplot(plots(1));

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
    subplot(plots(1));
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
    
    % Plotting RCAC theta variables
    subplot(plots(2));
    plot(tt_RCAC(1:ii), rcac_att_theta(1:ii,:),'linewidth',2)
    xlim([tt_RCAC(1) tt_RCAC(end)])
    ylabel('$\theta_q$ (m)','interpreter','latex')
    
    subplot(plots(3));
    plot(tt_RCAC(1:ii), rcac_rate_theta(1:ii,:),'linewidth',2)
    xlim([tt_RCAC(1) tt_RCAC(end)])
    ylabel('$\theta_\omega$ (m)','interpreter','latex')
    
    subplot(plots(4));
    plot(tt_RCAC(1:ii), rcac_pos_theta(1:ii,:),'linewidth',2)
    xlim([tt_RCAC(1) tt_RCAC(end)])
    ylabel('$\theta_r$ (m)','interpreter','latex')
    
    subplot(plots(5));
    plot(tt_RCAC(1:ii), rcac_vel_theta(1:ii,:),'linewidth',2)
    xlim([tt_RCAC(1) tt_RCAC(end)])
    xlabel('$t$ (s)','interpreter','latex')
    ylabel('$\theta_v$ (m)','interpreter','latex')
    
    ff = gcf;
    ff.Position = [400,100,1000,800];
    
    % Uncomment instructions to record video
    %currFrame = getframe(gcf);
    %writeVideo(vidObj,currFrame);
    
    % Comment instructions to record video
    drawnow
    pause(0.1)
    
    % Plots previous local setpoint as a green sphere is it is not NAN
    subplot(plots(1));
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