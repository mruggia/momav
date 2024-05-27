
% only works with bag files >124 !!!

%BAG = "339"; % orientation sweep
%BAG = "326"; % orientation sweep no sqp
%BAG = "287"; % continuous rotation
BAG = "290"; % position sweep

plot_3d = true; % flag if 3d plots should be generated


%% load bag file
disp("loading bag file...");

% load bag file
filename = "../1_bags/"+string(BAG)+".bag";
bag = rosbag(filename);
topics = bag.AvailableTopics.Properties.RowNames;

% find body state topic (imu, sim, cam or track)
body_state_topic = "";
if any(strcmp(topics, "/momav/body_state_imu")),   body_state_topic = "/momav/body_state_imu"; end
if any(strcmp(topics, "/momav/body_state_sim")),   body_state_topic = "/momav/body_state_sim"; end
if any(strcmp(topics, "/momav/body_state_track")), body_state_topic = "/momav/body_state_track"; end
if any(strcmp(topics, "/momav/body_state_cam")),   body_state_topic = "/momav/body_state_cam"; end

% load body setpoint and state
body_setp  = readMessages(select(bag, "Topic", "/momav/body_setp_pose"), 'DataFormat','struct');
body_state = readMessages(select(bag, "Topic", body_state_topic), 'DataFormat','struct');

% get time data from bag file
func_time = @(o) double(o.Header.Stamp.Sec) + double(o.Header.Stamp.Nsec)*1e-9;
body_setp_time  = cell2mat(cellfun(func_time, body_setp, 'UniformOutput',false));
body_state_time = cell2mat(cellfun(func_time, body_state, 'UniformOutput',false));

% get position data from bag file
func_pos = @(o) [o.Translation.X, o.Translation.Y, o.Translation.Z];
func_vel = @(o) [o.LinearVelocity.X, o.LinearVelocity.Y, o.LinearVelocity.Z];
func_acc = @(o) [o.LinearAcceleration.X, o.LinearAcceleration.Y, o.LinearAcceleration.Z];
func_frc = @(o) [o.Force.X, o.Force.Y, o.Force.Z];
body_setp_pos  = cell2mat(cellfun(func_pos, body_setp,  'UniformOutput',false));
body_setp_vel  = cell2mat(cellfun(func_vel, body_setp,  'UniformOutput',false));
body_setp_acc  = cell2mat(cellfun(func_acc, body_setp,  'UniformOutput',false));
body_setp_frc  = cell2mat(cellfun(func_frc, body_setp,  'UniformOutput',false));
body_state_pos = cell2mat(cellfun(func_pos, body_state, 'UniformOutput',false));
body_state_vel = cell2mat(cellfun(func_vel, body_state, 'UniformOutput',false));
body_state_acc = cell2mat(cellfun(func_acc, body_state, 'UniformOutput',false));

% get rotation data from bag file
func_rot  = @(o) [o.Orientation.W, o.Orientation.X, o.Orientation.Y, o.Orientation.Z];
func_rvel = @(o) [o.AngularVelocity.X, o.AngularVelocity.Y, o.AngularVelocity.Z];
func_racc = @(o) [o.AngularAcceleration.X, o.AngularAcceleration.Y, o.AngularAcceleration.Z];
func_trq  = @(o) [o.Torque.X, o.Torque.Y, o.Torque.Z];
body_setp_rot   = cell2mat(cellfun(func_rot,  body_setp,  'UniformOutput',false));
body_setp_rvel  = cell2mat(cellfun(func_rvel, body_setp,  'UniformOutput',false));
body_setp_racc  = cell2mat(cellfun(func_racc, body_setp,  'UniformOutput',false));
body_setp_trq   = cell2mat(cellfun(func_trq,  body_setp,  'UniformOutput',false));
body_state_rot  = cell2mat(cellfun(func_rot,  body_state, 'UniformOutput',false));
body_state_rvel = cell2mat(cellfun(func_rot,  body_state, 'UniformOutput',false));
body_state_racc = cell2mat(cellfun(func_rot,  body_state, 'UniformOutput',false));

% get start/end time and position offset
body_setp_type = cell2mat(cellfun(@(o) double(o.SetpointType), body_setp, 'UniformOutput',false));
if any(body_setp_type>-1)
    if any(body_setp_type==12)
        valid_times = find( body_setp_type>=1 & body_setp_type<=12 );
    else
        valid_times = find( body_setp_type==0);
    end
    time_start = body_setp_time(valid_times(1));
    time_end = body_setp_time(valid_times(end));

    body_setp_offset = body_setp_pos(valid_times(end),:);
    body_setp_pos = body_setp_pos - body_setp_offset;
    body_state_pos = body_state_pos - body_setp_offset;
else
    time_start = body_setp_time(1);
    time_end = body_setp_time(end);
end

% resample time
time_series = (0.0:0.05:(time_end-time_start))';
body_setp_pos  = interp1(body_setp_time-time_start, body_setp_pos, time_series);
body_setp_vel  = interp1(body_setp_time-time_start, body_setp_vel, time_series);
body_setp_acc  = interp1(body_setp_time-time_start, body_setp_acc, time_series);
body_setp_frc  = interp1(body_setp_time-time_start, body_setp_frc, time_series);
body_setp_rot  = interp1(body_setp_time-time_start, body_setp_rot, time_series);
body_setp_rvel = interp1(body_setp_time-time_start, body_setp_rvel, time_series);
body_setp_racc = interp1(body_setp_time-time_start, body_setp_racc, time_series);
body_setp_trq  = interp1(body_setp_time-time_start, body_setp_trq, time_series);
body_setp_type = interp1(body_setp_time-time_start, body_setp_type, time_series);
body_state_pos  = interp1(body_state_time-time_start, body_state_pos, time_series);
body_state_vel  = interp1(body_state_time-time_start, body_state_vel, time_series);
body_state_acc  = interp1(body_state_time-time_start, body_state_acc, time_series);
body_state_rot  = interp1(body_state_time-time_start, body_state_rot, time_series);
body_state_rvel = interp1(body_state_time-time_start, body_state_rvel, time_series);
body_state_racc = interp1(body_state_time-time_start, body_state_racc, time_series);

% get trajectory type
max_body_setp_pos = max(body_setp_pos);
max_body_setp_rot = max(body_setp_rot);
if all(max_body_setp_pos>0.01) && all(max_body_setp_rot(2:4)<0.01)
    if all(abs(max_body_setp_pos-max_body_setp_pos(1))<0.01)
        traj_type = 1; % position sweep
    end
elseif all(max_body_setp_rot(2:4)>0.01) && all(max_body_setp_pos<0.01)
    if all(abs(max_body_setp_rot(2:4)-max_body_setp_rot(2))<0.01)
        traj_type = 2; % orientation sweep
    end
elseif any(max_body_setp_rot(2:4)>0.01) && all(max_body_setp_pos<0.01)
    traj_type = 3; % continuous rotation
else 
    traj_type = 0; % general trajectory
end


%% calculate derived values
disp("calculating derived values...");

% calculate position errors
body_setp_pos = body_setp_pos*1000;
body_state_pos = body_state_pos*1000;
body_err_pos = (body_setp_pos - body_state_pos);

% calculate rotation errors
body_setp_rot_q = quaternion(body_setp_rot);
body_state_rot_q = quaternion(body_state_rot);
body_err_rot_q = body_setp_rot_q.^-1 .* body_state_rot_q;

body_setp_rot_rv = unwind_rv(rotvec(body_setp_rot_q))*180/pi;
body_state_rot_rv = match_rv(rotvec(body_state_rot_q), body_setp_rot_rv*pi/180)*180/pi;
body_err_rot_rv = unmirror_rv(rotvec(body_err_rot_q))*180/pi;

body_err_rot_ypr = euler(body_err_rot_q,"ZYX","frame")*180/pi;

rv_abs = vecnorm(body_err_rot_rv,2,2);
rv_dir = body_err_rot_rv./rv_abs;
body_err_rot_rv(rv_abs>180,:) = body_err_rot_rv(rv_abs>180,:) - 360*rv_dir(rv_abs>180,:);

% calculate trajectory phase change points
body_setp_type = int32(body_setp_type);
body_setp_type_cng = time_series(diff(body_setp_type)~=0);
body_setp_type_cng = round([0; body_setp_type_cng; time_series(end)]);


%% generate plots
disp("generating plots...");

% constants
RANGE_SETP_POS = 500*1.15; TICKS_SETP_POS = (-500:125:500);
RANGE_SETP_ROT = 180*1.15; TICKS_SETP_ROT = (-180:45:180);
RANGE_ERR_POS  = 60;  TICKS_ERR_POS  = (-60:20:60);
RANGE_ERR_ROT  = 12;  TICKS_ERR_ROT  = (-12:4:12);
RANGE_TIME     = 72;  TICKS_TIME = (0:3:72);
COLORMAP_SKIPS = 1;

if traj_type == 1
    RANGE_SETP_ROT = 12*1.15; TICKS_SETP_ROT = (-12:3:12);
elseif traj_type == 2
    RANGE_SETP_POS = 80*1.15; TICKS_SETP_POS = (-80:20:80); 
elseif traj_type == 3
    RANGE_TIME = 80;  TICKS_TIME = (0:4:80);
    time_series = time_series - 1;
end

% initialize figure
fig = figure(1); clf(fig);
set(gcf,'Position',[500,200,1200,800])
new_co = [031/255 119/255 180/255; 255/255 127/255 014/255; 044/255 160/255 044/255];
colororder(new_co);
new_cm = turbo(256); new_cm = new_cm(110:230,:);
colormap(new_cm);


% 3d position
ax_3d_pos = subplot(4,2,[1 3]);
title('\textbf{Position Trajectory} (cartesian)');
hold on; grid on;
view(35, 20); pbaspect([1 1 1]);
xlabel('$x$ [mm]'); ylabel('$y$ [mm]'); zlabel('$z$ [mm]');
xlim([-RANGE_SETP_POS RANGE_SETP_POS]); ylim([-RANGE_SETP_POS RANGE_SETP_POS]); zlim([-RANGE_SETP_POS RANGE_SETP_POS]);
xticks(TICKS_SETP_POS); yticks(TICKS_SETP_POS); zticks(TICKS_SETP_POS);
xtickangle(45); ytickangle(-45);
colorbar(); clim([0 RANGE_ERR_POS]); ylabel(ax_3d_pos.Colorbar, 'Absolute Position Error [mm]', 'Interpreter','latex');
ax_3d_pos.SortMethod = 'childorder';

p1 = scatter3(body_state_pos(1:COLORMAP_SKIPS:end,1),body_state_pos(1:COLORMAP_SKIPS:end,2),body_state_pos(1:COLORMAP_SKIPS:end,3), 60, vecnorm(body_err_pos(1:COLORMAP_SKIPS:end,:),2,2), 'filled');
%p2 = plot3(body_setp_pos(:,1),body_setp_pos(:,2),body_setp_pos(:,3), 'Color','#AAAAAA');
p3 = plot3(body_state_pos(:,1),body_state_pos(:,2),body_state_pos(:,3), 'Color','#000000');

if max(abs(body_setp_pos), [],"all") > 0.1
    text(0.80*RANGE_SETP_POS,0,-0.18*RANGE_SETP_POS,'$+x$','HorizontalAlignment','center'); text(-0.90*RANGE_SETP_POS,0,-0.18*RANGE_SETP_POS,'$-x$','HorizontalAlignment','center');
    text(0,1.00*RANGE_SETP_POS,-0.18*RANGE_SETP_POS,'$+y$','HorizontalAlignment','center'); text(0,-0.75*RANGE_SETP_POS,-0.18*RANGE_SETP_POS,'$-y$','HorizontalAlignment','center');
    text(0.25*RANGE_SETP_POS,0,0.90*RANGE_SETP_POS,'$+z$','HorizontalAlignment','center');  text(0.25*RANGE_SETP_POS,0,-0.75*RANGE_SETP_POS,'$-z$','HorizontalAlignment','center');
end

%legend([p2, p3], 'Setpoint', 'State', 'Location','northeast');

% 3d rotation
ax_3d_rot = subplot(4,2,[2 4]);
title('\textbf{Orientation Trajectory} (rotation vector)');
hold on; grid on;
view(35, 20); pbaspect([1 1 1]);
xlabel('$x$ [$^{\circ}$]'); ylabel('$y$ [$^{\circ}$]'); zlabel('$z$ [$^{\circ}$]');
xlim([-RANGE_SETP_ROT RANGE_SETP_ROT]); ylim([-RANGE_SETP_ROT RANGE_SETP_ROT]); zlim([-RANGE_SETP_ROT RANGE_SETP_ROT]);
xticks(TICKS_SETP_ROT); yticks(TICKS_SETP_ROT); zticks(TICKS_SETP_ROT);
xtickangle(45); ytickangle(-45);
colorbar(); clim([0 RANGE_ERR_ROT]); ylabel(ax_3d_rot.Colorbar, 'Absolute Orientation Error [$^{\circ}$]', 'Interpreter','latex');
ax_3d_rot.SortMethod = 'childorder';

p1 = scatter3(body_state_rot_rv(1:COLORMAP_SKIPS:end,1),body_state_rot_rv(1:COLORMAP_SKIPS:end,2),body_state_rot_rv(1:COLORMAP_SKIPS:end,3), 60, vecnorm(body_err_rot_rv(1:COLORMAP_SKIPS:end,:),2,2), 'filled');
%p2 = plot3(body_setp_rot_rv(:,1),body_setp_rot_rv(:,2),body_setp_rot_rv(:,3), 'Color','#AAAAAA');
p3 = plot3(body_state_rot_rv(:,1),body_state_rot_rv(:,2),body_state_rot_rv(:,3), 'Color','#000000');

if max(abs(body_setp_rot_rv), [],"all") > 0.1
    text(0.70*RANGE_SETP_ROT,0,-0.22*RANGE_SETP_ROT,'$+\phi\ \mathrm{(roll)}$','HorizontalAlignment','left');   text(-0.90*RANGE_SETP_ROT,0,-0.20*RANGE_SETP_ROT,'$-\phi$','HorizontalAlignment','center');
    text(0,0.86*RANGE_SETP_ROT,-0.14*RANGE_SETP_ROT,'$+\theta\ \mathrm{(pitch)}$','HorizontalAlignment','left'); text(0,-0.75*RANGE_SETP_ROT,-0.18*RANGE_SETP_ROT,'$-\theta$','HorizontalAlignment','center');
    text(0.15*RANGE_SETP_ROT,0,0.87*RANGE_SETP_ROT,'$+\psi\ \mathrm{(yaw)}$','HorizontalAlignment','left');    text(0.25*RANGE_SETP_ROT,0,-0.75*RANGE_SETP_ROT,'$-\psi$','HorizontalAlignment','center');
end

%legend([p2, p3], 'Setpoint', 'State', 'Location','northeast');

% position error
ax_pos_err = subplot(4,2,[5 6]);
hold on; grid on;
xlim([0 RANGE_TIME]); xticks(TICKS_TIME); set(gca,'Xticklabel',[]);
ylim([-RANGE_ERR_POS RANGE_ERR_POS]); yticks(TICKS_ERR_POS);
ylabel('Pos. Error [mm]');

plot(time_series, body_err_pos(:,1))
plot(time_series, body_err_pos(:,2))
plot(time_series, body_err_pos(:,3))

legend('$x$', '$y$', '$z$', 'NumColumns',3);

% rotation error
ax_rot_err = subplot(4,2,[7 8]);
hold on; grid on;
xlim([0 RANGE_TIME]); xticks(TICKS_TIME); xlabel('Time [s]'); 
ylim([-RANGE_ERR_ROT RANGE_ERR_ROT]); yticks(TICKS_ERR_ROT);
ylabel('Orient. Error [$^{\circ}$]');

p1 = plot(time_series, body_err_rot_ypr(:,3));
p2 = plot(time_series, body_err_rot_ypr(:,2));
p3 = plot(time_series, body_err_rot_ypr(:,1));

legend([p3 p2 p1], '$\psi$', '$\theta$', '$\phi$', 'NumColumns',3);

% trajectory type
ax_traj_typ = axes('Position',ax_pos_err.Position);
title('\textbf{Position Error} (cartesian) \& \textbf{Orientation Error} (yaw $\psi$, pitch $\theta$, roll $\phi$) vs. \textbf{Time} \& \textbf{Trajectory Step}');
hold on; grid on;
xlim([0 RANGE_TIME]); xticks(TICKS_TIME); set(gca,'Xticklabel',[]);
ylim([-1 1]); yticks([]);
ylabel('Step');

if traj_type == 1
    xnames = {'$+x$','$+y$','$+z$','$-x$','$-y$','$-z$'};
elseif traj_type == 2
    xnames = {'$+\psi$','$+\theta$','$+\phi$','$-\psi$','$-\theta$','$-\phi$'};
elseif traj_type == 3
    body_setp_type_cng = (0:4:80)';
    xnames = cell(10,1);
    for i = (1:10)
        xnames{i} = strcat('$\phi\!: ', num2str(i),'\cdot2\pi\ $');
    end
end

if traj_type > 0
    for i=1:2:size(body_setp_type_cng,1)
        plot(repmat(body_setp_type_cng(i),2,1),[-1 1],"k");
        if i ~= size(body_setp_type_cng,1)
            text(body_setp_type_cng(i+1),0,xnames{(i-1)/2+1},'VerticalAlignment','middle','HorizontalAlignment','center');
        end
    end
end

% finalize figure
set(findall(gcf,'-property','FontSize'),'FontSize',12);

ax_3d_pos.Position = ax_3d_pos.Position + [+0.004 0.04 0.015 0.04];
ax_3d_rot.Position = ax_3d_rot.Position + [-0.014 0.04 0.015 0.04];

ax_3d_pos.Colorbar.Position = ax_3d_pos.Colorbar.Position + [0.002 0.012 -0.01 -0.024];
ax_3d_rot.Colorbar.Position = ax_3d_rot.Colorbar.Position + [0.002 0.012 -0.01 -0.024];

ax_3d_pos.XLabel.Units='normalized'; ax_3d_pos.XLabel.Position=ax_3d_pos.XLabel.Position+[-0.04 0.06 0];
ax_3d_pos.YLabel.Units='normalized'; ax_3d_pos.YLabel.Position=ax_3d_pos.YLabel.Position+[+0.04 0.10 0];
ax_3d_pos.ZLabel.Units='normalized'; ax_3d_pos.ZLabel.Position=ax_3d_pos.ZLabel.Position+[+0.02 0.00 0];
ax_3d_rot.XLabel.Units='normalized'; ax_3d_rot.XLabel.Position=ax_3d_rot.XLabel.Position+[-0.04 0.06 0];
ax_3d_rot.YLabel.Units='normalized'; ax_3d_rot.YLabel.Position=ax_3d_rot.YLabel.Position+[+0.04 0.10 0];
ax_3d_rot.ZLabel.Units='normalized'; ax_3d_rot.ZLabel.Position=ax_3d_rot.ZLabel.Position+[+0.02 0.00 0];

ax_rot_err.YLabel.Position(1) = ax_pos_err.YLabel.Position(1);
ax_traj_typ.YLabel.Position(1) = ax_pos_err.YLabel.Position(1);

ax_traj_typ.Position = [ax_traj_typ.Position(1) ax_traj_typ.Position(2)+ax_traj_typ.Position(4)-0.03 ax_traj_typ.Position(3) 0.04];
ax_pos_err.Position  = ax_pos_err.Position + [0 -0.05 0 0];
ax_rot_err.Position  = ax_rot_err.Position + [0 -0.01 0 0];

if plot_3d == false
    delete(ax_3d_pos)
    delete(ax_3d_rot)
end

tightfig(fig);


%%
disp("saving figure...");
exportgraphics(gcf,BAG+'.eps','ContentType','vector');
exportgraphics(gcf,BAG+'.png','ContentType','image');
disp("DONE");


%%
% print statistics
err_pos = vecnorm(body_err_pos,2,2);
fprintf("pos: mean=%.1fmm, sd=%.1fmm, p90=%.1fmm\n", mean(err_pos), std(err_pos), prctile(err_pos, 90));
err_rot = vecnorm(body_err_rot_rv,2,2);
fprintf("rot: mean=%.1f°, sd=%.1f°, p90=%.1f°\n", mean(err_rot), std(err_rot), prctile(err_rot, 90));


%% utility
function [rv] = unmirror_rv(rv)
    rv = unwind_rv(rv);
    rv_abs = vecnorm(rv,2,2);
    rv_dir = rv./rv_abs;
    rv(rv_abs>pi,:) = rv(rv_abs>pi,:) - 2*pi*rv_dir(rv_abs>pi,:);
end
function [rv] = unwind_rv(rv)
    rv_abs = vecnorm(rv,2,2);
    rv_flips = find(vecnorm(diff(rv),2,2)>pi)+1;
    rv_mask = zeros(size(rv_abs));
    rv_mask(rv_flips(1:2:end)) = 1;
    rv_mask(rv_flips(2:2:end)) = -1;
    rv_mask = logical(cumsum(rv_mask));
    rv(rv_mask,:) = rv(rv_mask,:) - 2*pi*rv(rv_mask,:)./rv_abs(rv_mask);
end
function [rv] = match_rv(rv, rv_match)
    rv_abs = vecnorm(rv,2,2);
    rv_p = rv + 2*pi*rv./rv_abs;
    rv_m = rv - 2*pi*rv./rv_abs;
    rv_err = vecnorm(rv - rv_match, 2,2);
    rv_err_p = vecnorm(rv_p - rv_match, 2,2);
    rv_err_m = vecnorm(rv_m - rv_match, 2,2);
    rv(rv_err>rv_err_p,:) = rv_p(rv_err>rv_err_p,:);
    rv(rv_err>rv_err_m,:) = rv_m(rv_err>rv_err_m,:);
end



