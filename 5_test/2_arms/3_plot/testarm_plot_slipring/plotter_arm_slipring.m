
%  1: com_ts
%  2: srv_pos
%  3: srv_setp
%  4: srv_volt
%  5: esc_setp
%  6: esc_volt
%  7: esc_curr
%  8: esc_rpm
%  9: bota_fx
% 10: bota_fy
% 11: bota_fz
% 12: bota_mx
% 13: bota_my
% 14: bota_mz
% 15: bota_f_n
% 16: bota_t_n

% 17: motor power
% 18: motor efficiency
% 19: slipring consumption

% calculate hover thrust
global hover1 hover2;
hover1 = 2.4*9.81/6/0.67; % worst case hover thrust
hover2 = 2.4*9.81/6/0.82; % best case hover thrust

% load test data
data_nonslip = load_data('001');
data_motslip = load_data('002');
data_batslip = load_data('003');

% replace voltage with fixed number, so batslip becomes comparable to the other two
data_nonslip(:,6) = 23.0;
data_motslip(:,6) = 23.0;
data_batslip(:,6) = 23.0;

% fix outliers (outside of plotted range)
for i=1:16
    % interpolate force/torque data for 60% and 65% throttle, since vibration made thrust/torque readings unreliable
    if i>=9 && i<=16
        mask = (1:13==9 | 1:13==10);
        data_nonslip(mask,i) = interp1(data_nonslip(~mask,5),data_nonslip(~mask,i),[60;65],'pchip');
        data_batslip(mask,i) = interp1(data_batslip(~mask,5),data_batslip(~mask,i),[60;65],'pchip');
        data_batslip(mask,i) = interp1(data_batslip(~mask,5),data_batslip(~mask,i),[60;65],'pchip');
    end
    % extrapolate 80% throttle for data_batslip, since arm motor broke before end of tests
    data_batslip(end,i) = interp1(data_batslip(1:end-1,5),data_batslip(1:end-1,i),80,'linear','extrap');
end

% calculate motor power
data_nonslip(:,17) = data_nonslip(:,6).*data_nonslip(:,7);
data_motslip(:,17) = data_motslip(:,6).*data_motslip(:,7);
data_batslip(:,17) = data_batslip(:,6).*data_batslip(:,7);
% calculate motor efficiency
data_nonslip(:,18) = data_nonslip(:,15)./data_nonslip(:,17);
data_motslip(:,18) = data_motslip(:,15)./data_motslip(:,17);
data_batslip(:,18) = data_batslip(:,15)./data_batslip(:,17);
% calculate overconsumption
for id = 1:size(data_nonslip,1)
    % nonslip
    data_nonslip(id,19) = 100;
    % motslip
    power_slip   = data_motslip(id,17);
    power_noslip = interp1(data_nonslip(:,15),data_nonslip(:,17),data_motslip(id,15),'linear','extrap');
    overconsumption = (100 * power_noslip / power_slip);
    data_motslip(id,19) = overconsumption;
    % batslip
    power_slip   = data_batslip(id,17);
    power_noslip = interp1(data_nonslip(:,15),data_nonslip(:,17),data_batslip(id,15),'linear','extrap');
    overconsumption = (100 * power_noslip / power_slip);
    data_batslip(id,19) = overconsumption;
end

%%

close all;
fig = figure(1);
set(gcf,'Position',[500,100,1200,540])
newcolors = colororder;
newcolors = [ newcolors(1,:); newcolors(2,:); newcolors(3,:); [1 1 1]; [0.6 0.6 0.6] ];
newcolors = min(1.0, newcolors);
colororder(newcolors)

h1 = subplot(2,2,1); hold on;
title('(A) \ \textbf{Motor Throttle}');
ylim([20, 60]); yticks(20:10:60); ylabel('Throttle [$\%$]');
plot(data_nonslip(:,15),data_nonslip(:,5));
plot(data_motslip(:,15),data_motslip(:,5));
plot(data_batslip(:,15),data_batslip(:,5));
plot(nan,nan);plot(nan,nan);
plot(data_nonslip(:,15),data_nonslip(:,5),'o');
plot(data_motslip(:,15),data_motslip(:,5),'x');
plot(data_batslip(:,15),data_batslip(:,5),'x');
plot_finalize(1);


h2 = subplot(2,2,2); hold on;
title('(B) \ \textbf{Electrical Power Draw}');
ylim([0, 240]); yticks(0:60:240); ylabel('Power [$W$]');
plot(data_nonslip(:,15),data_nonslip(:,17));
plot(data_motslip(:,15),data_motslip(:,17));
plot(data_batslip(:,15),data_batslip(:,17));
plot(nan,nan);plot(nan,nan);
plot(data_nonslip(:,15),data_nonslip(:,17),'o');
plot(data_motslip(:,15),data_motslip(:,17),'x');
plot(data_batslip(:,15),data_batslip(:,17),'x');
plot_finalize(2);

h3 = subplot(2,2,3); hold on;
title('(C) \ \textbf{Thrust Efficiency}');
ylim([0.04, 0.12]); yticks(0.04:0.02:0.12); ylabel('Efficiency [$N/W$]'); ytickformat("%.2f");
plot(data_nonslip(:,15),data_nonslip(:,18));
plot(data_motslip(:,15),data_motslip(:,18));
plot(data_batslip(:,15),data_batslip(:,18));
plot(nan,nan);plot(nan,nan);
plot(data_nonslip(:,15),data_nonslip(:,18),'o');
plot(data_motslip(:,15),data_motslip(:,18),'x');
plot(data_batslip(:,15),data_batslip(:,18),'x');
plot_finalize(3);

h4 = subplot(2,2,4); hold on;
title('(D) \ \textbf{Relative Thrust Efficiency}');
ylim([85, 110]); yticks(85:5:110); ylabel('Relative Efficiency [$\%$]');
plot(data_nonslip(:,15),data_nonslip(:,19));
plot(data_motslip(:,15),data_motslip(:,19));
plot(data_batslip(:,15),data_batslip(:,19));
plot(nan,nan);plot(nan,nan);plot(nan,nan);
plot(data_motslip(:,15),data_motslip(:,19),'x');
plot(data_batslip(:,15),data_batslip(:,19),'x');
plot_finalize(4);

set(findall(fig,'-property','FontSize'),'FontSize',12);
plot_legend();
tightfig(fig);
plot_noslip_arrow(h1,data_nonslip(:,15),data_nonslip(:,5));
plot_noslip_arrow(h2,data_nonslip(:,15),data_nonslip(:,17));
exportgraphics(fig,'2_slipring_loss.eps','ContentType','vector');

%%

function plot_finalize(id)
    ax = gca;
    
    % hover lines
    global hover1 hover2;
    plot(nan, nan);
    plot([hover1 hover1 nan hover2 hover2], [-1e9 +1e9 nan -1e9 +1e9]);
    y_label = interp1([0 1], ax.YLim, 0.84);
    x1_label = hover1;
    x2_label = hover2;
    text(x1_label, y_label,{'\boldmath$X_{2,max}$','\textbf{hover}'}, HorizontalAlignment='center', Color=[0.2 0.2 0.2], BackgroundColor=[1 1 1])
    text(x2_label, y_label,{'\boldmath$X_{2,min}$','\textbf{hover}'}, HorizontalAlignment='center', Color=[0.2 0.2 0.2], BackgroundColor=[1 1 1])

    % axis labels & ticks
    xlim([2, 9]);
    xticks(2:1:9);
    xlabel("Thrust [$N$]");
    grid minor;
    tick_x = ax.XAxis.TickValues; ax.XAxis.MinorTickValues = (tick_x(1):(tick_x(2)-tick_x(1))/2:tick_x(end));
    tick_y = ax.YAxis.TickValues; ax.YAxis.MinorTickValues = (tick_y(1):(tick_y(2)-tick_y(1))/2:tick_y(end));
    
    % reposition plot
    widen = 0.017;
    ax.Position(4)=ax.Position(4)-0.01;
    if mod(id,2) == 1, ax.Position(3)=ax.Position(3)+widen;
    else, ax.Position(1)=ax.Position(1)-widen; ax.Position(3)=ax.Position(3)+widen; end

end

function plot_noslip_arrow(ax, X, Y)
    x1_arrow = 0.74;
    x2_arrow = x1_arrow + 0.04;
    y1_arrow = interp1(ax.YLim, [0 1], interp1(X, Y, interp1([0 1], ax.XLim, 0.74))) - 0.04;
    y2_arrow = y1_arrow - 0.14;

    pos = ax.Position;
    x1_arrow = interp1([0 1], [pos(1), pos(1)+pos(3)], x1_arrow);
    x2_arrow = interp1([0 1], [pos(1), pos(1)+pos(3)], x2_arrow);
    y1_arrow = interp1([0 1], [pos(2), pos(2)+pos(4)], y1_arrow);
    y2_arrow = interp1([0 1], [pos(2), pos(2)+pos(4)], y2_arrow);
    
    co = colororder;
    annotation('arrow', [x2_arrow x1_arrow], [y2_arrow y1_arrow], HeadStyle='vback1', HeadWidth=5, HeadLength=5, Color=co(1,:));
end

function plot_legend()
    
    % place hover axes in background
    axs = findall(gcf,'type','axes');
    for i = 1:length(axs)
        lines = get(axs(i),'Children');
        uistack(lines(3),'bottom');
    end
    
    % create axis for legend
    ax2_sz = [0.5 0.5 0 0];
    for i = 1:length(axs)
        ax_sz = axs(i).Position;
        if ax_sz(1) < ax2_sz(1), ax2_sz(1) = ax_sz(1); end
        if ax_sz(2) < ax2_sz(2), ax2_sz(2) = ax_sz(2); end
        if ax_sz(1)+ax_sz(3) > ax2_sz(3), ax2_sz(3) = ax_sz(1)+ax_sz(3); end
        if ax_sz(2)+ax_sz(4) > ax2_sz(4), ax2_sz(4) = ax_sz(2)+ax_sz(4); end
    end
    ax2_sz(3) = ax2_sz(3) - ax2_sz(1);
    ax2_sz(4) = ax2_sz(4) - ax2_sz(2);
    ax2_sz(2) = ax2_sz(2) - 0.15;
    ax2_sz(4) = 0.1;

    % populate legend
    ax2 = axes(gcf, 'Position',ax2_sz); hold(ax2,'on'); grid(ax2,'off');
    set(ax2, 'Color','none','XColor','none','YColor','none');
    plot(nan,nan, '-o');
    plot(nan,nan, '-x');
    plot(nan,nan, '-x');
    plot(nan,nan);
    legend_labels = { ...
        '\ No slip-rings \enspace', ...
        '\ Slip-rings after motor driver \enspace', ...
        '\ Slip-rings before motor driver \enspace', ...
        '\ Hover thrust at $X_{2,min}$, $X_{2,max}$' ...
    };
    l = legend(legend_labels, 'Location','southoutside', 'AutoUpdate','off', 'NumColumns',4, 'FontSize',axs(1).FontSize);
    l.Position(1) = 0.5 - l.Position(3)/2;
    
    ax3 = axes(gcf, 'Position',ax2.Legend.Position); hold(ax3,'on'); grid(ax3,'off');
    set(ax3, 'Color','none','XColor','none','YColor','none');
    set(ax3, 'XLim',[0 1], 'YLim',[0 1]);
    line_mid = [0.747, 0.5]; line_sz = [0.021 0.15];
    line_x = [ ...
        line_mid(1)-line_sz(1), line_mid(1)+line_sz(1), nan, ...
        line_mid(1)-line_sz(1), line_mid(1)+line_sz(1), ...
    ];
    line_y = [ ...
        line_mid(2)-line_sz(2), line_mid(2)-line_sz(2), nan, ...
        line_mid(2)+line_sz(2), line_mid(2)+line_sz(2), ...
    ];
    plot(nan,nan);plot(nan,nan);plot(nan,nan);plot(nan,nan);
    plot(line_x, line_y);

end

function data = load_data(file)
    
    % load data
    data_all = readtable(strcat('logs/',file,'.csv'));
    data_all = data_all{:,:};
    data_throttles = unique(data_all(:,5));
    data = cell(length(data_throttles), 1);
    for i = 1:length(data_throttles)
        data{i} = data_all(data_all(:,5)==data_throttles(i),:);
    end
    
    % filter data
    Fs = 100;
    for i = 1:length(data_throttles)
        data{i} = lowpass(data{i}, 0.5, Fs, ...   % lowpass filter 1Hz
            ImpulseResponse="fir", ...
            Steepness=0.98 ...
        );
        data{i} = data{i}(Fs:end-Fs,:);           % trim first/last second before ramp starts
    end
    
    % calculate derived quantities
    for i = 1:length(data_throttles)
        data{i}(:,5) = 100.*data{i}(:,5);
        normals = [0*data{i}(:,9),data{i}(:,10),data{i}(:,11)];
        normals = normals ./ vecnorm(normals,2,2);
        data{i}(:,15) = dot(data{i}(:,9:11), normals, 2);
        data{i}(:,16) = dot(data{i}(:,12:14), -normals, 2);
    end
    
    % average results
    data_full = data; data = zeros(length(data_throttles), 16);
    for i = 1:length(data_throttles)
        data(i,:) = mean(data_full{i});
    end

end
