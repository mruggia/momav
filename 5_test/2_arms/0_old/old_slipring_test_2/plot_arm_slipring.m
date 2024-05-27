
%% load slipring data

v_slip = []; i_slip = [];

for i = 1:15
    name = strcat('logs/',string(i),'A.csv');
    data = readtable(name, 'VariableNamingRule','preserve');
    data = data.Reading;
    mask = data>0.1;
    if i>=13, mask = data>1.4; end
    mask = mask & circshift(mask,1) & circshift(mask,-1);
    data = data(mask);
    v_slip = [v_slip;data];
    i_slip = [i_slip;(0*data+i)];
end

r_slip = v_slip./i_slip;
p_slip = v_slip.*i_slip;

%% load motor data

% https://store.tmotor.com/product/f100-fpv-motor.html
% F100 1100KV GF7040-3
throt = [0.30; 0.35; 0.40; 0.45; 0.50; 0.55; 0.60; 0.65; 0.70; 0.75; 0.80; 0.85; 0.90; 0.95; 1.00];
thrust = [401.76; 474.60; 557.59; 627.53; 714.03; 786.22; 870.11; 971.07; 1104.05; 1252.2; 1398.93; 1523.04; 1676.6; 1869.5; 2033.89];
rpm = [9515; 10387; 11099; 11846; 12541; 13259; 13890; 14713; 15622; 16597; 17425; 18245; 19003; 19765; 20433];
p_mot = [61.66; 82.04; 102.60; 124.58; 149.20; 174.17; 196.22; 232.76; 283.44; 345.97; 406.21; 466.24; 536.15; 615.85; 699.92];
v_bat = [25.16; 25.14; 25.13; 25.11; 25.09; 25.07; 25.05; 25.02; 24.98; 24.92; 24.87; 24.81; 24.75; 24.67; 24.59];
i_bat = [2.45; 3.26; 4.08; 4.96; 5.95; 6.95; 7.83; 9.30; 11.35; 13.88; 16.33; 18.79; 21.66; 24.96; 28.47];
i_mot = i_bat./throt;

% calculate hover setpoint
hover_thrust = 2500/4; %worst case hover thrust
hover_throt = interp1(thrust,throt,hover_thrust);
hover_i_mot = round(interp1(thrust,i_mot,hover_thrust));
hover_i_bat = round(interp1(thrust,i_bat,hover_thrust));

% resample p_mot with i_mot=1:15 and p_bat with i_bat=1:15
p_bat = interp1(i_bat,p_mot,(1:15)', 'linear','extrap');
p_mot = interp1(i_mot,p_mot,(1:15)', 'linear','extrap');

% calculate percentage slipring loss
loss_mot = p_slip./p_mot(i_slip)*100;
loss_bat = p_slip./p_bat(i_slip)*100;
loss_mot(i_slip<8) = NaN; % trim out extrapolated data
%loss_mot(i_slip<2) = NaN; % trim out extrapolated data

%% plot slipring data

fig = figure(1); clf(fig);
set(gcf,'Position',[500,200,1200,800])

h1 = subplot(2,2,1); hold on;
boxplot(r_slip,i_slip, 'Whisker',100,'PlotStyle','compact','Colors',[0 0.4470 0.7410]);
xlim([0, 16]); xticks(0:16); xticklabels(0:16);
ylim([0, 1]);  yticks(0:0.2:1); ylabel("Resistance [$\Omega$]");
text(0.5,-0.21,'Current [A]', 'HorizontalAlignment','center', 'Units','normalized','Color',[0.15 0.15 0.15]);
title('(A) \ \textbf{Total resistance across two slip-rings}');

h2 = subplot(2,2,3); hold on;
boxplot(p_slip,i_slip, 'Whisker',100,'PlotStyle','compact','Colors',[0.8500 0.3250 0.0980]);
xlim([0, 16]); xticks(0:16); xticklabels(0:16);
ylim([0, 40]);  yticks(0:5:40); ylabel("Power loss [W]");
text(0.5,-0.21,'Current [A]', 'HorizontalAlignment','center', 'Units','normalized','Color',[0.15 0.15 0.15]);
title('(B) \ \textbf{Total power loss  across two slip-rings}');

h3 = subplot(2,2,2); hold on;
plot([hover_i_mot hover_i_mot],[0 999], 'Color',[0.4 0.4 0.4]);
boxplot(loss_mot,i_slip, 'Whisker',100,'PlotStyle','compact','Colors',[0.9290 0.6940 0.1250]);
xlim([0, 16]); xticks(0:16); xticklabels(0:16);
ylim([8, 24]);  yticks(8:2:24); ylabel("Power loss [\%]");
text(0.5,-0.21,'Motor Current [A]', 'HorizontalAlignment','center', 'Units','normalized','Color',[0.15 0.15 0.15]);
text(hover_i_mot/16,0.85,'\textbf{Hover}', 'HorizontalAlignment','center', 'Units','normalized','BackgroundColor','w','Color',[0.15 0.15 0.15]);
text(3.5/16,0.5,{'\textbf{No Data}','\textbf{Available}'}, 'HorizontalAlignment','center', 'Units','normalized','BackgroundColor','w','Color',[0.15 0.15 0.15]);
title({'(C) \ \textbf{Percentage power loss relative to motor power}','\textbf{with slip-rings in front of motor}'});

h4 = subplot(2,2,4); hold on;
plot([hover_i_bat hover_i_bat],[0 999], 'Color',[0.4 0.4 0.4]);
boxplot(loss_bat,i_slip, 'Whisker',100,'PlotStyle','compact','Colors',[0.4660 0.6740 0.1880]);
xlim([0, 16]); xticks(0:16); xticklabels(0:16);
ylim([0, 12]);  yticks(0:2:12); ylabel("Power loss [\%]");
text(0.5,-0.21,'Battery Current [A]', 'HorizontalAlignment','center', 'Units','normalized','Color',[0.15 0.15 0.15]);
text(hover_i_bat/16,0.85,'\textbf{Hover}', 'HorizontalAlignment','center', 'Units','normalized','BackgroundColor','w','Color',[0.15 0.15 0.15]);
title({'(D) \ \textbf{Percentage power loss relative to motor power}','\textbf{with slip-rings in front of battery}'});


%% finalize figures

h1.Position(4) = 0.23; h1.Position(2) = 0.06+0.33+0.025;
h2.Position(4) = 0.23; h2.Position(2) = 0.06;
h3.Position(4) = 0.23; h3.Position(2) = 0.06+0.33+0.025;
h4.Position(4) = 0.23; h4.Position(2) = 0.06;

h1.Position(3)=h1.Position(3)+0.021;
h2.Position(3)=h2.Position(3)+0.021;
h3.Position(1)=h3.Position(1)-0.043; h3.Position(3)=h3.Position(3)+0.043;
h4.Position(1)=h4.Position(1)-0.043; h4.Position(3)=h4.Position(3)+0.043;

ax = axes('Position',[0.5 0.03 0 0]); ax.XColor='w'; ax.YColor='w';

box_h = findobj('Tag','Box');             for i = 1:length(box_h),     set(box_h(i), 'LineWidth',14); end
whisker_h = findobj('Tag','Whisker');     for i = 1:length(whisker_h), set(whisker_h(i), 'LineWidth',4); end
median1_h = findobj('Tag','MedianInner'); for i = 1:length(median1_h), set(median1_h(i), 'MarkerSize',8); end
median2_h = findobj('Tag','MedianOuter'); for i = 1:length(median2_h), set(median2_h(i), 'MarkerSize',8); end

set(findall(fig,'-property','FontSize'),'FontSize',12);
tightfig(fig);

exportgraphics(fig,'2_slipring_loss.eps','ContentType','vector');
