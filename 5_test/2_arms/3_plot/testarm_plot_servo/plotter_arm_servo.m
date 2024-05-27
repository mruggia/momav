data = readtable('data.csv');

time = data.com_ts / 1000000;
pos  = data.srv_pos*180/pi;
setp = data.srv_setp*180/pi;

N = 800;
Ns = [
    401
    1205
    2012 % 2010
    2815
    3821
    4724
    5628
    6531
];

time_step_7   = time(Ns(1):Ns(1)+N)-time(Ns(1)); pos_step_7   = pos(Ns(1):Ns(1)+N); setp_step_7   = setp(Ns(1):Ns(1)+N);
time_step_15  = time(Ns(2):Ns(2)+N)-time(Ns(2)); pos_step_15  = pos(Ns(2):Ns(2)+N); setp_step_15  = setp(Ns(2):Ns(2)+N);
time_step_30  = time(Ns(3):Ns(3)+N)-time(Ns(3)); pos_step_30  = pos(Ns(3):Ns(3)+N); setp_step_30  = setp(Ns(3):Ns(3)+N);
time_step_180 = time(Ns(4):Ns(4)+N)-time(Ns(4)); pos_step_180 = pos(Ns(4):Ns(4)+N); setp_step_180 = setp(Ns(4):Ns(4)+N);
time_chirp_7  = time(Ns(5):Ns(5)+N)-time(Ns(5)); pos_chirp_7  = pos(Ns(5):Ns(5)+N); setp_chirp_7  = setp(Ns(5):Ns(5)+N);
time_chirp_15 = time(Ns(6):Ns(6)+N)-time(Ns(6)); pos_chirp_15 = pos(Ns(6):Ns(6)+N); setp_chirp_15 = setp(Ns(6):Ns(6)+N);
time_chirp_30 = time(Ns(7):Ns(7)+N)-time(Ns(7)); pos_chirp_30 = pos(Ns(7):Ns(7)+N); setp_chirp_30 = setp(Ns(7):Ns(7)+N);
time_chirp_60 = time(Ns(8):Ns(8)+N)-time(Ns(8)); pos_chirp_60 = pos(Ns(8):Ns(8)+N); setp_chirp_60 = setp(Ns(8):Ns(8)+N);

%%

fig = figure(1); clf(fig);
set(gcf,'Position',[500,200,600,900])

subplot(3,1,1); hold on;
time_step_30 = time_step_30(180:180+200)-time_step_30(200); 
pos_step_30  = pos_step_30 (180:180+200); 
setp_step_30 = setp_step_30(180:180+200);
plot(time_step_30, setp_step_30);
plot(time_step_30, pos_step_30);
plot(time_step_30, pos_step_30-setp_step_30);
xlim([-0.2, 1.8]); xticks(-0.2:0.2:1.8); 
ylim([-60 60]); yticks(-60:30:60);
grid minor; ax = gca;
tick_x = ax.XAxis.TickValues; ax.XAxis.MinorTickValues = (tick_x(1):(tick_x(2)-tick_x(1))/2:tick_x(end));
tick_y = ax.YAxis.TickValues; ax.YAxis.MinorTickValues = (tick_y(1):(tick_y(2)-tick_y(1))/2:tick_y(end));
xlabel("Time [s]"); ylabel("Angle [$^{\circ}$]");
legend(["Setpoint","Position","Error"], 'Location','northeast')
title('(A) \ \textbf{Arm Actuator - 60$\mathbf{^{\circ}}$ Step Response}');
h = gca; h.Position(2) = h.Position(2)+0.008;

Fs = 100; L = 800;
freq_chirp = Fs/L*(0:L-1);
fft_chirp_7  = fft(pos_chirp_7(1:end-1))  ./ fft(setp_chirp_7(1:end-1));
fft_chirp_15 = fft(pos_chirp_15(1:end-1)) ./ fft(setp_chirp_15(1:end-1));
fft_chirp_30 = fft(pos_chirp_30(1:end-1)) ./ fft(setp_chirp_30(1:end-1));
fft_chirp_60 = fft(pos_chirp_60(1:end-1)) ./ fft(setp_chirp_60(1:end-1));

subplot(3,1,2); hold on;
plot(freq_chirp,db(abs(fft_chirp_7)));
plot(freq_chirp,db(abs(fft_chirp_15)));
plot(freq_chirp,db(abs(fft_chirp_30)));
plot(freq_chirp,db(abs(fft_chirp_60)));
xlim([1, 8]); xticks(1:1:8); set(gca,'Xticklabel',[]);
ylim([-12, 4]); yticks(-12:4:4);
grid minor; ax = gca;
tick_x = ax.XAxis.TickValues; ax.XAxis.MinorTickValues = (tick_x(1):(tick_x(2)-tick_x(1))/2:tick_x(end));
tick_y = ax.YAxis.TickValues; ax.YAxis.MinorTickValues = (tick_y(1):(tick_y(2)-tick_y(1))/2:tick_y(end));
ylabel("Magnitude [dB]");
legend(["$\pm7.5^{\circ}$ Chirp","$\pm15^{\circ}$ Chirp","$\pm30^{\circ}$ Chirp","$\pm60^{\circ}$ Chirp"], 'Location','southwest')
title('(B) \ \textbf{Arm Actuator - Bode Plot}');

subplot(3,1,3); hold on;
plot(freq_chirp,angle(fft_chirp_7)*180/pi);
plot(freq_chirp,angle(fft_chirp_15)*180/pi);
plot(freq_chirp,angle(fft_chirp_30)*180/pi);
plot(freq_chirp,angle(fft_chirp_60)*180/pi);
xlim([1, 8]); xticks(1:1:8);
ylim([-180, 0]); yticks(-180:45:0);
grid minor; ax = gca;
tick_x = ax.XAxis.TickValues; ax.XAxis.MinorTickValues = (tick_x(1):(tick_x(2)-tick_x(1))/2:tick_x(end));
tick_y = ax.YAxis.TickValues; ax.YAxis.MinorTickValues = (tick_y(1):(tick_y(2)-tick_y(1))/2:tick_y(end));
xlabel("Frequency [Hz]"); ylabel("Phase [$^{\circ}$]");
legend(["$\pm7.5^{\circ}$ Chirp","$\pm15^{\circ}$ Chirp","$\pm30^{\circ}$ Chirp","$\pm60^{\circ}$ Chirp"], 'Location','southwest')
h = gca; h.Position(2) = h.Position(2)+0.065;


set(findall(gcf,'-property','FontSize'),'FontSize',13.5);
axis_h = findobj('Type','Axes'); for i = 1:length(axis_h), axis_h(i).Position(4) = 0.21; end
tightfig(fig);
exportgraphics(gcf,'2_servo_dynamics.eps','ContentType','vector');
