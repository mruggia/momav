
uw  = 1.0;
upw = 400;
upl = 0.05;
uph = 0.95;
aw = 5e-4;
apw = 1e-2;
aph = 3.14;

u = linspace(0,1,200);
p_u = zeros(size(u));
for i = 1:length(u)
    p_u(i) = uw * u(i)*u(i);
    if u(i) < upl, p_u(i) = p_u(i) + upw * (u(i)-upl)*(u(i)-upl); end
    if u(i) > uph, p_u(i) = p_u(i) + upw * (u(i)-uph)*(u(i)-uph); end
end


a = linspace(-2*pi,2*pi,200);
p_a = zeros(size(a));
for i = 1:length(a)
    p_a(i) = aw * a(i)*a(i);
    if a(i) <-aph, p_a(i) = p_a(i) + apw * (a(i)+aph)*(a(i)+aph); end
    if a(i) > aph, p_a(i) = p_a(i) + apw * (a(i)-aph)*(a(i)-aph); end
end

fig = figure(1); clf(fig);
set(gcf,'Position',[500,200,600,400])

subplot(2,1,1);
plot(u,p_u);
title('\textbf{Motor Throttle Penalty} $p_u(u)$');
xlim([0 1]);   xticks(0:0.1:1);  xlabel('Motor Throttle $u$ [0-1]');
ylim([0 1.5]); yticks(0:0.5:2); ylabel('Penalty $p_u$');
grid minor; ax = gca;
tick_x = ax.XAxis.TickValues; ax.XAxis.MinorTickValues = (tick_x(1):(tick_x(2)-tick_x(1))/2:tick_x(end));
tick_y = ax.YAxis.TickValues; ax.YAxis.MinorTickValues = (tick_y(1):(tick_y(2)-tick_y(1))/2:tick_y(end));
h = gca; h.Position(2) = h.Position(2)+0.09;

subplot(2,1,2);
plot(a,p_a);
title('\textbf{Arm Velocity Penalty} $p_{\dot{a}}(\dot{a})$');
xlim([-2*pi 2*pi]); xticks(-2*pi:pi/2:2*pi); xlabel('Arm Velocity $\dot{a}$ [rad/s]');
ylim([0 0.12]);     yticks(0:0.04:0.12);     ylabel('Penalty $p_{\dot{a}}$');
xticklabels({'$-2\pi$','$-\frac{3}{2}\pi$','$-1\pi$','$-\frac{1}{2}\pi$','0','$\frac{1}{2}\pi$','$1\pi$','$\frac{3}{2}\pi$','$2\pi$'});
grid minor; ax = gca;
tick_x = ax.XAxis.TickValues; ax.XAxis.MinorTickValues = (tick_x(1):(tick_x(2)-tick_x(1))/2:tick_x(end));
tick_y = ax.YAxis.TickValues; ax.YAxis.MinorTickValues = (tick_y(1):(tick_y(2)-tick_y(1))/2:tick_y(end));

set(findall(gcf,'-property','FontSize'),'FontSize',13.5);
tightfig(fig);
exportgraphics(gcf,'3_sqp_penalty.eps','ContentType','vector');

