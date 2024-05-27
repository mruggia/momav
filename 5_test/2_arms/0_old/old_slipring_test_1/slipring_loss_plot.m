brush{1} = [
	1	0.25	0.21	0.21
	2	0.36	0.15	0.58
	3	0.43	0.11	0.96
	4	0.53	0.10	1.56
	5	0.55	0.07	1.85
];
brush{2} = [
	1	0.23	0.19	0.19
	2	0.36	0.15	0.58
	3	0.49	0.13	1.13
	4	0.54	0.10	1.60
	5	0.63	0.09	2.25
]; 
brush{3} = [
	1	0.26	0.22	0.22
	2	0.42	0.17	0.69
	3	0.51	0.13	1.19
	4	0.59	0.11	1.80
	5	0.72	0.11	2.68
];
brush{4} = [
	1	0.37	0.33	0.33
	2	0.48	0.21	0.82
	3	0.5 	0.13	1.17
	4	0.62	0.12	1.92
	5	0.65	0.09	2.35
];
brush{5} = [
	1	0.22	0.18	0.18
	2	0.32	0.13	0.50
	3	0.39	0.09	0.84
	4	0.47	0.08	1.32
	5	0.51	0.07	1.65
];
brush{6} = [
	1	0.37	0.33	0.33
	2	0.51	0.22	0.88
	3	0.61	0.17	1.50
	4	0.67	0.13	2.12
	5	0.73	0.11	2.75
];

figure('Position', [100, 100, 700, 610])
tiledlayout(3,1, 'Padding', 'none', 'TileSpacing', 'compact');
nexttile; hold on; grid on;
xticks([1 2 3 4 5]); xlim([0.75 5.25])
yticks([0 0.25 0.5 0.75 1]); ylim([0 1]); ylabel("Voltage [V]")
for i = 1:6
	plot(brush{i}(:,1),brush{i}(:,2),'.-', 'LineWidth',1.5, 'MarkerSize',10);
end
nexttile; hold on; grid on;
xticks([1 2 3 4 5]); xlim([0.75 5.25])
ylim([0 0.4]); ylabel("Resistance [Ω]")
for i = 1:6
	plot(brush{i}(:,1),brush{i}(:,3),'.-', 'LineWidth',1.5, 'MarkerSize',10);
end
nexttile; hold on; grid on;
xticks([1 2 3 4 5]); xlim([0.75 5.25]); xlabel("Current [A]")
ylim([0 4]); ylabel("Power [W]")
for i = 1:6
	plot(brush{i}(:,1),brush{i}(:,4),'.-', 'LineWidth',1.5, 'MarkerSize',10);
end
%legend("Brush 1","Brush 2","Brush 3","Brush 4","Brush 5","Brush 6", 'Location','southoutside','Orientation','horizontal')



