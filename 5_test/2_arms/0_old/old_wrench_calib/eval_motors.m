data = xlsread('data_70_2.csv');
N = size(data,1);

%% 
R = [
	[ 0.1543,  0.0000,  0.1091]
	[ 0.0772,  0.1336, -0.1091]
	[-0.0772,  0.1336,  0.1091]
	[-0.1543,  0.0000, -0.1091]
	[-0.0772, -0.1336,  0.1091]
	[ 0.0772, -0.1336, -0.1091]
];
X = [
	[ 0.816496,  0.000000,  0.577352]
	[ 0.408248,  0.707106, -0.577352]
	[-0.408248,  0.707106,  0.577352]
	[-0.816496,  0.000000, -0.577352]
	[-0.408248, -0.707106,  0.577352]
	[ 0.408248, -0.707106, -0.577352]
];
Z = [
	[-0.408248,  0.707107, 0.577350]
	[ 0.816497,  0.000000, 0.577350]
	[-0.408248, -0.707107, 0.577350]
	[-0.408248,  0.707107, 0.577350]
	[ 0.816497,  0.000000, 0.577350]
	[-0.408248, -0.707107, 0.577350]
];

%%
F = zeros(N,1);
T = zeros(N,1);
TnoF = zeros(N,1);
TnoFsym = zeros(N,1);

for i = 1:N
	f  = data(i,4:6);
	t = data(i,7:9);
	
	a = data(i,2);
	r = R(data(i,1)+1,:);
	x = X(data(i,1)+1,:); x = x/norm(x);
	z = Z(data(i,1)+1,:); z = z/norm(z);
	rot = rotationVectorToMatrix(-a*x);
	n = rot*z';
	
	F(i) = norm(f);
	T(i) = norm(t);
	TnoF(i) = dot(t,(f/norm(f)));
	TnoFsym(i) = dot(t,n);
end


hold on
plot(F);
plot(T);
plot(TnoF);
plot(TnoFsym);

legend('F','T', 'TnoF', 'TnoFsym');

F_avg = zeros(6,1);
for i = 1:6
	F_avg(i) = mean(F(data(:,1)==(i-1)));
end
F_avg