%%
r = [
	 154.3170,    0.0000,  109.1190
	  77.1585,  133.6424, -109.1190
	 -77.1585,  133.6424,  109.1190
	-154.3170,    0.0000, -109.1190
	 -77.1585, -133.6424,  109.1190
	  77.1585, -133.6424, -109.1190
]';
x = [	
	 0.816496,  0.000000,  0.577352
	 0.408248,  0.707106, -0.577352
	-0.408248,  0.707106,  0.577352
	-0.816496,  0.000000, -0.577352
	-0.408248, -0.707106,  0.577352
	 0.408248, -0.707106, -0.577352
]'; x = x./vecnorm(x);
z = [
	-0.408248,  0.707107, 0.577350
	 0.816497,  0.000000, 0.577350
	-0.408248, -0.707107, 0.577350
	-0.408248,  0.707107, 0.577350
	 0.816497,  0.000000, 0.577350
	-0.408248, -0.707107, 0.577350	
]'; z = z./vecnorm(z);
% cf = 2.2;
% cm = 3.3;
% qbF = [4.4, 5.5, 6.6]';
% qbM = [7.7, 8.8, 9.9]';
% w_u = 42.0;
% w_a = 24.0;
% ual_val = [ 1;2;3;4;5;6; 11;12;13;14;15;16; 21;22;23;24;25;26];
cf = 15.6;
cm = 0.177;
qbF = [0.0, 0.0, 2.2*9.81]';
qbM = [0.0, 0.0, 0.0]';
w_u = 1.0;
w_a = 0.0001;

% r = sym('r', [3,6], 'real');
% x = sym('x', [3,6], 'real');
% z = sym('z', [3,6], 'real');
% cf = sym('cf', 'real');
% cm = sym('cm', 'real');
% qbF = sym('qbF', [3,1], 'real');
% qbM = sym('qbM', [3,1], 'real');

a = sym('a', [6,1], 'real');
u = sym('u', [6,1], 'real');
l = sym('l', [6,1], 'real');

%%
n = sym(zeros(3,6));
n_a = sym(zeros(3,6));
n_aa = sym(zeros(3,6));
for i = 1:6
	q = [cos(a(i)/2), x(1,i)*sin(a(i)/2), x(2,i)*sin(a(i)/2), x(3,i)*sin(a(i)/2) ];
	C = [ q(1)^2+q(2)^2-q(3)^2-q(4)^2, 2*q(2)*q(3)-2*q(1)*q(4),     2*q(1)*q(3)+2*q(2)*q(4)
	      2*q(1)*q(4)+2*q(2)*q(3),     q(1)^2-q(2)^2+q(3)^2-q(4)^2, 2*q(3)*q(4)-2*q(1)*q(2)
	      2*q(2)*q(4)-2*q(1)*q(3),     2*q(1)*q(2)+2*q(3)*q(4),     q(1)^2-q(2)^2-q(3)^2+q(4)^2 ];
	n(:,i) = C*z(:,i);
	
	n_a(:,i) = cross(x(:,i),n(:,i));
	n_aa(:,i) = cross(x(:,i),cross(x(:,i),n(:,i)));
	
end

%%
f = sym(zeros(3,6));
f_u = f; f_uu = f; f_a = f; f_aa = f; f_ua = f;
m = sym(zeros(3,6));
m_u = m; m_uu = m; m_a = m; m_aa = m; m_ua = m;
for i = 1:6
	f(:,i) = cf*u(i)*n(:,i);
	
	f_u(:,i)  = cf*n(:,i);
	f_uu(:,i) = sym(zeros(3,1));
	f_a(:,i)  = cf*u(i)*n_a(:,i);
	f_aa(:,i) = cf*u(i)*n_aa(:,i);
	f_ua(:,i) = cf*n_a(:,i);
	
	sgn = mod(i+1,2)*2-1;
	m(:,i) = cf*u(i)*cross(r(:,i),n(:,i)) + sgn*cm*u(i)*n(:,i);
	
	m_u(:,i)  = cf*cross(r(:,i),n(:,i)) + sgn*cm*n(:,i);
	m_uu(:,i) = sym(zeros(3,1));
	m_a(:,i)  = cf*u(i)*cross(r(:,i),n_a(:,i)) + sgn*cm*u(i)*n_a(:,i);
	m_aa(:,i) = cf*u(i)*cross(r(:,i),n_aa(:,i)) + sgn*cm*u(i)*n_aa(:,i);
	m_ua(:,i) = cf*cross(r(:,i),n_a(:,i)) + sgn*cm*n_a(:,i);
	
end

%%
G = sym([-qbF; -qbM]);
G_u = sym(zeros(6,6));
G_a = sym(zeros(6,6));
G_uu = sym(zeros(6,6));
G_aa = sym(zeros(6,6));
G_ua = sym(zeros(6,6));
for i = 1:6
	G(1:3) = G(1:3) + f(:,i);
	G(4:6) = G(4:6) + m(:,i);
	
	G_u(:,i) = [ f_u(:,i); m_u(:,i) ];
	G_a(:,i) = [ f_a(:,i); m_a(:,i) ];
	G_uu(:,i) = [ f_uu(:,i); m_uu(:,i) ];
	G_aa(:,i) = [ f_aa(:,i); m_aa(:,i) ];
	G_ua(:,i) = [ f_ua(:,i); m_ua(:,i) ];
	
end

% G_u_sym = sym(zeros(6,6));
% G_a_sym = sym(zeros(6,6));
% for i = 1:6
% 	for j = 1:6
% 		G_u_sym(i,j) = diff(G(i),u(j));
% 		G_a_sym(i,j) = diff(G(i),a(j));
% 	end
% end
% G_u_err = vpa(subs(G_u, [u;a;l], ual_val)) - vpa(subs(G_u_sym, [u;a;l], ual_val))
% G_a_err = vpa(subs(G_a, [u;a;l], ual_val)) - vpa(subs(G_a_sym, [u;a;l], ual_val))

%%
O = w_u*u'*u + w_a*a'*a;

O_u = 2*w_u*u;
O_a = 2*w_a*a;
O_uu = sym(2*w_u*eye(6));
O_aa = sym(2*w_a*eye(6));
O_ua = sym(zeros(6,6));

%%
L = O+l'*G;

L_u = O_u + G_u'*l;
L_a = O_a + G_a'*l;
L_l = G;

L_uu = O_uu;
L_aa = O_aa;
L_ua = O_ua;
L_ll = sym(zeros(6,6));
for i = 1:6
	L_uu(i,i) = L_uu(i,i) + G_uu(:,i)'*l;
	L_aa(i,i) = L_aa(i,i) + G_aa(:,i)'*l;
	L_ua(i,i) = L_ua(i,i) + G_ua(:,i)'*l;
end
L_ul = G_u';
L_al = G_a';

%%

ddL = [L_uu,  L_ua,  L_ul;
 	   L_ua', L_aa,  L_al;
 	   L_ul', L_al', L_ll];
 
dL = [L_u; L_a; L_l];

H = ddL;
K = [O_u; O_a; G];

%%

% dL_sym = sym(zeros(18,1));
% ddL_sym = sym(zeros(18,18));
% 
% ual = [u;a;l];
% ual_val = [1;2;3;4;5;6; 7;8;9;10;11;12; 13;14;15;16;17;18];
% 
% for i = 1:18
% 	dL_sym(i) = diff(L,ual(i));
% end
% for i = 1:18
% 	for j = 1:18
% 		ddL_sym(i,j) = diff(dL_sym(i), ual(j));
% 	end
% end
% 
% dL_err = double(subs(dL, ual, ual_val)) - double(subs(dL_sym, ual, ual_val))
% ddL_err = double(subs(ddL, ual, ual_val)) - double(subs(ddL_sym, ual, ual_val))


%%

% ual_val = [ 1;2;3;4;5;6; 11;12;13;14;15;16; 21;22;23;24;25;26];
% ual_val = [0.5;0.5;0.5;0.5;0.5;0.5; 0;0;0;0;0;0; 0;0;0;0;0;0];
% ual_val = [0.8024; 1.4244;-0.6079; 0.8037; 1.4234;-0.6077; 0.0743;-0.3902;-0.6455;-0.0744; 0.3890; 0.6442;-0.5495;-0.4127;-1.1333;-0.0000;-0.0000;-0.0000];
% ual_val = [0.8024; 1.4244;-0.6079; 0.8037; 1.4234;-0.6077; 0.0743;-0.3902;-0.6455;-0.0744; 0.3890; 0.6442; 0;0;0;0;0;0];
% ual_val = [ 0;0;0;0;0;0; 0.0743;-0.3902;-0.6455;-0.0744; 0.3890; 0.6442; 0;0;0;0;0;0];
ual_val = [ 0;0;0;0;0;0; 0;0;0;0;0;0; 0;0;0;0;0;0];

O_eval_old = inf;
tic
for i = 1:20
	H_eval = double(subs(H, [u;a;l], ual_val));
	K_eval = double(subs(K, [u;a;l], ual_val));
	d = H_eval\(-K_eval);
	d(13:18) = d(13:18) - ual_val(13:18);
	
	d_u_fact = 0.2 / max(abs(d(1:6)));
	d_a_fact = 0.5 / max(abs(d(7:12)));
	alpha = min([ 1.0; d_u_fact; d_a_fact]);
	
	ual_val = ual_val + alpha*d;
	
	O_eval = double(subs(O, [u;a;l], ual_val));
	G_eval = norm(double(subs(G, [u;a;l], ual_val)));
	
	fprintf('iter:  %d\n', i)
	fprintf('alpha: %f\n', alpha)
	fprintf('cost:  %f\n', O_eval)
	fprintf('constr:%f\n', G_eval)
	fprintf('--------------------\n')
	
	if(G_eval < 1e-8 && abs(O_eval_old-O_eval)/O_eval_old < 1e-8)
		break
	end
	O_eval_old = O_eval;
	
end
toc