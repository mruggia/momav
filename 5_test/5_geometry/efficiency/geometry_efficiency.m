
% tetrahedron arm axes
X_tet = [
	normalize([0.00000001, 0.0, 0.61237244],'norm')
	normalize([0.57735027, 0.0, -0.20412415],'norm')
	normalize([-0.28867513, 0.5, -0.20412415],'norm')
	normalize([-0.28867513, -0.5, -0.20412415],'norm')
]';

% octahedron arm axes
X_oct = [
	normalize([ 0.1543,  0.0000,  0.1091],'norm')
	normalize([ 0.0772,  0.1336, -0.1091],'norm')
	normalize([-0.0772,  0.1336,  0.1091],'norm')
	normalize([-0.1543,  0.0000, -0.1091],'norm')
	normalize([-0.0772, -0.1336,  0.1091],'norm')
	normalize([ 0.0772, -0.1336, -0.1091],'norm')
]';

% cube arm axes
X_cub = [
	normalize([+1 +1 +1],'norm')
	normalize([-1 +1 +1],'norm')
	normalize([+1 -1 +1],'norm')
	normalize([-1 -1 +1],'norm')
	normalize([+1 +1 -1],'norm')
	normalize([-1 +1 -1],'norm')
	normalize([+1 -1 -1],'norm')
	normalize([-1 -1 -1],'norm')
]';

% hexagon arm axes
X_hex = [
	normalize([+1 0 0],'norm')
    normalize([+0.5 +sqrt(3)/2 0],'norm')
    normalize([-0.5 +sqrt(3)/2 0],'norm')
    normalize([-1 0 0],'norm')
    normalize([-0.5 -sqrt(3)/2 0],'norm')
    normalize([+0.5 -sqrt(3)/2 0],'norm')
]';

% square arm axes
X_sqr = [
	normalize([+1 +1 0],'norm')
    normalize([+1 -1 0],'norm')
    normalize([-1 -1 0],'norm')
    normalize([-1 +1 0],'norm')
]';

% odar arm axes
X_odar = [
	normalize([+0.40 -0.17 +0.17],'norm')
    normalize([+0.40 +0.17 +0.17],'norm')
    normalize([+0.40 -0.17 -0.17],'norm')
    normalize([+0.40 +0.17 -0.17],'norm')
    normalize([-0.40 -0.17 +0.17],'norm')
    normalize([-0.40 +0.17 +0.17],'norm')
    normalize([-0.40 -0.17 -0.17],'norm')
    normalize([-0.40 +0.17 -0.17],'norm')
]';

% heptagon arm axes
X_hep = zeros([3 7]);
for i = 1:7
	X_hep(:,i) = rotationVectorToMatrix( [0;0;1]*(-2*pi/7*(i-1)) )*[1;0;0];
end

%% 

% tetrahedron variable thrust axes at 0° rotation
N_tet_rot = zeros([3 8]);
for i=1:4
    N_tet_rot(:,(i-1)*2+1) = normalize( cross(X_tet(:,i), [0;0;1]), 'norm');
    N_tet_rot(:,(i-1)*2+2) = normalize( cross(cross(X_tet(:,i), [0;0;1]),X_tet(:,i)), 'norm');
end

% octahedron variable thrust axes at 0° rotation
N_oct_rot = zeros([3 12]);
for i=1:6
    N_oct_rot(:,(i-1)*2+1) = normalize( cross(X_oct(:,i), [0;0;1]), 'norm');
    N_oct_rot(:,(i-1)*2+2) = normalize( cross(cross(X_oct(:,i), [0;0;1]),X_oct(:,i)), 'norm');
end

% cube variable thrust axes at 0° rotation
N_cub_rot = zeros([3 16]);
for i=1:8
    N_cub_rot(:,(i-1)*2+1) = normalize( cross(X_cub(:,i), [0;0;1]), 'norm');
    N_cub_rot(:,(i-1)*2+2) = normalize( cross(cross(X_cub(:,i), [0;0;1]),X_cub(:,i)), 'norm');
end

% hexagon variable thrust axes at 0° rotation
N_hex_rot = zeros([3 12]);
for i=1:6
    N_hex_rot(:,(i-1)*2+1) = normalize( [0;0;1], 'norm');
    N_hex_rot(:,(i-1)*2+2) = normalize( cross([0;0;1], X_hex(:,i)), 'norm');
end

% square variable thrust axes at 0° rotation
N_sqr_rot = zeros([3 8]);
for i=1:4
    N_sqr_rot(:,(i-1)*2+1) = normalize( [0;0;1], 'norm');
    N_sqr_rot(:,(i-1)*2+2) = normalize( cross([0;0;1], X_sqr(:,i)), 'norm');
end

%%

% octahedron (fix) thrust axes (lynchpin)
N_oct_fix = [
    normalize([0,          -1,   0        ],'norm')
    normalize([sqrt(1/12), 1/2,  sqrt(2/3)],'norm')
    normalize([sqrt(3/4),  1/2,  0        ],'norm')
    normalize([-sqrt(1/3), 0,    sqrt(2/3)],'norm')
    normalize([-sqrt(3/4), 1/2,  0        ],'norm')
    normalize([sqrt(1/12), -1/2, sqrt(2/3)],'norm')
]';


% cube (fix) thrust axes (Dario Brescianini)
a = 1/2+1/sqrt(12); b = 1/2-1/sqrt(12); c = 1/sqrt(3);
N_cub_fix = [
    normalize([-a, +b, +c],'norm')
    normalize([+b, +a, -c],'norm')
    normalize([-b, -a, -c],'norm')
    normalize([+a, -b, +c],'norm')
    normalize([+a, -b, +c],'norm')
    normalize([-b, -a, -c],'norm')
    normalize([+b, +a, -c],'norm')
    normalize([-a, +b, +c],'norm')
]';

% odar (fix) thrust axes
N_odar_fix = [
	normalize([+0.68 +0.25 +0.68],'norm')
    normalize([+0.68 +0.25 -0.68],'norm')
    normalize([+0.68 -0.25 +0.68],'norm')
    normalize([+0.68 -0.25 -0.68],'norm')
    normalize([-0.68 +0.25 +0.68],'norm')
    normalize([-0.68 +0.25 -0.68],'norm')
    normalize([-0.68 -0.25 +0.68],'norm')
    normalize([-0.68 -0.25 -0.68],'norm')
]';

% O7P (fix) thrust axes
N_o7p_fix = [
    normalize([+0.36 -0.90 +0.25],'norm')
    normalize([-0.35 +0.44 +0.83],'norm')
    normalize([+0.29 +0.76 -0.58],'norm')
    normalize([-0.81 -0.12 -0.58],'norm')
    normalize([-0.37 +0.45 +0.81],'norm')
    normalize([+0.78 -0.57 +0.26],'norm')
    normalize([+0.10 -0.07 -0.99],'norm')
]';

% hexagon (fix) thrust axes (30° tilt)
N_hex_fix = zeros([3 6]);
for i = 1:6
	N_hex_fix(:,i) = rotationVectorToMatrix( X_hex(:,i)*deg2rad(30)*(mod(i,2)*2-1) )*[0;0;1];
end


%%
[roll,pitch] = ndgrid(-180:2:180);

fprintf('calc E1_tet_rot..\n'); E1_tet_rot = arrayfun(@(r,p) efficiency(r,p, N_tet_rot,X_tet, 'rot','E1'), roll,pitch);
fprintf('calc E1_oct_rot..\n'); E1_oct_rot = arrayfun(@(r,p) efficiency(r,p, N_oct_rot,X_oct, 'rot','E1'), roll,pitch);
fprintf('calc E1_cub_rot..\n'); E1_cub_rot = arrayfun(@(r,p) efficiency(r,p, N_cub_rot,X_cub, 'rot','E1'), roll,pitch);
fprintf('calc E1_hex_rot..\n'); E1_hex_rot = arrayfun(@(r,p) efficiency(r,p, N_hex_rot,X_hex, 'rot','E1'), roll,pitch);
fprintf('calc E1_sqr_rot..\n'); E1_sqr_rot = arrayfun(@(r,p) efficiency(r,p, N_sqr_rot,X_sqr, 'rot','E1'), roll,pitch);

fprintf('calc E2_tet_rot..\n'); E2_tet_rot = arrayfun(@(r,p) efficiency(r,p, N_tet_rot,X_tet, 'rot','E2'), roll,pitch);
fprintf('calc E2_oct_rot..\n'); E2_oct_rot = arrayfun(@(r,p) efficiency(r,p, N_oct_rot,X_oct, 'rot','E2'), roll,pitch);
fprintf('calc E2_cub_rot..\n'); E2_cub_rot = arrayfun(@(r,p) efficiency(r,p, N_cub_rot,X_cub, 'rot','E2'), roll,pitch);
fprintf('calc E2_hex_rot..\n'); E2_hex_rot = arrayfun(@(r,p) efficiency(r,p, N_hex_rot,X_hex, 'rot','E2'), roll,pitch);
fprintf('calc E2_sqr_rot..\n'); E2_sqr_rot = arrayfun(@(r,p) efficiency(r,p, N_sqr_rot,X_sqr, 'rot','E2'), roll,pitch);

fprintf('calc E1_oct_fix..\n'); E1_oct_fix = arrayfun(@(r,p) efficiency(r,p, N_oct_fix,X_oct, 'fix','E1'), roll,pitch);
fprintf('calc E1_cub_fix..\n'); E1_cub_fix = arrayfun(@(r,p) efficiency(r,p, N_cub_fix,X_cub, 'fix','E1'), roll,pitch);
fprintf('calc E1_odar_fix..\n');E1_odar_fix= arrayfun(@(r,p) efficiency(r,p, N_odar_fix,X_odar,'fix','E1'),roll,pitch);
fprintf('calc E1_o7p_fix..\n'); E1_o7p_fix = arrayfun(@(r,p) efficiency(r,p, N_o7p_fix,X_hep, 'fix','E1'), roll,pitch);
fprintf('calc E1_hex_fix..\n'); E1_hex_fix = arrayfun(@(r,p) efficiency(r,p, N_hex_fix,X_hex, 'fix','E1'), roll,pitch);

fprintf('calc E2_oct_fix..\n'); E2_oct_fix = arrayfun(@(r,p) efficiency(r,p, N_oct_fix,X_oct, 'fix','E2'), roll,pitch);
fprintf('calc E2_cub_fix..\n'); E2_cub_fix = arrayfun(@(r,p) efficiency(r,p, N_cub_fix,X_cub, 'fix','E2'), roll,pitch);
fprintf('calc E2_odar_fix..\n');E2_odar_fix= arrayfun(@(r,p) efficiency(r,p, N_odar_fix,X_odar,'fix','E2'),roll,pitch);
fprintf('calc E2_o7p_fix ..\n');E2_o7p_fix = arrayfun(@(r,p) efficiency(r,p, N_o7p_fix,X_hep, 'fix','E2'), roll,pitch);
fprintf('calc E2_hex_fix..\n'); E2_hex_fix = arrayfun(@(r,p) efficiency(r,p, N_hex_fix,X_hex, 'fix','E2'), roll,pitch);

disp('generate plots..');
fig1 = figure(1); clf(fig1);
set(fig1,'Position',[500,100,1200,900]);
fig2 = figure(2); clf(fig2);
set(fig2,'Position',[500,100,1200,900]);

figure(fig1);

plot_icon(subplot(3,5,1),1);
plot_icon(subplot(3,5,2),2);
plot_icon(subplot(3,5,3),3);
plot_icon(subplot(3,5,4),4);
plot_icon(subplot(3,5,5),5);

plot_res(subplot(3,5,6),6,   roll,pitch,E1_tet_rot);
plot_res(subplot(3,5,7),7,   roll,pitch,E1_oct_rot);
plot_res(subplot(3,5,8),8,   roll,pitch,E1_cub_rot);
plot_res(subplot(3,5,9),9,   roll,pitch,E1_hex_rot);
plot_res(subplot(3,5,10),10, roll,pitch,E1_sqr_rot);

plot_res(subplot(3,5,11),11, roll,pitch,E2_tet_rot);
plot_res(subplot(3,5,12),12, roll,pitch,E2_oct_rot);
plot_res(subplot(3,5,13),13, roll,pitch,E2_cub_rot);
plot_res(subplot(3,5,14),14, roll,pitch,E2_hex_rot);
plot_res(subplot(3,5,15),15, roll,pitch,E2_sqr_rot);

set(findall(fig1,'-property','FontSize'),'FontSize',12);
cbh = findall(fig1, 'Type', 'Colorbar'); cbh(1).Position(1) = cbh(2).Position(1);
tightfig(fig1);

figure(fig2);

plot_icon(subplot(3,5,1),6);
plot_icon(subplot(3,5,2),7);
plot_icon(subplot(3,5,3),8);
plot_icon(subplot(3,5,4),9);
plot_icon(subplot(3,5,5),10);

plot_res(subplot(3,5,6),6,   roll,pitch,E1_oct_fix);
plot_res(subplot(3,5,7),7,   roll,pitch,E1_cub_fix);
plot_res(subplot(3,5,8),8,   roll,pitch,E1_odar_fix);
plot_res(subplot(3,5,9),9,   roll,pitch,E1_o7p_fix);
plot_res(subplot(3,5,10),10, roll,pitch,E1_hex_fix);

plot_res(subplot(3,5,11),11, roll,pitch,E2_oct_fix);
plot_res(subplot(3,5,12),12, roll,pitch,E2_cub_fix);
plot_res(subplot(3,5,13),13, roll,pitch,E2_odar_fix);
plot_res(subplot(3,5,14),14, roll,pitch,E2_o7p_fix);
plot_res(subplot(3,5,15),15, roll,pitch,E2_hex_fix);

set(findall(fig2,'-property','FontSize'),'FontSize',12);
cbh = findall(fig2, 'Type', 'Colorbar'); cbh(1).Position(1) = cbh(2).Position(1);
tightfig(fig2);

disp('export graphics..');
exportgraphics(fig1,'1_geometry_analysis1.eps','ContentType','vector');
exportgraphics(fig2,'1_geometry_analysis2.eps','ContentType','vector');

disp('DONE!');

%%

function res = efficiency(roll, pitch, N,X, arm,type)
    
    R = eul2rotm([0 pitch*pi/180 roll*pi/180]);
    N = R*N;
    X = R*X;

    if strcmp(arm,'rot')
        X = repelem(X,1,2);
    end

    K = [N; cross(X,N)];
    F = pinv(K) * [0;0;1;0;0;0];
    T = N.*F';

    if strcmp(arm,'rot')
        T_new = zeros([3 size(T,2)/2]);
        for i = 1:size(T,2)/2
            T_new(:,i) = T(:,(i-1)*2+1) + T(:,(i-1)*2+2);
        end
        T = T_new;
        F = vecnorm(T)';
        N = T./F';
    end
    
    if strcmp(type,'E1')
        res = 1/sum(abs(F));
    end
    if strcmp(type,'E2')
        res = 1/(max(abs(F))*length(F));
    end

end

%%

function plot_icon(h,i)

    if i== 1
        imshow(imread('icons/tet_rot.png', 'BackgroundColor', [1 1 1]));
        title({'(A) \ \textbf{Tetrahedron with}','\textbf{rotating arms}'});
    elseif i==2
        imshow(imread('icons/oct_rot.png', 'BackgroundColor', [1 1 1]));
        title({'(B) \ \textbf{Octahedron with}','\textbf{rotating arms}'});
    elseif i==3
        imshow(imread('icons/cub_rot.png', 'BackgroundColor', [1 1 1]));
        title({'(C) \ \textbf{Cube with}','\textbf{rotating arms}'});
    elseif i==4
        imshow(imread('icons/hex_rot.png', 'BackgroundColor', [1 1 1]));
        title({'(D) \textbf{Hexagon with}','\textbf{rotating arms} [15, 16]'});
    elseif i==5
        imshow(imread('icons/sqr_rot.png', 'BackgroundColor', [1 1 1]));
        title({'(E) \ \textbf{Square with}','\textbf{rotating arms} [26]'});

    elseif i==6
        imshow(imread('icons/oct_fix.png', 'BackgroundColor', [1 1 1]));
        title({'(A) \ \textbf{Octahedron with}','\textbf{fixed-tilt arms} [3]'});
    elseif i==7
        imshow(imread('icons/cub_fix.png', 'BackgroundColor', [1 1 1]));
        title({'(B) \ \textbf{Cube with}','\textbf{fixed-tilt arms} [4]'});
    elseif i==8
        imshow(imread('icons/odar_fix.png', 'BackgroundColor', [1 1 1]));
        title({'(C) \ \textbf{ODAR with}','\textbf{fixed-tilt arms} [5]'});
    elseif i==9
        imshow(imread('icons/o7p_fix.png', 'BackgroundColor', [1 1 1]));
        title({'(D) \ \textbf{O7+ with}','\textbf{fixed-tilt arms} [6]'});
    elseif i==10
        imshow(imread('icons/hex_fix.png', 'BackgroundColor', [1 1 1]));
        title({'(E) \ \textbf{Hexagon with 30$\mathbf{^{\circ}}$}','\textbf{fixed-tilt arms} [13]'});
    end

    h.Position = h.Position .* [1, 1, 1.2, 1.2] + [0, -0.07, 0, 0];

end

function plot_res(h,i, roll,pitch,effic)

	surf(roll,pitch,effic);
	xlim([-180;180]); ylim([-180;180]);
    xticks([-180 -90 0 90 180]); yticks([-180 -90 0 90 180]);
    xticklabels([]); yticklabels([]);
	shading interp; view(0,90); pbaspect([1 1 1]);
    set(gca, 'layer', 'top');
    h.GridAlpha = 0.6;
    
    pos = [0 0]; sze = [180 90];
    effic_min = min(effic,[],'all');
    effic_max = max(effic,[],'all'); if effic_max > 0.985, effic_max = 1.00; end
    str = { sprintf('min: $\\,%0.2f$',effic_min) ; sprintf('max: $%0.2f$',effic_max)};
    rct = rectangle('Position',[-180+pos(1) -180+pos(2) sze(1) sze(2)], 'FaceColor','w');
    txt = text(-180+sze(1)/2+pos(1),-180+sze(2)/2+pos(2)-5, str,'HorizontalAlignment','center','VerticalAlignment','middle');
    ax2 = copyobj(gca,gcf); cla(ax2); set(ax2, 'Visible','off');
    linkprop([gca ax2],{'CameraPosition' 'XLim' 'YLim' 'ZLim' 'Position'});
    set(rct,'Parent',ax2); set(txt,'Parent',ax2);

    if i==8
        title('\textbf{Fraction of hover thrust pointing upwards - $\mathbf{X_1}$}');
    end
    if i==13
        title('\textbf{Fraction of total thrust available for hovering - $\mathbf{X_2}$}');
    end

    if i==6 || i==11
        yticklabels({'$-\pi$','$-\frac{\pi}{2}$','0','$\frac{\pi}{2}$','$\pi$'});
        ylabel('Pitch [rad]');
    end
    if i>5 && i<=15
        xticklabels({'$\quad-\pi$','$-\frac{\pi}{2}$','0','$\frac{\pi}{2}$','$\pi\ $'});
        xtickangle(0);
        xlabel('Roll [rad]');
    end

    if i>5 && i<=10
        clim([0.6;1.0]);
        set(h, 'Colormap', flip(turbo), 'CLim', [0.6 1.0])
        if i==10
            cbh = colorbar();
            cbh.Ticks = linspace(0.6, 1.0, 9);
            text(1.16,-0.14,'$X_1$', 'Units','normalized');
        end
    end
    if i>10 && i<=15
        clim([0.3;1.0]);
        set(h, 'Colormap', flip(turbo), 'CLim', [0.3 1.0])
        if i==15
            cbh = colorbar();
            cbh.Ticks = linspace(0.3, 1.0, 8);
            text(1.16,-0.14,'$X_2$', 'Units','normalized');
        end
    end
    
    h.Position = h.Position .* [1, 1, 1.2, 1.2];
    if i>10 && i<=15
        h.Position = h.Position + [0, 0.01, 0, 0];
    end

end


