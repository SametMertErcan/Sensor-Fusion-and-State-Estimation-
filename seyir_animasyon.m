%%  ------ #4 -----
% SAMET MERT ERCAN, 2025
close all;
fps    = 30;
Tdur   = t(end) - t(1);
Nf     = max(2, round(Tdur*fps)+1);
t_anim = linspace(t(1), t(end), Nf);

E_anim = interp1(t, pos_ned(:,2), t_anim, 'pchip');
N_anim = interp1(t, pos_ned(:,1), t_anim, 'pchip');
D_anim = interp1(t, pos_ned(:,3), t_anim, 'pchip');

wrapA     = @(x) mod(x+pi,2*pi)-pi;
yaw_anim   = wrapA( interp1(t, unwrap(yaw_ned)  , t_anim, 'pchip') );
pitch_anim = wrapA( interp1(t, unwrap(pitch_ned), t_anim, 'pchip') );
roll_anim  = wrapA( interp1(t, unwrap(roll_ned) , t_anim, 'pchip') );

deg = 180/pi;
phi_deg   = roll_anim  * deg;
theta_deg = pitch_anim * deg;
psi_deg   = yaw_anim   * deg;

%% Sahne Ölçekleri
span = @(v) max(v)-min(v);
sceneSpan = max([ span(pos_ned(:,2)), span(pos_ned(:,1)), span(-pos_ned(:,3)), 1 ]);
Lmodel = max(15, 0.045*sceneSpan);
Laxes  = 0.60*Lmodel;
Rwin   = 6.0*Lmodel;
camDist   = 10.0*Lmodel;
camHeight = 3.0*Lmodel;

%% Figür 1: Takip Kamera
figAnim = figure('Color','w','Name','Uçuş Animasyonu', ...
    'Units','pixels','Position',[100 100 1280 720], 'Renderer','opengl');
ax3d = axes('Parent',figAnim); hold(ax3d,'on'); axis(ax3d,'vis3d'); axis(ax3d,'off');
set(ax3d,'Projection','perspective');
lighting(ax3d,'gouraud'); camlight(ax3d,'headlight'); material(ax3d,'dull');

axX = plot3(ax3d, NaN,NaN,NaN,'-','LineWidth',2.0,'Color','c');
axY = plot3(ax3d, NaN,NaN,NaN,'-','LineWidth',2.0,'Color',[0.15 0.6 0.15]);
axZ = plot3(ax3d, NaN,NaN,NaN,'-','LineWidth',2.0,'Color',[0.85 0.1 0.1]);

tx = text(ax3d, NaN,NaN,NaN, 'X', 'Color',[0 0 0],   'FontWeight','bold', ...
          'HorizontalAlignment','center','VerticalAlignment','middle', ...
          'BackgroundColor',[1 1 1 .6],'Margin',1);
ty = text(ax3d, NaN,NaN,NaN, 'Y', 'Color',[0.15 0.6 0.15], 'FontWeight','bold', ...
          'HorizontalAlignment','center','VerticalAlignment','middle', ...
          'BackgroundColor',[1 1 1 .6],'Margin',1);
tz = text(ax3d, NaN,NaN,NaN, 'Z', 'Color',[0.85 0.1 0.1], 'FontWeight','bold', ...
          'HorizontalAlignment','center','VerticalAlignment','middle', ...
          'BackgroundColor',[1 1 1 .6],'Margin',1);

[Vb,F] = aircraft_tri_model(Lmodel);
air = patch('Faces',F,'Vertices',nan(size(Vb)), ...
            'FaceColor',[0.00 0.45 0.85], ...
            'EdgeColor','k','LineWidth',0.8, ...
            'FaceAlpha',1.0, ...
            'Parent',ax3d);

%% Figür 2: Açı Grafikleri
figAng = figure('Color','w','Name','Duruş Açıları','Units','centimeters','Position',[2 2 14 12]);
tloAng = tiledlayout(figAng,3,1,'TileSpacing','compact','Padding','compact');
ax_phi = nexttile(tloAng); plot(ax_phi,t_anim,phi_deg,'b','LineWidth',1); ylabel(ax_phi,'\phi [deg]'); box(ax_phi,'on');
title('Duruş Açıları'); grid minor;
ax_the = nexttile(tloAng); plot(ax_the,t_anim,theta_deg,'b','LineWidth',1); ylabel(ax_the,'\theta [deg]'); box(ax_the,'on'); grid minor;
ax_psi = nexttile(tloAng); plot(ax_psi,t_anim,psi_deg,'b','LineWidth',1); ylabel(ax_psi,'\psi [deg]'); xlabel(ax_psi,'t [s]'); box(ax_psi,'on'); grid minor;
xlim(ax_phi,[t_anim(1) t_anim(end)]); xlim(ax_the,[t_anim(1) t_anim(end)]); xlim(ax_psi,[t_anim(1) t_anim(end)]);
vline_phi = xline(ax_phi, t_anim(1), 'r-','LineWidth',1);
vline_the = xline(ax_the, t_anim(1), 'r-','LineWidth',1);
vline_psi = xline(ax_psi, t_anim(1), 'r-','LineWidth',1);

%% Figür 3: Dış Kamera
figExt = figure('Color','w','Name','Dış Kamera — Rota Takibi', ...
    'Units','pixels','Position',[80 80 900 700],'Renderer','opengl');
axExt = axes('Parent',figExt); hold(axExt,'on'); axis(axExt,'equal'); axis(axExt,'vis3d');
grid(axExt,'on'); box(axExt,'off'); grid minor;
plot3(axExt, pos_ned(:,2), pos_ned(:,1), -pos_ned(:,3), 'Color',[0.75 0.75 0.75],'LineStyle','-','LineWidth',0.5);
plot3(axExt, pos_ned(1,2), pos_ned(1,1), -pos_ned(1,3), 'o','MarkerFaceColor',[1 .8 .1],'MarkerEdgeColor','k','MarkerSize',6);
plot3(axExt, pos_ned(end,2), pos_ned(end,1), -pos_ned(end,3), 's','MarkerFaceColor',[.85 .1 .1],'MarkerEdgeColor','k','MarkerSize',6);
xlabel(axExt,'E [m]'); ylabel(axExt,'N [m]'); zlabel(axExt,'Up [m]');
padRoute = 0.04*sceneSpan + 20;
xlim(axExt,[min(pos_ned(:,2))-padRoute, max(pos_ned(:,2))+padRoute]);
ylim(axExt,[min(pos_ned(:,1))-padRoute, max(pos_ned(:,1))+padRoute]);
zlim(axExt,[min(-pos_ned(:,3))-padRoute, max(-pos_ned(:,3))+padRoute]);
view(axExt, 42, 22);
air2 = patch('Faces',F,'Vertices',nan(size(Vb)), ...
             'FaceColor',[0.00 0.45 0.85], ...
             'EdgeColor','k','LineWidth',0.8, ...
             'FaceAlpha',1.0, ...
             'Parent',axExt);
lighting(axExt,'gouraud'); camlight(axExt,'headlight'); material(axExt,'dull');

%% Figür 4: Üstten Rota + Seri
figTop = figure('Color','w','Name','Üstten Rota + İrtifa/Hız (dar alt panel)', ...
                'Units','pixels','Position',[120 120 980 760],'Renderer','opengl');
tloTop = tiledlayout(figTop,5,1,'TileSpacing','compact','Padding','compact');

axTop = nexttile(tloTop,[4 1]); hold(axTop,'on'); axis(axTop,'equal');
box(axTop,'on'); grid(axTop,'on'); axTop.GridAlpha = 0.15; grid minor;
plot(axTop, pos_ned(:,2), pos_ned(:,1), 'Color',[0.75 0.75 0.75], 'LineWidth',1.2);
trail2D = animatedline(axTop,'LineWidth',2.0,'Color',[0.00 0.65 0.00]);
plot(axTop, pos_ned(1,2),pos_ned(1,1),'o','MarkerSize',6,'MarkerFaceColor',[1 .8 .1],'MarkerEdgeColor','k');
plot(axTop, pos_ned(end,2),pos_ned(end,1),'s','MarkerSize',6,'MarkerFaceColor',[.85 .1 .1],'MarkerEdgeColor','k');
xlabel(axTop,'E [m]'); ylabel(axTop,'N [m]');
pad2 = 0.04*sceneSpan + 20;
xlim(axTop,[min(pos_ned(:,2))-pad2, max(pos_ned(:,2))+pad2]);
ylim(axTop,[min(pos_ned(:,1))-pad2, max(pos_ned(:,1))+pad2]);

Licon = max(10, 0.025*sceneSpan);  Wicon = 0.50*Licon;
Vtri  = [ +0.60*Licon, 0;
          -0.40*Licon, +0.35*Wicon;
          -0.40*Licon, -0.35*Wicon ];
plane2D = patch('XData',nan(3,1),'YData',nan(3,1), ...
                'FaceColor',[0.00 0.45 0.85], ...
                'EdgeColor','k','LineWidth',1.0, ...
                'Parent',axTop);

axTS = nexttile(tloTop,[1 1]); hold(axTS,'on');
box(axTS,'off'); grid(axTS,'on');
set(axTS,'XMinorGrid','on','YMinorGrid','on');

alt_anim = -D_anim;

dE  = [0 diff(E_anim)];  dN  = [0 diff(N_anim)];
dtv = [1 diff(t_anim)];  spd_anim = hypot(dE,dN) ./ max(dtv,eps);

yyaxis(axTS,'left');  axTS.YColor = [0 0 1];
yyaxis(axTS,'right'); axTS.YColor = [0 0 0];

yyaxis(axTS,'left');
pltAlt = plot(axTS, t_anim, alt_anim, 'b', 'LineWidth',1.5);
ylabel(axTS,'İrtifa [m]');

yyaxis(axTS,'right');
pltSpd = plot(axTS, t_anim, spd_anim, 'k', 'LineWidth',1.5);
ylabel(axTS,'Hız [m/s]');
xlabel(axTS,'t [s]');
xlim(axTS,[t_anim(1) t_anim(end)]);

lg = legend(axTS,[pltAlt,pltSpd],{'İrtifa','Hız'},'Location','best');
set(lg,'Box','off');

vline_ts = xline(axTS, t_anim(1), 'k-','LineWidth',1);

%% Video Bayrakları
fps = 30;
makeVidFollow = false;
makeVidAngles = false;
makeVidRoute  = false;
makeVidTop    = false;

vwFollow = []; vwAngles = []; vwRoute = []; vwTop = [];
if makeVidFollow, vwFollow = openVideoWriter('uav_follow', fps); end
if makeVidAngles, vwAngles = openVideoWriter('uav_angles', fps); end
if makeVidRoute , vwRoute  = openVideoWriter('uav_route' , fps); end
if makeVidTop   , vwTop    = openVideoWriter('uav_topdown', fps); end

% % ---------- 6) Videolar  ----------------------------------------
% makeVideoMain = false;   % Takip kamerası
% makeVideoExt  = false;   % Dış kamera
% vwMain = []; vwExt = [];
% if makeVideoMain, vwMain = openVideoWriter('uav_follow'); end
% if makeVideoExt , vwExt  = openVideoWriter('uav_route' ); end

%% Ana Döngü
for k = 1:Nf
    P = [E_anim(k); N_anim(k); -D_anim(k)];
    Cnb = Rz(yaw_anim(k)) * Ry(pitch_anim(k)) * Rx(roll_anim(k));

    ex_ned = Cnb*[1;0;0]; ey_ned = Cnb*[0;1;0]; ez_ned = Cnb*[0;0;1];
    ex_d = [ex_ned(2); ex_ned(1); -ex_ned(3)];
    ey_d = [ey_ned(2); ey_ned(1); -ey_ned(3)];
    ez_d = [ez_ned(2); ez_ned(1); -ez_ned(3)];

    set(axX,'XData',[P(1) P(1)+Laxes*ex_d(1)],'YData',[P(2) P(2)+Laxes*ex_d(2)],'ZData',[P(3) P(3)+Laxes*ex_d(3)]);
    set(axY,'XData',[P(1) P(1)+Laxes*ey_d(1)],'YData',[P(2) P(2)+Laxes*ey_d(2)],'ZData',[P(3) P(3)+Laxes*ey_d(3)]);
    set(axZ,'XData',[P(1) P(1)+Laxes*ez_d(1)],'YData',[P(2) P(2)+Laxes*ez_d(2)],'ZData',[P(3) P(3)+Laxes*ez_d(3)]);
    set(tx,'Position', [P(1)+1.05*Laxes*ex_d(1), P(2)+1.05*Laxes*ex_d(2), P(3)+1.05*Laxes*ex_d(3)]);
    set(ty,'Position', [P(1)+1.05*Laxes*ey_d(1), P(2)+1.05*Laxes*ey_d(2), P(3)+1.05*Laxes*ey_d(3)]);
    set(tz,'Position', [P(1)+1.05*Laxes*ez_d(1), P(2)+1.05*Laxes*ez_d(2), P(3)+1.05*Laxes*ez_d(3)]);

    Vnav  = (Cnb * Vb.').';
    Vdisp = [Vnav(:,2), Vnav(:,1), -Vnav(:,3)] + P.';
    set(air,'Vertices',Vdisp);

    xlim(ax3d,[P(1)-Rwin, P(1)+Rwin]);
    ylim(ax3d,[P(2)-Rwin, P(2)+Rwin]);
    zlim(ax3d,[P(3)-Rwin, P(3)+Rwin]);
    camPos = (P - camDist*ex_d + camHeight*[0;0;1]).';
    set(ax3d,'CameraPosition',camPos, 'CameraTarget',P.', 'CameraUpVector',[0 0 1]);

    set(air2,'Vertices',Vdisp);

    vline_phi.Value = t_anim(k);
    vline_the.Value = t_anim(k);
    vline_psi.Value = t_anim(k);

    addpoints(trail2D, E_anim(k), N_anim(k));
    if k>1, chi = atan2(E_anim(k)-E_anim(k-1), N_anim(k)-N_anim(k-1)); else, chi = yaw_anim(k); end
    alpha = pi/2 - chi;
    R2 = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)];
    Vrot = (R2 * Vtri.').';
    set(plane2D, 'XData', Vrot(:,1) + E_anim(k), 'YData', Vrot(:,2) + N_anim(k));
    vline_ts.Value = t_anim(k);

    Vrot = (R2 * Vtri.').';
    set(plane2D, 'XData', Vrot(:,1) + E_anim(k), ...
        'YData', Vrot(:,2) + N_anim(k));
    drawnow limitrate nocallbacks

    if ~isempty(vwFollow), writeVideo(vwFollow, getframe(figAnim)); end
    if ~isempty(vwAngles), writeVideo(vwAngles, getframe(figAng )); end
    if ~isempty(vwRoute ), writeVideo(vwRoute , getframe(figExt )); end
    if ~isempty(vwTop   ), writeVideo(vwTop   , getframe(figTop )); end
end

%% Kapanış
if ~isempty(vwFollow), close(vwFollow); disp('Video (takip) kaydedildi.'); end
if ~isempty(vwAngles), close(vwAngles); disp('Video (açılar) kaydedildi.'); end
if ~isempty(vwRoute ), close(vwRoute ); disp('Video (dış kamera) kaydedildi.'); end
if ~isempty(vwTop   ), close(vwTop   ); disp('Video (üstten rota) kaydedildi.'); end

%% Yardımcı Fonksiyonlar
function vw = openVideoWriter(basename, fps)
vw = [];
try
    profs = VideoWriter.getProfiles; names = string({profs.Name});
    if any(names=="MPEG-4")
        vw = VideoWriter([basename '.mp4'],'MPEG-4');
    elseif any(names=="Motion JPEG AVI")
        vw = VideoWriter([basename '.avi'],'Motion JPEG AVI'); vw.Quality = 95;
    else
        vw = VideoWriter([basename '.avi']); vw.Quality = 95;
    end
    vw.FrameRate = fps;
    open(vw);
    disp(['[Video] Profil: ', class(vw), '  Dosya: ', vw.Filename, ...
          '  FPS: ', num2str(fps)]);
catch ME
    warning('VideoWriter acilamadi: %s\nVideo kapali: %s', ME.message, basename);
    vw = [];
end
end

function R = Rx(a), c=cos(a); s=sin(a); R=[1 0 0; 0 c -s; 0 s c]; end
function R = Ry(a), c=cos(a); s=sin(a); R=[c 0 s; 0 1 0; -s 0 c]; end
function R = Rz(a), c=cos(a); s=sin(a); R=[c -s 0; s c 0; 0 0 1]; end

function [V,F] = aircraft_tri_model(L)
Lf=1.00*L; Ww=0.80*L; Wh=0.36*L;
nose=[+0.55*Lf, 0, 0]; rb=[-0.35*Lf, +0.06*L, 0]; lb=[-0.35*Lf, -0.06*L, 0];
wR1=[0.00*Lf, +0.50*Ww, +0.02*L]; wR2=[-0.10*Lf, +0.12*Ww, 0]; wR3=[-0.22*Lf, +0.05*Ww, 0];
wL1=[0.00*Lf, -0.50*Ww, +0.02*L]; wL2=[-0.10*Lf, -0.12*Ww, 0]; wL3=[-0.22*Lf, -0.05*Ww, 0];
vt1=[-0.42*Lf, 0, -0.18*L]; vt2=[-0.32*Lf, 0, 0]; vt3=[-0.52*Lf, 0, 0];
htR=[-0.46*Lf, +0.18*Wh, 0]; htL=[-0.46*Lf, -0.18*Wh, 0]; htC=[-0.40*Lf, 0, 0];
V=[nose; rb; lb; wR1; wR2; wR3; wL1; wL2; wL3; vt1; vt2; vt3; htR; htL; htC];
F=[ 1 2 3;  1 5 4; 1 6 5; 1 8 7; 1 9 8; 11 10 12; 12 13 15; 12 15 14 ];
end

function styleThisFigure(fig)
set(fig,'Color',[1 1 1]);
ax = findall(fig,'Type','axes');
if ~isempty(ax)
    set(ax, ...
        'Box','off', ...
        'Color',[1  1 1], ...
        'XGrid','on','YGrid','on','ZGrid','on', ...
        'XMinorTick','off','YMinorTick','off','ZMinorTick','off', ...
        'XMinorGrid','on','YMinorGrid','on','ZMinorGrid','on', ...
        'GridColor',[1 1 1],'GridAlpha',1, ...
        'TickDir','out','LineWidth',0.75, ...
        'FontName','Arial','FontSize',11, ...
        'TitleFontWeight','bold','TitleFontSizeMultiplier',0.9);
end
axh = findall(fig,'Type','axes');
cizgiKal =1;
for ax = axh.'
    ln = findall(ax,'Type','line'); ln = flipud(ln);
    if numel(ln)>=1, set(ln(1),'Color','r','LineWidth',cizgiKal); end
    if numel(ln)>=2, set(ln(2),'Color','k','LineWidth',cizgiKal,'LineStyle','-'); end
    if numel(ln)>=3, set(ln(3),'Color','b','LineWidth',cizgiKal); end
    if numel(ln)>=4, set(ln(4),'Color','b','LineWidth',cizgiKal); end
end
lgd = findall(fig,'Type','legend');
if ~isempty(lgd)
    set(lgd,'Box','off','Fontsize',9);
end
end
