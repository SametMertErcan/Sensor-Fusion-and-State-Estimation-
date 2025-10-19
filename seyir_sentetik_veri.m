%%  ------ #1 -----
% SAMET MERT ERCAN, 2025
clear; clc; close all;
set(groot, 'defaultAxesColor', 'w', ...
    'defaultFigureColor', 'w', ...
    'defaultAxesBox', 'on');
% set(groot, ...
%   'defaultFigureColor',[1 1 1], ...
%   'defaultAxesColor',[0.92 0.92 0.92], ...
%   'defaultAxesBox','off', ...
%   'defaultAxesXGrid','on','defaultAxesYGrid','on','defaultAxesZGrid','on', ...
%   'defaultAxesXMinorTick','off','defaultAxesYMinorTick','off','defaultAxesZMinorTick','off', ...
%   'defaultAxesXMinorGrid','off','defaultAxesYMinorGrid','off','defaultAxesZMinorGrid','off', ...
%   'defaultAxesGridColor',[1 1 1], 'defaultAxesGridAlpha',1, ...
%   'defaultAxesLineWidth',0.75, 'defaultAxesTickDir','out', ...a
%   'defaultAxesFontName','Arial','defaultAxesFontSize',11, ...
%   'defaultAxesTitleFontWeight','bold', ...
%   'defaultLineLineWidth',1.1,'defaultLineColor','k', ...
%   'defaultLegendBox','off');

%% Parametreler
fs = 1;
dt = 1/fs;
g  = 9.80665;

%% 1a: Kalkış Koşusu
T_acc  = 20;
v_max1 = 30;
tA = (0:dt:T_acc).';  uA = tA/T_acc;
sA = 10*uA.^3 - 15*uA.^4 + 6*uA.^5;
vA = v_max1 * sA;
xA = cumtrapz(tA, vA);
yA = zeros(size(xA));
zA = zeros(size(xA));

%% 1b: Düz Tırmanış
T_linTO   = 25;
gamma_deg = 8;
v_linTO   = vA(end);
h_linTO = v_linTO * T_linTO * tand(gamma_deg);
tB0 = (dt:dt:T_linTO).';
xB0 = xA(end) + v_linTO*tB0;
yB0 = yA(end) + 0*tB0;
u0  = tB0/T_linTO;
zB0 = zA(end) + h_linTO*(0.5 - 0.5*cos(pi*u0));

%% 1b2: Level Uçuş
T_flatTO = 20;
tB1  = (dt:dt:T_flatTO).';
xB1  = xB0(end) + v_linTO*tB1;
yB1  = yB0(end) + 0*tB1;
zB1  = zB0(end) + 0*tB1;

%% 1c: Helisel Geçiş
laps      = 4;
R_circ    = 200;
Vxy       = v_linTO;
omega_tgt = Vxy / R_circ;
psi0      = 0;
T_turnin = 6;
tIn   = (dt:dt:T_turnin).';  u = tIn/T_turnin;
env   = 10*u.^3 - 15*u.^4 + 6*u.^5;
omega = omega_tgt * env;
psi_in = psi0 + dt*cumsum(omega);
x_in   = xB1(end) + cumsum(Vxy*cos(psi_in))*dt;
y_in   = yB1(end) + cumsum(Vxy*sin(psi_in))*dt;
Theta_total = 2*pi*laps;
deficit_ang = trapz(tIn, omega);
T_const     = (Theta_total - deficit_ang)/omega_tgt;
tCo   = (dt:dt:T_const).';
z_target = 1000;
vz_eff_den = trapz(tIn, env) + T_const;
vz_target  = (z_target - zB1(end)) / max(vz_eff_den,eps);
vz_in = vz_target * env;
z_in  = zB1(end) + cumsum(vz_in)*dt;
psi_co = psi_in(end) + omega_tgt * tCo;
x_co   = x_in(end) + cumsum(Vxy*cos(psi_co))*dt;
y_co   = y_in(end) + cumsum(Vxy*sin(psi_co))*dt;
vz_co  = vz_target * ones(size(tCo));
z_co   = z_in(end) + cumsum(vz_co)*dt;
tB = [tIn;  T_turnin + tCo];
xB = [x_in; x_co];
yB = [y_in; y_co];
zB = [z_in; z_co];
t1 = [tA;
      T_acc               + tB0;
      T_acc+T_linTO       + tB1;
      T_acc+T_linTO+T_flatTO + tB];
x1 = [xA;  xB0;  xB1;  xB];
y1 = [yA;  yB0;  yB1;  yB];
z1 = [zA;  zB0;  zB1;  zB];

%% 2: Düz Seyir
T_cruise = 30;  tC = (dt:dt:T_cruise).';  v_cruise = 30;
xC = x1(end) + v_cruise*tC;  yC = y1(end) + 0*tC;  zC = 1000 + 0*tC;
t2 = t1(end) + tC;

%% 3: S-Manevrası
T_S   = 40; A_S = 100; T_ramp = 10; T_Seg = 30;
tD = (dt:dt:T_Seg).';
xD = xC(end) + v_cruise*tD;
if T_Seg >= 2*T_ramp
    env = ones(size(tD));
    maskIn  = tD < T_ramp;                 env(maskIn)  = 0.5 - 0.5*cos(pi*tD(maskIn)/T_ramp);
    maskOut = tD > (T_Seg - T_ramp); tau = T_Seg - tD(maskOut);
    env(maskOut) = 0.5 - 0.5*cos(pi*tau/T_ramp);
else
    env = 0.5*(1 - cos(2*pi*tD/T_Seg));
end
yD = yC(end) + A_S * env .* sin(2*pi*tD/T_S);
zD = 1000 + 0*tD;
t3 = t2(end) + tD;

%% 3b: Ekstra Düz
T_flat = 30; tD2 = (dt:dt:T_flat).';
xD2 = xD(end) + v_cruise*tD2; yD2 = yD(end) + 0*tD2; zD2 = 1000 + 0*tD2;
t3b = t3(end) + tD2;

%% 4: 180° Dönüş
R=200; v_turn=30; omega_turn=v_turn/R; T_turn=pi/omega_turn;
tE = (dt:dt:T_turn).';
xS = xD2(end); yS = yD2(end);
vx0 = (xD2(end)-xD2(end-1))/dt; vy0 = (yD2(end)-yD2(end-1))/dt;
psi0_enu = atan2(vy0, vx0);
xc4 = xS + R * sin(psi0_enu);   yc4 = yS - R * cos(psi0_enu);
theta0_4 = atan2(yS - yc4, xS - xc4);
thetaE = theta0_4 - omega_turn * tE;
xE = xc4 + R*cos(thetaE);  yE = yc4 + R*sin(thetaE);  zE = 1000 + 0*tE;
t4 = t3b(end) + tE;

%% 4b: Hizalama
T_align=5; tAlign=(dt:dt:T_align).';
vx4=(xE(end)-xE(end-1))/dt; vy4=(yE(end)-yE(end-1))/dt;
xE2 = xE(end) + vx4*tAlign;  yE2 = yE(end) + vy4*tAlign;  zE2 = zE(end) + 0*tAlign;
t4b = t4(end) + tAlign;

%% 5: Dalış
T_dive = 60; tF=(dt:dt:T_dive).'; uF=tF/T_dive;
s    = 10*uF.^3 - 15*uF.^4 + 6*uF.^5;
A_dive=150;
zF = 1000 - A_dive*(1 - cos(2*pi*s));
vx4 = (xE2(end)-xE2(end-1))/dt; vy4 = (yE2(end)-yE2(end-1))/dt;
psi5_const_enu = atan2(vy4, vx4);
xF = xE2(end) + v_cruise*cos(psi5_const_enu)*tF;
yF = yE2(end) + v_cruise*sin(psi5_const_enu)*tF;
t5   = t4b(end) + tF;

%% 6: Dalış Sonrası Düz
T_straight_after=10; tG=(dt:dt:T_straight_after).';
xG = xF(end) + v_cruise*cos(psi5_const_enu)*tG;
yG = yF(end) + v_cruise*sin(psi5_const_enu)*tG;
zG = 1000*ones(size(tG));
t6 = t5(end) + tG;

%% 7: Daire (İniş)
laps_desc=2; R_desc=R+200; v_touch=25; r_taper=v_touch/v_cruise;
T_desc_nom = (2*pi*R_desc*laps_desc)/v_cruise;
tH=(dt:dt:T_desc_nom).'; uH=tH/tH(end);
xS2=xG(end); yS2=yG(end); zS2=zG(end);
vx0=(xG(end)-xG(end-1))/dt; vy0=(yG(end)-yG(end-1))/dt; psi0=atan2(vy0,vx0);
xc = xS2 + R_desc*sin(psi0);  yc = yS2 - R_desc*cos(psi0);
theta0 = atan2(yS2 - yc, xS2 - xc);
env = 1 - (1 - r_taper) * 0.5*(1 - cos(pi*uH));
omega0 = (2*pi*laps_desc) / (sum(env)*dt);
theta = theta0 - omega0 * dt * cumsum(env);
xH = xc + R_desc*cos(theta); yH = yc + R_desc*sin(theta);
zH = zS2 * (1 + cos(pi*uH))/2;
v_arc = R_desc * omega0 * env;
t7 = t6(end) + tH;

%% 8: Pist ve Durma
T_roll=30; tI=(dt:dt:T_roll).'; uI=tI/T_roll;
sI = 10*uI.^3 - 15*uI.^4 + 6*uI.^5;
psi_end = theta(end) - pi/2;
v0_roll = max(v_arc(end), 1e-6); v_eps = 1e-3;
vI = max(v0_roll*(1 - sI), v_eps);
xI = xH(end) + cumsum(vI.*cos(psi_end)*dt);
yI = yH(end) + cumsum(vI.*sin(psi_end)*dt);
zI = linspace(zH(end), 0, numel(tI)).';
t8 = t7(end) + tI;

%% Birleştir
t = [t1;  t2;  t3;  t3b; t4; t4b;  t5;  t6;  t7;  t8];
x = [x1;  xC;  xD;  xD2; xE; xE2;  xF;  xG;  xH;  xI];
y = [y1;  yC;  yD;  yD2; yE; yE2; yF;  yG;  yH;  yI];
z = [z1;  zC;  zD;  zD2; zE; zE2; zF;  zG;  zH;  zI];

%% ENU->NED
w_mov = max(8, 2*ceil(0.3*fs)+1);
x = smoothdata(x,'movmean',w_mov);
y = smoothdata(y,'movmean',w_mov);
z = smoothdata(z,'movmean',w_mov);
pos_enu = [x y z];
T_enu2ned = [0 1 0; 1 0 0; 0 0 -1];
pos_ned = (T_enu2ned * pos_enu.').';
w_sg  = max(5, 2*ceil(0.5*fs)+1);
vel_ned_raw = [gradient(pos_ned(:,1),dt), gradient(pos_ned(:,2),dt), gradient(pos_ned(:,3),dt)];
vel_ned = smoothdata(vel_ned_raw,'sgolay',w_sg);
acc_ned_raw = [gradient(vel_ned(:,1),dt), gradient(vel_ned(:,2),dt), gradient(vel_ned(:,3),dt)];
acc_ned = smoothdata(acc_ned_raw,'sgolay',w_sg);

%% Yönelim
vN = vel_ned(:,1); 
vE = vel_ned(:,2); 
vD = vel_ned(:,3);
Vh = hypot(vN, vE);
V  = sqrt(Vh.^2 + vD.^2);
yaw_ned_raw = atan2(vE, vN);
yaw_u = unwrap(yaw_ned_raw);
%yaw_s = smoothdata(yaw_u,'sgolay',w_sg);
psi_dot = gradient(yaw_u, dt);
%psi_dot = smoothdata(psi_dot,'sgolay',w_sg);
pitch_ned = atan2(-vD, max(Vh,1e-6));
%pitch_ned = smoothdata(pitch_ned_raw,'sgolay',w_sg);
% tan(phi) = -(Vh * psi_dot) / (g * cos(theta))
cos_th = cos(pitch_ned);
cos_th = max(cos_th, 0.1);
tan_phi = (Vh .* psi_dot) ./ (g .* cos_th);
% roll_mask = (Vh < 1.0) | (abs(psi_dot) < 1e-3);
% tan_phi(roll_mask) = 0;
roll_ned = atan(tan_phi);
%roll_ned = max(min(roll_ned, deg2rad(60)), -deg2rad(60));
yaw_ned   = wrapToPi(yaw_u);
eul_ned = [yaw_ned, pitch_ned, roll_ned];
ori_ned = quaternion(eul_ned,'euler','ZYX','frame');

%% Görselleştirme
figure; plot(pos_ned(:,2), pos_ned(:,1),'k-','LineWidth',1.6); hold on; axis equal
plot(pos_ned(1,2), pos_ned(1,1),'sk','MarkerSize',8,'MarkerFaceColor','y');
plot(pos_ned(end,2),pos_ned(end,1),'sk','MarkerSize',8,'MarkerFaceColor','r');
xlabel('E [m]'); ylabel('N [m]'); title('Uçuş Rotası (NED: E vs N)');
legend('Yörünge','Başlangıç','Bitiş','Location','best');
styleThisFigure(gcf);
figure; plot3(pos_ned(:,2),pos_ned(:,1),-pos_ned(:,3),'k-','LineWidth',1.5); hold on
plot3(pos_ned(1,2), pos_ned(1,1), pos_ned(1,3),'sk','MarkerSize',8,'MarkerFaceColor','y');
plot3(pos_ned(end,2),pos_ned(end,1),pos_ned(end,3),'sk','MarkerSize',8,'MarkerFaceColor','r');
axis equal; xlabel('E [m]'); ylabel('N [m]'); zlabel('D [m]');
title('3B Rota (NED)'); legend('Yörünge','Başlangıç','Bitiş','Location','best');
styleThisFigure(gcf);
figure;
subplot(3,1,1);
plot(t, rad2deg(roll_ned),  'k','LineWidth',1); ylabel('Roll [derece]');xlabel('t [s]');
title('Duruş Açıları: Yatış, Dikilme ve Baş Açısı');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-25)  max(yl(2),25)]);
yticks([ -25   -15:5:15 25]);
subplot(3,1,2);
plot(t, rad2deg(pitch_ned), 'k','LineWidth',1);  ylabel('Pitch [derece]');xlabel('t [s]');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-40)  max(yl(2),40)]);
yticks([ -40   -20:10:20 40]);
subplot(3,1,3);
plot(t, rad2deg(unwrap(yaw_ned)),   'k','LineWidth',1);  ylabel('Yaw [derece]'); xlabel('t [s]');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-1440)  max(yl(2),180)]);
yticks([ -1440 -1080:360:0  90]);
styleThisFigure(gcf);
figure; 
plot(t, pos_ned, 'LineWidth', 1.4); 
legend('N','E','D','Location','best');
xlabel('t [s]'); ylabel('Pozisyon [m]');
title('Pozisyon (NED)');
styleThisFigure(gcf); 
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-1000)  max(yl(2),5000)]);

%% Hız Profili
figure; hold on
subplot(3,1,1);
plot(t, vel_ned(:,1), 'LineWidth',1);
title('Hız Profili (NED)');
xlabel('t [s]'); ylabel('V_N [m/s]');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-40)  max(yl(2),40)]);
yticks([ -40:10:40]);  
subplot(3,1,2);
plot(t, vel_ned(:,2), 'LineWidth',1);
xlabel('t [s]'); ylabel('V_E [m/s]');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-40)  max(yl(2),40)]);
yticks([ -40:10:40]);  
subplot(3,1,3);
plot(t, vel_ned(:,3), 'LineWidth',1);
xlabel('t [s]'); ylabel('V_D Hız [m/s]');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-30)  max(yl(2),30)]);
yticks([ -30:10:30]);  
styleThisFigure(gcf);

%% İvme Profili
figure; hold on
subplot(3,1,1);
plot(t, acc_ned(:,1), 'LineWidth',1);
title('İvme Profili (NED)');
xlabel('t [s]'); ylabel('a_N [m/s^2]');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-5)  max(yl(2),5)]);
yticks([-5 -3 -1 0 1 3 5]);  
subplot(3,1,2);
plot(t, acc_ned(:,2), 'LineWidth',1);
xlabel('t [s]'); ylabel('a_E  [m/s^2]');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-5)  max(yl(2),5)]);
yticks([-5 -3 -1 0 1 3 5]);  
subplot(3,1,3);
plot(t, acc_ned(:,3)-9.8, 'LineWidth',1);
xlabel('t [s]'); ylabel('a_D  [m/s^2]');
styleThisFigure(gcf);

%% Döküm Tablosu
Tnav = table(t(:), ...
    pos_ned(:,1), pos_ned(:,2), pos_ned(:,3), ...
    vel_ned(:,1), vel_ned(:,2), vel_ned(:,3), ...
    acc_ned(:,1), acc_ned(:,2), acc_ned(:,3), ...
    yaw_ned(:), pitch_ned(:), roll_ned(:), ...
    'VariableNames', {'Time','N','E','D','vN','vE','vD','aN','aE','aD','Yaw','Pitch','Roll'});

%% Faz Süreleri
ph = {};
ph(end+1,:) = {'1a: Pistte hızlanma',             tA};
ph(end+1,:) = {'1b: Düz tırmanış',                T_acc + tB0};
ph(end+1,:) = {'1c: Düz (level)',                 T_acc + T_linTO + tB1};
ph(end+1,:) = {'1d: Helisel tırmanış',            T_acc + T_linTO + T_flatTO + tB};
ph(end+1,:) = {'2: Düz seyir',                    t2};
ph(end+1,:) = {'3: S manevrası',                  t3};
ph(end+1,:) = {'3b: Ekstra düz',                  t3b};
if exist('t3c','var'), ph(end+1,:) = {'3c: Barrel roll', t3c}; end
ph(end+1,:) = {'4: 180° dönüş',                   t4};
ph(end+1,:) = {'4b: Hizalama',                    t4b};
ph(end+1,:) = {'5: Dalış',                        t5};
ph(end+1,:) = {'6: Dalış sonrası düz',            t6};
ph(end+1,:) = {'7: Daire (iniş)',                 t7};
ph(end+1,:) = {'8: Pistte durma',                 t8};
fprintf('\n==== Faz Süreleri ====\n');
totT = t(end) - t(1);
for i = 1:size(ph,1)
    name = ph{i,1}; tv = ph{i,2};
    dur = tv(end) - tv(1);
    fprintf('%-22s Δt = %7.2f s\n', name, dur);
end
fprintf('------------------------ Toplam süre: %.2f s (%.2f dk)\n\n', totT, totT/60);

function styleThisFigure(fig)
set(fig,'Color',[1 1 1]);
ax = findall(fig,'Type','axes');
if ~isempty(ax)
  set(ax, ...
    'Box','off', ...
    'Color',[0.9 0.9 0.9], ...
    'XGrid','on','YGrid','on','ZGrid','on', ...
    'XMinorTick','off','YMinorTick','off','ZMinorTick','off', ...
    'XMinorGrid','off','YMinorGrid','off','ZMinorGrid','off', ...
    'GridColor',[1 1 1],'GridAlpha',1, ...
    'TickDir','out','LineWidth',0.75, ...
    'FontName','Arial','FontSize',11, ...
    'TitleFontWeight','bold','TitleFontSizeMultiplier',0.9);
end
axh = findall(fig,'Type','axes');
cizgiKal =1;
for ax = axh.'
    ln = findall(ax,'Type','line'); ln = flipud(ln);
    if numel(ln)>=1, set(ln(1),'Color','k','LineWidth',cizgiKal); end
    if numel(ln)>=2, set(ln(2),'Color','b','LineWidth',cizgiKal); end
    if numel(ln)>=3, set(ln(3),'Color','r','LineWidth',cizgiKal); end
end
lgd = findall(fig,'Type','legend');
if ~isempty(lgd)
  set(lgd,'Box','on');
end
end
