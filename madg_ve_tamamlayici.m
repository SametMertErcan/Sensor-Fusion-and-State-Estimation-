%%  ------ #3 -----
% SAMET MERT ERCAN, 2025


mag_imu_200 = interp1(t_mag, mag_meas_50, t_imu, 'pchip','extrap');   % [µT]
B0 = median(vecnorm(mag_imu_200,2,2));

wrapA = @(x) mod(x+pi, 2*pi) - pi;
Mfun  = @(ph,th) [ 1, sin(ph)*tan(th),  cos(ph)*tan(th);
                  0, cos(ph),         -sin(ph);
                  0, sin(ph)/cos(th),  cos(ph)/cos(th) ];
RyA = @(a)[cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
RxA = @(a)[1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];

tau_rp   = 1.5;                        % roll/pitch zaman sabiti (gyro ağırlığı ↑)
a_rp     = tau_rp /(tau_rp + dt_imu);  % 0..1
tau_yaw  = 1.0;                        % yaw zaman sabiti
a_yaw    = tau_yaw/(tau_yaw + dt_imu);
acc_gate_rel = 0.03;                   % | |f|-g | < 0.03g ise ivmeye güven
mag_gate_rel = 0.01;                   % | |m|-B0| < 0.01B0 ise manyetoya güven

N = numel(t_imu);
if exist('est_vel','var') && size(est_vel,1)==N
    vN_src = est_vel(:,1);  vE_src = est_vel(:,2);
elseif exist('gps_vel_ned_true_10','var') && ~isempty(gps_vel_ned_true_10)
    vN_src = interp1(t_gps, gps_vel_ned_true_10(:,1), t_imu, 'pchip','extrap');
    vE_src = interp1(t_gps, gps_vel_ned_true_10(:,2), t_imu, 'pchip','extrap');
elseif exist('vel_gt_imu','var') && size(vel_gt_imu,1)==N
    vN_src = vel_gt_imu(:,1); vE_src = vel_gt_imu(:,2);
else
    vN_src = zeros(N,1);     vE_src = zeros(N,1);
end
Vh_200 = hypot(vN_src, vE_src);      % [m/s]

fb1 = acc_meas_200(1,:).';
g_b = -fb1; ng = norm(g_b); if ng>1e-9, g_b = g_b/ng; end
phi0   = atan2( g_b(2), g_b(3) );
theta0 = atan2(-g_b(1), hypot(g_b(2), g_b(3)) );

mb1  = mag_imu_200(1,:).';
mn1  = RyA(theta0)*RxA(phi0)*mb1;
psi0 = deg2rad(90);         % atan2(-E, N)  [NED]

Vmin_turn = 6.0;            % [m/s] altında turn kriterini kullanma
alat_gate = 0.12*g;         % >0.12g ise ivme düzeltmesini bastır
wacc_tau  = 0.8;            % [s] gating LPF zaman sabiti (yumuşak geçiş)

eul_ck = zeros(N,3); eul_ck(1,:) = [phi0 theta0 psi0];

fc_acc   = 2.0;  aa = exp(-2*pi*fc_acc*dt_imu);
a_lp     = acc_meas_200(1,:);
dphi_max = deg2rad(0.20);
dth_max  = deg2rad(0.20);
turn_lpf = 0;

for k = 2:N
    ph = eul_ck(k-1,1); th = eul_ck(k-1,2); ps = eul_ck(k-1,3);

    wb = gyro_meas_200(k-1,:).';
    eul_g = [ph;th;ps] + Mfun(ph,th)*wb*dt_imu;

    pqr = gyro_meas_200(k-1,:).';
    cph = cos(ph); sph = sin(ph); cth = max(cos(th), 1e-6);
    psi_dot = (sph/cth)*pqr(2) + (cph/cth)*pqr(3);       % [rad/s]
    a_lat_est = Vh_200(k-1) * abs(psi_dot);              % [m/s^2]
    turn_raw  = (Vh_200(k-1) > Vmin_turn) .* (a_lat_est/alat_gate);
    alpha_turn = exp(-dt_imu/wacc_tau);
    turn_lpf = alpha_turn*turn_lpf + (1-alpha_turn)*min(1,turn_raw);
    turn_block = min(1, max(0, turn_lpf));               % 0..1

    a_lp = aa*a_lp + (1-aa)*acc_meas_200(k,:);
    err_g = abs(norm(a_lp)-g);
    w_acc_mag = max(0, 1 - err_g/(acc_gate_rel*g));      % |f|-g kapısı
    w_acc = w_acc_mag * (1 - turn_block);                % *** kritik: dönüşte kıs ***

    if w_acc > 1e-3
        gu = (-a_lp)/max(norm(a_lp),1e-9);               % gravity yönü (FRD)
        phi_a   = atan2( gu(2), gu(3) );
        theta_a = atan2(-gu(1), hypot(gu(2),gu(3)));
        dphi = wrapA(phi_a   - eul_g(1));
        dth  = wrapA(theta_a - eul_g(2));
        dphi = max(min(dphi, dphi_max), -dphi_max);
        dth  = max(min(dth , dth_max ), -dth_max );
        ph = eul_g(1) + (1-a_rp)*w_acc * dphi;
        th = eul_g(2) + (1-a_rp)*w_acc * dth;
    else
        ph = eul_g(1); th = eul_g(2);
    end

    mb = mag_imu_200(k,:).';
    if abs(norm(mb)-B0) < mag_gate_rel*B0
        mn = RyA(th)*RxA(ph)*mb;
        psi_mag = atan2(-mn(2), mn(1));           % atan2(-E,N)
        dpsi = wrapA(psi_mag - eul_g(3));
        ps = eul_g(3) + (1-a_yaw)*dpsi;
    else
        ps = eul_g(3);
    end

    eul_ck(k,:) = [wrapA(ph) wrapA(th) wrapA(ps)];
end

q = eul2quat_zyx(phi0, theta0, psi0);      % başlangıç
eul_madg = zeros(N,3);
eul_madg(1,:) = [phi0 theta0 psi0];

beta = 0.0000001;                               % temel Madgwick katsayısı
turn_lpf_m = 0;

for k = 2:N
    wb = gyro_meas_200(k-1,:);     
    ab = acc_meas_200(k-1,:);     
    mb = mag_imu_200(k-1,:);

    ph_prev = eul_madg(k-1,1); th_prev = eul_madg(k-1,2);
    pqr = gyro_meas_200(k-1,:).';
    cph = cos(ph_prev); sph = sin(ph_prev); cth = max(cos(th_prev),1e-6);
    psi_dot = (sph/cth)*pqr(2) + (cph/cth)*pqr(3);
    a_lat_est = Vh_200(k-1) * abs(psi_dot);
    turn_raw  = (Vh_200(k-1) > Vmin_turn) .* (a_lat_est/alat_gate);
    alpha_turn = exp(-dt_imu/wacc_tau);
    turn_lpf_m = alpha_turn*turn_lpf_m + (1-alpha_turn)*min(1,turn_raw);
    turn_block_m = min(1, max(0, turn_lpf_m));

    errg = abs(norm(ab) - 9.80665);
    wacc = max(0, 1 - errg/(0.30*9.80665));
    beta_eff = beta * wacc * (1 - turn_block_m);     % *** ek ağırlık kısma ***

    q  = madgwick_step_ned(q, wb, ab, mb, beta_eff, dt_imu);
    [ph,th,ps] = quat2eul_zyx(q);
    eul_madg(k,:) = [ph th ps];
end

figure;
tl=tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile; 
plot(t_imu,rad2deg(eul_madg(:,1)),'b-', ...
     t_imu,rad2deg(eul_ck(:,1)),'r-', ...
     t_imu,rad2deg(est_ang(:,1)),'g-', ...
     t_imu,rad2deg(ang_gt_imu(:,1)),'k--');
grid on; ylabel('\phi [deg]'); legend('Madg','CF','EKF','GT','Location','best');
title('Roll'); styleThisFigure(gcf);

nexttile; 
plot(t_imu,rad2deg(eul_madg(:,2)),'b-', ...
     t_imu,rad2deg(eul_ck(:,2)),'r-', ...
     t_imu,rad2deg(est_ang(:,2)),'g-', ...
     t_imu,rad2deg(ang_gt_imu(:,2)),'k--');
grid on; ylabel('\theta [deg]'); title('Pitch'); styleThisFigure(gcf);

nexttile; 
plot(t_imu,rad2deg(eul_madg(:,3)),'b-', ...
     t_imu,rad2deg(eul_ck(:,3)),'r-', ...
     t_imu,rad2deg(est_ang(:,3)),'g-', ...
     t_imu,rad2deg(ang_gt_imu(:,3)),'k--');
grid on; ylabel('\psi [deg]'); xlabel('t [s]'); title('Yaw'); styleThisFigure(gcf);

wrapPi = @(a) atan2(sin(a),cos(a));
rmse    = @(e) sqrt(mean(wrapPi(e).^2))*180/pi;

idx = (t_imu >= 100 & t_imu <= 150) | (t_imu >= 280 & t_imu <= 330);
finiteMask = all(isfinite([eul_ck, eul_madg, est_ang, ang_gt_imu]),2);
idx = idx & finiteMask;

e_ck   = eul_ck(idx,:)    - ang_gt_imu(idx,:);
e_madg = eul_madg(idx,:)  - ang_gt_imu(idx,:);
e_ekf  = est_ang(idx,:)   - ang_gt_imu(idx,:);

ck   = [rmse(e_ck(:,1)),   rmse(e_ck(:,2)),   rmse(e_ck(:,3))];
madg = [rmse(e_madg(:,1)), rmse(e_madg(:,2)), rmse(e_madg(:,3))];
ekf  = [rmse(e_ekf(:,1)),  rmse(e_ekf(:,2)),  rmse(e_ekf(:,3))];

fprintf('___________________________________\n');
fprintf('Zaman aralığı: [100,150] ∪ [280,330] s\n');
fprintf('Flt.  roll(deg) pitch(deg) yaw(deg)\n');
fprintf('___________________________________\n');
fprintf('EKF  %6.2f     %6.2f      %6.2f\n', ekf(1), ekf(2), ekf(3));
fprintf('___________________________________\n');
fprintf('CF   %6.2f     %6.2f      %6.2f\n',   ck(1),   ck(2),   ck(3));
fprintf('___________________________________\n');
fprintf('Madg %6.2f     %6.2f      %6.2f\n', madg(1), madg(2), madg(3));

function q = eul2quat_zyx(phi,theta,psi)
c1=cos(psi/2);  s1=sin(psi/2);
c2=cos(theta/2); s2=sin(theta/2);
c3=cos(phi/2);   s3=sin(phi/2);
q = [ c1*c2*c3 + s1*s2*s3, ...
      c1*c2*s3 - s1*s2*c3, ...
      c1*s2*c3 + s1*c2*s3, ...
      s1*c2*c3 - c1*s2*s3 ];
q = q./norm(q);
end

function [phi,theta,psi] = quat2eul_zyx(q)
w=q(1); x=q(2); y=q(3); z=q(4);
R = [1-2*(y^2+z^2),   2*(x*y - z*w),  2*(x*z + y*w);
     2*(x*y + z*w), 1-2*(x^2+z^2),    2*(y*z - x*w);
     2*(x*z - y*w),   2*(y*z + x*w),  1-2*(x^2+y^2)];
theta = asin( -R(3,1) );
phi   = atan2( R(3,2), R(3,3) );
psi   = atan2( R(2,1), R(1,1) );
phi   = mod(phi+pi,2*pi)-pi; 
theta = mod(theta+pi,2*pi)-pi; 
psi   = mod(psi+pi,2*pi)-pi;
end

function qn = madgwick_step_ned(q, wb, ab, mb, beta_eff, dt)
if any(~isfinite(ab)) || norm(ab)==0, ab = [0 0 -1]; end
if any(~isfinite(mb)) || norm(mb)==0, mb = [1 0 0];  end
a = -ab / norm(ab);
m =  mb / norm(mb);

w=q(1); x=q(2); y=q(3); z=q(4);
h  = quatrotate(q, m);
bx = sqrt(h(1)^2 + h(2)^2);  bz = h(3);

f = [ 2*(x*z - w*y)                 - a(1);
      2*(w*x + y*z)                 - a(2);
      2*(0.5 - x^2 - y^2)           - a(3);
      2*bx*(0.5 - y^2 - z^2) + 2*bz*(x*z - w*y) - m(1);
      2*bx*(x*y - w*z)       + 2*bz*(w*x + y*z) - m(2);
      2*bx*(w*y + x*z)       + 2*bz*(0.5 - x^2 - y^2) - m(3) ];

J = [ -2*y,  2*z,  -2*w,  2*x;
       2*x,  2*w,   2*z,  2*y;
       0,   -4*x,  -4*y,  0;
      -2*bz*y,  2*bz*z,  -4*bx*y-2*bz*w,  -4*bx*z+2*bz*x;
      -2*bx*z+2*bz*x,  2*bx*y+2*bz*w,  2*bx*x+2*bz*z,  -2*bx*w+2*bz*y;
       2*bx*y,  2*bx*z-4*bz*x,  2*bx*w-4*bz*y,  2*bx*x ];

step = (J.'*f); 
step = step / max(norm(step),eps);

Omega = [ 0,     -wb(1),    -wb(2),      -wb(3);
          wb(1),      0,     wb(3),      -wb(2);
          wb(2), -wb(3),         0,       wb(1);
          wb(3),  wb(2),    -wb(1),         0 ];

qDot = 0.5*(Omega*q.') - beta_eff*step;
q = q + (qDot.' * dt);
qn = q / norm(q);
end

function v_n = quatrotate(q, v_b)
w=q(1); x=q(2); y=q(3); z=q(4);
R = [1-2*(y^2+z^2),   2*(x*y - z*w),  2*(x*z + y*w);
     2*(x*y + z*w), 1-2*(x^2+z^2),    2*(y*z - x*w);
     2*(x*z - y*w),   2*(y*z + x*w),  1-2*(x^2+y^2)];
v_n = (R * v_b.').';
end

function styleThisFigure(fig)
set(fig,'Color',[1 1 1]);
ax = findall(fig,'Type','axes');
if ~isempty(ax)
  set(ax, 'Box','off','Color',[1 1 1], ...
      'XGrid','on','YGrid','on','ZGrid','on', ...
      'XMinorTick','off','YMinorTick','off','ZMinorTick','off', ...
      'XMinorGrid','on','YMinorGrid','on','ZMinorGrid','on', ...
      'GridColor',[1 1 1],'GridAlpha',1, ...
      'TickDir','out','LineWidth',0.75, ...
      'FontName','Arial','FontSize',11, ...
      'TitleFontWeight','bold','TitleFontSizeMultiplier',0.9);
end
axh = findall(fig,'Type','axes'); cizgiKal = 1;
for ax = axh.'
    ln = findall(ax,'Type','line'); ln = flipud(ln);
    if numel(ln)>=1, set(ln(1),'Color','b','LineWidth',cizgiKal); end
    if numel(ln)>=2, set(ln(2),'Color','r','LineWidth',cizgiKal); end
    if numel(ln)>=3, set(ln(3),'Color','g','LineWidth',cizgiKal); end
    if numel(ln)>=4, set(ln(4),'Color','k','LineWidth',cizgiKal); end
end
lgd = findall(fig,'Type','legend');
if ~isempty(lgd), set(lgd,'Box','off','Fontsize',9); end
end
