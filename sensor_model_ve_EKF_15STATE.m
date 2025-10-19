%%  ------ #2 -----
% SAMET MERT ERCAN, 2025
rng(11);
close all; clc;
clearvars -except  yaw_ned pitch_ned roll_ned pos_ned vel_ned acc_ned eul_ned ...
    t dt fs g...
    %C_ecefNed C_nedEcef pos_ecef vel_ecef acc_ecef   lat0 lon0 h0

set(groot, 'defaultAxesColor', 'w', ...
    'defaultFigureColor', 'w', ...
    'defaultAxesBox', 'on');
% set(groot, ...
%     'defaultAxesXGrid','off', ...
%     'defaultAxesYGrid','off', ...
%     'defaultAxesZGrid','off', ...
%     'defaultAxesXMinorGrid','off', ...
%     'defaultAxesYMinorGrid','off', ...
%     'defaultAxesZMinorGrid','off');

k=0;

%% Ground Truth (NED)
pos_GT   = pos_ned;
vel_GT   = vel_ned;
acc_GT   = acc_ned;
phi_GT   = roll_ned;
theta_GT = pitch_ned;
psi_GT   = yaw_ned;
t_GT     = t(:);
Tend     = t_GT(end);

%% Oranlar (Hz)
fs_imu  = 200;  dt_imu  = 1/fs_imu;   t_imu  = (0:dt_imu:Tend).';  Nimu  = numel(t_imu);
fs_mag  = 50;   dt_mag  = 1/fs_mag;   t_mag  = (0:dt_mag:Tend).';  Nmag  = numel(t_mag);
fs_gps  = 5;   dt_gps  = 1/fs_gps;   t_gps  = (0:dt_gps:Tend).';  Ngps  = numel(t_gps);
fs_baro = 25;   dt_baro = 1/fs_baro;  t_baro = (0:dt_baro:Tend).'; Nbaro = numel(t_baro);

ratio_mag  = fs_imu/fs_mag;
ratio_gps  = fs_imu/fs_gps;
ratio_baro = fs_imu/fs_baro;

%% Dönüşümler
Cnb = @(ph,th,ps) Rz(ps)*Ry(th)*Rx(ph);
Cbn = @(ph,th,ps) Cnb(ph,th,ps).';

%% Euler 200 Hz
ph_u  = unwrap(phi_GT);     th_u  = unwrap(theta_GT);   ps_u = unwrap(psi_GT);
phi_imu   = wrapPi( interp1(t_GT, ph_u,  t_imu, 'pchip','extrap') );
theta_imu = wrapPi( interp1(t_GT, th_u,  t_imu, 'pchip','extrap') );
psi_imu   = wrapPi( interp1(t_GT, ps_u,  t_imu, 'pchip','extrap') );

phi_dot   = gradient(unwrap(phi_imu),  dt_imu);
theta_dot = gradient(unwrap(theta_imu),dt_imu);
psi_dot   = gradient(unwrap(psi_imu),  dt_imu);

p_true =  phi_dot - psi_dot.*sin(theta_imu);
q_true =  theta_dot.*cos(phi_imu) + psi_dot.*sin(phi_imu).*cos(theta_imu);
r_true = -theta_dot.*sin(phi_imu) + psi_dot.*cos(phi_imu).*cos(theta_imu);
omega_b_true_200 = [p_true q_true r_true];

%% İvme (NED→Body)
g_ned = [0;0;+g];
acc_ned_200 = interp1(t_GT, acc_GT, t_imu, 'pchip','extrap');

acc_f_body_true_200 = zeros(Nimu,3);
for k = 1:Nimu
    C_bn_k = Cbn(phi_imu(k), theta_imu(k), psi_imu(k));
    acc_f_body_true_200(k,:) = ( C_bn_k*(acc_ned_200(k,:).' - g_ned) ).';
end

%% Manyetometre (50 Hz)
% BN = 24.8641; BE = 2.5073; BD = 40.7158;  % [µT]
BN = 24.8641;
BE =  2.5073;
BD = 40.7158;
m_n = [BN; BE; BD];

Btot = 47.7733;
incl = deg2rad(58 + 27/60 + 34/3600);
decl = deg2rad( 5 + 45/60 + 30/3600);

BH = Btot * cos(incl);
% BN_check = BH * cos(decl);
% BE_check = BH * sin(decl);
% BD_check = Btot * sin(incl);
% fprintf('ΔN=%.4f µT, ΔE=%.4f µT, ΔD=%.4f µT\n', BN_check-BN, BE_check-BE, BD_check-BD);

phi_mag   = wrapPi( interp1(t_imu, phi_imu,   t_mag, 'pchip') );
theta_mag = wrapPi( interp1(t_imu, theta_imu, t_mag, 'pchip') );
psi_mag   = wrapPi( interp1(t_imu, psi_imu,   t_mag, 'pchip') );

m_b_true_50 = zeros(Nmag,3);
for k = 1:Nmag
    C_bn_k = Cbn(phi_mag(k), theta_mag(k), psi_mag(k));
    m_b_true_50(k,:) = (C_bn_k*m_n).';
end

%% GPS Modeli (ECEF↔NED)
lat0 = 40 + 47/60 + 12/3600;
lon0 = 26 + 36/60 + 24/3600;
h0 = 43.6;

a=6378137; f=1/298.257223563; e2=f*(2-f);
lat0r=deg2rad(lat0); lon0r=deg2rad(lon0);
sl=sin(lat0r); cl=cos(lat0r); sL=sin(lon0r); cL=cos(lon0r);
N0 = a/sqrt(1 - e2*sl^2);
r0_ecef = [(N0+h0)*cl*cL; (N0+h0)*cl*sL; (N0*(1-e2)+h0)*sl];

C_ecefNed = [ -sl*cL, -sl*sL,  cl;
              -sL   ,  cL   ,  0 ;
              -cl*cL, -cl*sL, -sl ];
C_nedEcef = C_ecefNed.';

gps_pos_ned_true_10 = interp1(t_GT, pos_GT, t_gps, 'pchip');
gps_vel_ned_true_10 = interp1(t_GT, vel_GT, t_gps, 'pchip');

gps_pos_ecef_true_10 = gps_pos_ned_true_10 * C_nedEcef.' + r0_ecef.';
gps_vel_ecef_true_10 = gps_vel_ned_true_10 * C_nedEcef.';

sig_gps_N = 4;
sig_gps_E = 4;
sig_gps_D = 5.5;
sig_gps_vHor = 0.25; sig_gps_vVert = 0.35;
sig_pos_ned = [sig_gps_N sig_gps_E sig_gps_D];
sig_vel_ned = [sig_gps_vHor sig_gps_vHor sig_gps_vVert];

noise_pos_ecef = (randn(Ngps,3).*sig_pos_ned) * C_nedEcef.';
noise_vel_ecef = (randn(Ngps,3).*sig_vel_ned) * C_nedEcef.';

gps_pos_meas_ecef_10 = gps_pos_ecef_true_10 + noise_pos_ecef ;
gps_vel_meas_ecef_10 = gps_vel_ecef_true_10 + noise_vel_ecef;

dr_true = gps_pos_ecef_true_10 - r0_ecef.';
dr_meas = gps_pos_meas_ecef_10 - r0_ecef.';
gps_pos_ned_true_10 = dr_true * C_ecefNed.';
gps_pos_meas_10     = dr_meas * C_ecefNed.';
gps_vel_ned_true_10 = gps_vel_ecef_true_10 * C_ecefNed.';
gps_vel_meas_10     = gps_vel_meas_ecef_10 * C_ecefNed.';

R_gps_pos = diag(sig_pos_ned.^2);
R_gps_vel = diag(sig_vel_ned.^2);
R_gps     =1e-6*blkdiag(R_gps_pos, R_gps_vel);

%% Barometre (25 Hz)
p0=101325; T0=288.15; L=0.0065; g0=9.80665; R=287.05; k_isa = g0/(R*L);
D_true_25 = interp1(t_GT, pos_GT(:,3), t_baro, 'pchip');
U_true_25 = -D_true_25;
p_true_25 = p0*(1 - (L*U_true_25)/T0).^k_isa;

sig_p_baro =30;
b0_p_baro  = 0;
baro_p_meas_25 = p_true_25 + b0_p_baro + sig_p_baro*randn(Nbaro,1);
R_baro_p = sig_p_baro^2;

%% IMU Ölçümleri
sig_gyro = deg2rad([0.315 0.315 0.315]);
b0_gyro      = deg2rad([0.20, -0.10, 0.15]);
b0_gyro = deg2rad([0 0 0]);

sigma_bg_rw   = deg2rad(0.00);
bias_true_200 = b0_gyro + cumsum(randn(Nimu,3)*sigma_bg_rw*sqrt(dt_imu),1);

gyro_meas_200 = omega_b_true_200 + bias_true_200 + randn(Nimu,3).*sig_gyro;

sig_acc = [0.15 0.15 0.15];
b0_accel      = [-0.3, 0.3, 0.4];
b0_accel = [0 0 0];

sigma_ba_rw   = 0.00;
bias_acc_true = b0_accel + cumsum(randn(Nimu,3)*sigma_ba_rw*sqrt(dt_imu),1);

acc_meas_200 = acc_f_body_true_200 + bias_acc_true + randn(Nimu,3).*sig_acc;

sigAcc  = sig_acc;
sigGyro = sig_gyro;

sig_m = 0.3*[1 1 1];
b_m   = [0 0 0];
mag_meas_50 = m_b_true_50 + b_m + randn(Nmag,3).*sig_m;

%% Pitot (25 Hz)
fs_pitot = 25;  ratio_pitot = fs_imu / fs_pitot;
t_pitot  = t_imu(1:ratio_pitot:end);
Npitot   = numel(t_pitot);

p_pitot = interp1(t_baro, baro_p_meas_25, t_pitot, 'pchip');
T_pitot = 288.15 + 0*p_pitot;
rho_pitot = p_pitot ./ (287.05*T_pitot);

vN_p = interp1(t_GT, vel_GT(:,1), t_pitot, 'pchip');
vE_p = interp1(t_GT, vel_GT(:,2), t_pitot, 'pchip');
vD_p = interp1(t_GT, vel_GT(:,3), t_pitot, 'pchip');
V_true_p = sqrt(max(vN_p.^2 + vE_p.^2 + vD_p.^2, 1e-9));

q_true = 0.5 .* rho_pitot .* (V_true_p.^2);
b_q    = 0;
sigma_q= 3;
sigmaV = 0.9;
q_meas = (1+0.00).*q_true + b_q + sigma_q.*randn(Npitot,1);

V_meas_pitot = sqrt( 2*max(q_meas,0) ./ max(rho_pitot,1e-3) );

%% Güncelleme Bayrakları
GNSSupdate =1;
MAGupdate = 1;
BAROupdate = 1;
GNSSyawUpd =1;
PITOTupdate=1;
t_UPDATE =700;
GNSSdrop=0;
GNSSspike=0;
baro_hold = false;

%% GNSS Kopma Senaryosu
if GNSSdrop == 1
    drop1 = (t_gps >= 100 & t_gps <= 150);
    drop2 = (t_gps >= 280 & t_gps <= 330);
    mask = drop1 | drop2;
    gps_pos_meas_10(mask, :) = NaN;
    gps_vel_meas_10(mask, :) = NaN;
end
% if GNSSspike == 1
%     win1 = (t_gps >= 200) & (t_gps < 202);
%     win2 = (t_gps >= 400) & (t_gps < 402);
%     maskS = win1 | win2;
%     posSpikeStd = [80 80 120];
%     velSpikeStd = [8  8   5 ];
%     nS = nnz(maskS);
%     gps_pos_meas_10(maskS, :) = gps_pos_meas_10(maskS, :) + randn(nS,3).*posSpikeStd;
%     gps_vel_meas_10(maskS, :) = gps_vel_meas_10(maskS, :) + randn(nS,3).*velSpikeStd;
% end

%% EKF Durum (15)
nx = 15; I = eye(nx);
xhat = zeros(Nimu,nx);

xhat(1,1:3)   = gps_pos_meas_10(1,:);
xhat(1,4:6)   = gps_vel_meas_10(1,:);
xhat(1,7:9)   = [phi_imu(1) theta_imu(1) psi_imu(1)];
xhat(1,10:12) = [0 0 0];
xhat(1,13:15) = [0 0 0];

%% P0
sig_roll0=deg2rad(0.2);
sig_pitch0=deg2rad(0.2);
sig_yaw0=deg2rad(0.2);

P9 = diag([sig_gps_N^2, sig_gps_E^2, sig_gps_D^2, ...
    sig_gps_vHor^2, sig_gps_vHor^2, sig_gps_vVert^2, ...
    sig_roll0^2, sig_pitch0^2, sig_yaw0^2]);

% sig_bg0 = deg2rad([0.04 -0.04 0.04]);
% sig_ba0 = [0.3 0.3 0.35];
sig_bg0 = deg2rad([0.1 0.1 0.1]);
sig_ba0 = [0.8 0.8 0.8];
sig_bg0 = [0 0 0 ];
sig_ba0 = [0 0 0];

P = blkdiag(P9, diag(sig_bg0.^2), diag(sig_ba0.^2));
Phist = nan(Nimu,nx); Phist(1,:) = diag(P).';

%% Q
q_p = 0.005;
q_v = 0.04;
q_ang = deg2rad(0.02);
% q_ang = 1 * sig_gyro(1) * sqrt(fs_imu);

qb_g_rw = deg2rad(0.0004);
qb_a_rw = 0.0008;

qb_g_rw = 0;
qb_a_rw = 0.0;

Qc  = diag([q_p^2*ones(1,3), q_v^2*ones(1,3), q_ang^2*ones(1,3)]);
Q9  = Qc * dt_imu;

Qbg     = (qb_g_rw^2)*dt_imu*eye(3);
Qba     = (qb_a_rw^2)*dt_imu*eye(3);

Q = blkdiag(Q9, Qbg, Qba);

R_mag  = diag((8*sig_m).^2);
% R_gps_pos = diag([sig_gps_N^2, sig_gps_E^2, sig_gps_D^2]);
% R_gps_vel = diag([sig_gps_vHor^2, sig_gps_vHor^2, sig_gps_vVert^2]);
% R_gps = blkdiag(R_gps_pos, R_gps_vel);
% R_baro_p = sig_p_baro^2;
% R_pitot = sigmaV^2;

%% Ölçüm Fonksiyonları
f_fun  = @(x,u) f_discrete_NED_BGa(x,u,dt_imu,g);
F_ana  = @(x,u) F_analytic_NED_BGa(x,u,dt_imu);

h_mag15   = @(x)   h_mag_vec_NED(x(1:9), m_n);
H_magA15  = @(x)  [H_mag_analytic_NED(x(1:9), m_n), zeros(3,6)];

h_pv15    = @(x)   [x(1:3); x(4:6)];
H_pv15    = @(x)   [eye(6), zeros(6,9)];

pos_gt_imu = interp1(t_GT, pos_GT, t_imu, 'pchip');
vel_gt_imu = interp1(t_GT, vel_GT, t_imu, 'pchip');
ang_gt_imu = [phi_imu theta_imu psi_imu];

sig_mag_gate = 3 * sqrt(diag(R_mag));
ev = repmat("P", Nimu, 1);

Kg_MAG   = nan(Nimu,1); Kang_MAG  = nan(Nimu,1);
Kg_GPS   = nan(Nimu,1); Kpos_GPS  = nan(Nimu,1); Kvel_GPS = nan(Nimu,1); Kang_GPS = nan(Nimu,1);
Kg_YAW   = nan(Nimu,1); Kpsi_YAW  = nan(Nimu,1);
Kg_BARO  = nan(Nimu,1); KD_BARO   = nan(Nimu,1);
Kg_PITOT = nan(Nimu,1); Kv_PITOT  = nan(Nimu,1);

for k = 1:Nimu-1
    % Q=  Q_from_IMU_noise_NED_15(xhat(k,:).', dt_imu, fs_imu, sigAcc, sigGyro, sigma_bg_rw,sigma_bg_rw);
    % Q = Q_simple_Euler15(dt_imu, q_p, q_v, q_ang, qb_g_rw, qb_a_rw);

    u_k   = [acc_meas_200(k,:).'; gyro_meas_200(k,:).'];
    x_pred = f_fun(xhat(k,:).', u_k);
    Fk     = F_ana(xhat(k,:).', u_k);
    P_pred = Fk*P*Fk.' + Q;

    x_upd = x_pred;
    P_upd = P_pred;
    idx = k+1;

    if k < t_UPDATE*200

        if mod(k-1, ratio_mag) == 0 && MAGupdate == 1
            jmag = (k-1)/ratio_mag + 1;
            zmag = mag_meas_50(jmag,:).';
            zhat = h_mag15(x_upd);
            Hk   = H_magA15(x_upd);

            innov = zmag - zhat;
            S = Hk*P_upd*Hk.' + R_mag;
            K = P_upd*Hk'/S;

            Kg_MAG(idx)  = norm(K,'fro');
            Kang_MAG(idx)= mean(vecnorm(K(7:9,:),2,2));

            x_upd = x_upd + K*innov;
            x_upd(7:9) = wrapPi(x_upd(7:9).');
            P_upd = (I - K*Hk)*P_upd*(I - K*Hk).' + K*R_mag*K.';
            ev(idx) = ev(idx) + "+M";
        end

        % if mod(k-1, ratio_mag) == 0 && MAGupdate == 1
        %     jmag = (k-1)/ratio_mag + 1;
        %     zmag = mag_meas_50(jmag,:).';
        %     zhat = h_mag15(x_upd);
        %     Hk   = H_magA15(x_upd);
        %     innov = zmag - zhat;
        %     sig_mag_gate_scalar = 3 * sqrt(max(diag(R_mag)));
        %     if all(abs(innov) <= sig_mag_gate_scalar)
        %         S = Hk*P_upd*Hk.' + R_mag;
        %         K = P_upd*Hk'/S;
        %         x_upd = x_upd + K*innov;
        %         x_upd(7:9) = wrapPi(x_upd(7:9).');
        %         P_upd = (I - K*Hk)*P_upd*(I - K*Hk).' + K*R_mag*K.';
        %         P_upd = (I - K*Hk)*P_upd;
        %         ev(idx) = ev(idx) + "+M";
        %     else
        %         ev(idx) = ev(idx) + "+P";
        %     end
        % end

        gps_available = false;
        if mod(k-1, ratio_gps) == 0 && GNSSupdate == 1
            jgps = (k-1)/ratio_gps + 1;
            if all(~isnan(gps_pos_meas_10(jgps,:)))
                gps_available = true;

                zgps = [gps_pos_meas_10(jgps,:).'; gps_vel_meas_10(jgps,:).'];
                zhat = h_pv15(x_upd);
                Hk   = H_pv15(x_upd);

                innov = zgps - zhat;
                S = Hk*P_upd*Hk.' + R_gps;
                K = P_upd*Hk'/S;
                x_upd = x_upd + K*innov;
                x_upd(7:9) = wrapPi(x_upd(7:9).');
                P_upd = (I - K*Hk)*P_upd*(I - K*Hk).' + K*R_gps*K.';
                ev(idx) = ev(idx) + "+G";

                vN = gps_vel_meas_10(jgps,1);  vE = gps_vel_meas_10(jgps,2);
                Vh = hypot(vN, vE);
                Vmin = 30.0;
                if Vh > Vmin  && GNSSyawUpd == 1
                    psi_gps = atan2(vE, vN);
                    sigma_yaw_gps = max(deg2rad(1.5), min(deg2rad(45), atan2(sig_gps_vHor, Vh)));
                    R_yaw = sigma_yaw_gps^2;

                    Hpsi = zeros(1,nx); Hpsi(9)=1;
                    innov_psi = wrapPi( psi_gps - x_upd(9) );
                    Spsi = Hpsi*P_upd*Hpsi.' + R_yaw;
                    Kpsi = (P_upd*Hpsi.')/Spsi;
                    x_upd = x_upd + Kpsi*innov_psi;
                    x_upd(7:9) = wrapPi(x_upd(7:9).');
                    P_upd = (I - Kpsi*Hpsi)*P_upd*(I - Kpsi*Hpsi).' + Kpsi*R_yaw*Kpsi.';
                    ev(idx) = ev(idx) + "+Y";

                    Kg_YAW(idx)  = norm(Kpsi);
                    Kpsi_YAW(idx)= abs(Kpsi(9));
                end

                Kg_GPS(idx)  = norm(K,'fro');
                Kpos_GPS(idx)= mean(vecnorm(K(1:3 ,:),2,2));
                Kvel_GPS(idx)= mean(vecnorm(K(4:6 ,:),2,2));
                Kang_GPS(idx)= mean(vecnorm(K(7:9 ,:),2,2));
            end
        end

        if mod(k-1, ratio_gps) == 0
            if gps_available
                baro_hold = false;
            else
                baro_hold = true;
            end
        end

        if baro_hold && mod(k-1, ratio_baro) == 0 && BAROupdate == 1
            jbaro = (k-1)/ratio_baro + 1;
            zbaro_p = baro_p_meas_25(jbaro,1);

            Dhat = x_upd(3);
            Uhat = -Dhat;
            oneMinus = 1 - (L*Uhat)/T0;
            zhat = p0 * (oneMinus)^k_isa;

            dp_dU = - (k_isa*L/T0) * p0 * (oneMinus)^(k_isa - 1);
            dp_dD = -dp_dU;

            Hk = zeros(1,nx); Hk(3) = dp_dD;
            innov = zbaro_p - zhat;
            S  = Hk*P_upd*Hk.' + R_baro_p;
            K  = (P_upd*Hk.')/S;
            Kg_BARO(idx) = norm(K);
            KD_BARO(idx) = abs(K(3));

            x_upd = x_upd + K*innov;
            x_upd(7:9) = wrapPi(x_upd(7:9).');
            P_upd = (I - K*Hk)*P_upd*(I - K*Hk).' + K*R_baro_p*K.';
            ev(idx) = ev(idx) + "+B";
        end

        if baro_hold && mod(k-1, ratio_pitot) == 0 && PITOTupdate == 1
            jpit = (k-1)/ratio_pitot + 1;
            Vmeas = V_meas_pitot(jpit,1);

            vN = x_upd(4); vE = x_upd(5); vD = x_upd(6);
            vnorm = sqrt(max(vN^2 + vE^2 + vD^2, 1e-6));
            zhat  = vnorm;
            if zhat > 10
                Hk = zeros(1,nx);
                Hk(4) = vN / vnorm;  Hk(5) = vE / vnorm;  Hk(6) = vD / vnorm;

                R_pitot = sigmaV^2;
                innov = Vmeas - zhat;
                S  = Hk*P_upd*Hk.' + R_pitot;
                K  = (P_upd*Hk.')/S;

                Kg_PITOT(idx) = norm(K);
                Kv_PITOT(idx) = norm(K(4:6));

                x_upd = x_upd + K*innov;
                x_upd(7:9) = wrapPi(x_upd(7:9).');
                P_upd = (I - K*Hk)*P_upd*(I - K*Hk).' + K*R_pitot*K.';
                ev(idx) = ev(idx) + "+P";
            end
        end
    end

    Phist(k+1,:) = diag(P_upd).';
    xhat(k+1,:)  = x_upd.';
    P = P_upd;
end

didMAG  = contains(ev, "+M");
didGPS  = contains(ev, "+G");
didYAW  = contains(ev, "+Y");
didBARO = contains(ev, "+B");
didPITOT = contains(ev, "+P");

%% Event Raster
figure; hold on
plot(t_imu(didMAG),   zeros(nnz(didMAG),1)+0, '.', 'MarkerSize',5, 'DisplayName','MAG');
title('EKF: hangi adımda hangi güncelleme?');
styleThisFigure(gca);
plot(t_imu(didGPS),   zeros(nnz(didGPS),1)+1, '.', 'MarkerSize',5, 'DisplayName','GPS');
styleThisFigure(gca);
plot(t_imu(didYAW),   zeros(nnz(didYAW),1)+2, '.', 'MarkerSize',5, 'DisplayName','YAW');
styleThisFigure(gca);
plot(t_imu(didBARO),  zeros(nnz(didBARO),1)+3, '.', 'MarkerSize',5, 'DisplayName','BARO');
styleThisFigure(gca);
plot(t_imu(didPITOT),  zeros(nnz(didPITOT),1)+4, '.', 'MarkerSize',5, 'DisplayName','PITOT');
styleThisFigure(gca);
yticks(0:4); yticklabels({'MAG','GNSS','YAW(GNSS)','BARO','PITOT'});
ylim([-0.5 4.5]); xlim([t_imu(1) t_imu(end)]);
xlabel('t [s]');

%% Kazanç Takibi
figure('Name','Kalman Kazancı — Frobenius Normları'); hold on
plot(t_imu(didMAG),   Kg_MAG(didMAG),   '.-k','DisplayName','MAG');
plot(t_imu(didGPS),   Kg_GPS(didGPS),   '.-b','DisplayName','GPS');
plot(t_imu(didYAW),   Kg_YAW(didYAW),   '.-m','DisplayName','YAW');
plot(t_imu(didBARO),  Kg_BARO(didBARO), '.-g','DisplayName','BARO');
plot(t_imu(didPITOT), Kg_PITOT(didPITOT),'.-c','DisplayName','PITOT');
grid on; legend show; xlabel('t [s]'); ylabel('||K||_F'); axis tight

figure('Name','Kalman Kazancı — Blok Etkileri');
subplot(3,1,1); hold on
plot(t_imu(didMAG), Kang_MAG(didMAG), '.-k','DisplayName','MAG→att');
plot(t_imu(didGPS), Kang_GPS(didGPS), '.-b','DisplayName','GPS→att');
ylabel('Attitude gain'); grid on; legend show
subplot(3,1,2); hold on
plot(t_imu(didGPS), Kpos_GPS(didGPS), '.-b','DisplayName','GPS pos');
plot(t_imu(didGPS), Kvel_GPS(didGPS), '.-r','DisplayName','GPS vel');
ylabel('GPS pos/vel'); grid on; legend show
subplot(3,1,3); hold on
plot(t_imu(didYAW),  Kpsi_YAW(didYAW), '.-m','DisplayName','Yaw→\psi');
plot(t_imu(didBARO), KD_BARO(didBARO), '.-g','DisplayName','Baro→D');
plot(t_imu(didPITOT),Kv_PITOT(didPITOT),'.-c','DisplayName','Pitot→v');
ylabel('Single-meas'); xlabel('t [s]'); grid on; legend show

%% Gyro Bias Kestirim
est_bias = xhat(:,10:12);
gyro_bias_err = est_bias - bias_true_200;
rmse_b_p = rad2deg( sqrt(mean(gyro_bias_err(:,1).^2)) );
rmse_b_q = rad2deg( sqrt(mean(gyro_bias_err(:,2).^2)) );
rmse_b_r = rad2deg( sqrt(mean(gyro_bias_err(:,3).^2)) );
fprintf('RMSE gyro bias [p q r] : %.3f  %.3f  %.3f deg/s\n', rmse_b_p, rmse_b_q, rmse_b_r);

figure; lll=1.2;
subplot(3,1,1); plot(t_imu, rad2deg(bias_true_200(:,1)), t_imu, rad2deg(est_bias(:,1)));
ylabel('b_p [deg/s]'); legend('True','EKF'); grid off;xlabel('t [s]');
title('Jiroskop Bias Kestirimi (EKF vs Gerçek)');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.02)  max(yl(2),0.02)]);
styleThisFigure(gcf);
subplot(3,1,2); plot(t_imu, rad2deg(bias_true_200(:,2)), t_imu, rad2deg(est_bias(:,2)));
ylabel('b_q [deg/s]');xlabel('t [s]');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.02)  max(yl(2),0.02)]);
styleThisFigure(gcf);
subplot(3,1,3); plot(t_imu, rad2deg(bias_true_200(:,3)), t_imu, rad2deg(est_bias(:,3)));
ylabel('b_r [deg/s]'); xlabel('t [s]');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.02)  max(yl(2),0.02)]);
styleThisFigure(gcf);

%% Accel Bias Kestirim
est_ba = xhat(:,13:15);
acc_ba_err = est_ba - bias_acc_true;
rmse_ba = sqrt(mean(acc_ba_err.^2,1));
fprintf('RMSE accel bias [x y z] : %.3f  %.3f  %.3f m/s^2\n', rmse_ba(1), rmse_ba(2), rmse_ba(3));

figure;
subplot(3,1,1); plot(t_imu, bias_acc_true(:,1), t_imu, est_ba(:,1)); ylabel('b_{ax} [m/s^2]'); legend('True','EKF');xlabel('t [s]');
title('İvmeölçer Bias Kestirimi (EKF vs Gerçek)');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.01)  max(yl(2),0.01)]);
styleThisFigure(gcf);
subplot(3,1,2); plot(t_imu, bias_acc_true(:,2), t_imu, est_ba(:,2)); ylabel('b_{ay} [m/s^2]');xlabel('t [s]');legend('True','EKF');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.01)  max(yl(2),0.01)]);
styleThisFigure(gcf);
subplot(3,1,3); plot(t_imu, bias_acc_true(:,3), t_imu, est_ba(:,3)); ylabel('b_{az} [m/s^2]'); xlabel('t [s]');legend('True','EKF');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.01)  max(yl(2),0.01)]);
styleThisFigure(gcf);

%% Kovaryans Diyagonali
NNN = min(length(t_imu), size(Phist,1));
tttt = t_imu(1:NNN);
Pdiag = Phist(1:NNN, :);

ssigma_pos = sqrt(Pdiag(:,1:3));
ssigma_vel = sqrt(Pdiag(:,4:6));
ssigma_ang = sqrt(Pdiag(:,7:9));
ssigma_gyro_bias =sqrt(Pdiag(:,10:12));
ssigma_acc_bias =sqrt(Pdiag(:,13:15));
ssigma_ang_deg = rad2deg(ssigma_ang);

set(groot,'defaultFigureColor','w', ...
    'defaultAxesFontName','Helvetica', ...
    'defaultAxesFontSize',11, ...
    'defaultLineLineWidth',1.6);

figure('Name','Konum belirsizliği (NED)');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(tttt, ssigma_pos(:,1),'r'); grid on; grid minor;
ylabel('N konum sigma [m]');
title('Konum belirsizliği (NED)');
nexttile; plot(tttt, ssigma_pos(:,2),'g'); grid on; grid minor;
ylabel('E konum sigma [m]');
nexttile; plot(tttt, ssigma_pos(:,3),'b'); grid on; grid minor;
ylabel('D konum sigma [m]'); xlabel('zaman [s]');

figure('Name','Hız belirsizliği (NED)');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(tttt, ssigma_vel(:,1),'r'); grid on; grid minor;
ylabel('N hız sigma [m/s]');
title('Hız belirsizliği (NED)');
nexttile; plot(tttt, ssigma_vel(:,2),'g'); grid on; grid minor;
ylabel('E hız sigma [m/s]');
nexttile; plot(tttt, ssigma_vel(:,3),'b'); grid on; grid minor;
ylabel('D hız sigma [m/s]'); xlabel('zaman [s]');

figure('Name','Duruş (Euler) belirsizliği');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(tttt, ssigma_ang_deg(:,1),'r'); grid on; grid minor;
ylabel('Yatış sigma [deg] (phi)');
title('Duruş (Euler) belirsizliği — derece');
nexttile; plot(tttt, ssigma_ang_deg(:,2),'g'); grid on; grid minor;
ylabel('Yunuslama sigma [deg] (teta)');
nexttile; plot(tttt, ssigma_ang_deg(:,3),'b'); grid on; grid minor;
ylabel('Sapma sigma [deg] (psi)'); xlabel('zaman [s]');

%% Sonuçlar
est_pos = xhat(:,1:3);
est_vel = xhat(:,4:6);
est_ang = xhat(:,7:9);
est_gyro_bias =xhat(:,10:12);
est_accel_bias =xhat(:,13:15);

pos_err = est_pos - pos_gt_imu;
vel_err = est_vel - vel_gt_imu;
ang_err = wrapPi(est_ang - ang_gt_imu);
acc_ba_err=acc_ba_err;
gyro_bias_err=gyro_bias_err;

rmse_pos_N = sqrt(mean(pos_err(:,1).^2));
rmse_pos_E = sqrt(mean(pos_err(:,2).^2));
rmse_pos_D = sqrt(mean(pos_err(:,3).^2));
rmse_vel_N = sqrt(mean(vel_err(:,1).^2));
rmse_vel_E = sqrt(mean(vel_err(:,2).^2));
rmse_vel_D = sqrt(mean(vel_err(:,3).^2));
rmse_phi   = rad2deg( sqrt(mean(ang_err(:,1).^2)) );
rmse_theta = rad2deg( sqrt(mean(ang_err(:,2).^2)) );
rmse_psi   = rad2deg( sqrt(mean(ang_err(:,3).^2)) );

fprintf('RMSE pos [N E D]  :   %.3f  %.3f  %.3f m\n',  rmse_pos_N, rmse_pos_E, rmse_pos_D);
fprintf('RMSE vel [N E D]  :   %.3f  %.3f  %.3f m/s\n', rmse_vel_N, rmse_vel_E, rmse_vel_D);
fprintf('RMSE ang [φ θ ψ]  :   %.3f  %.3f  %.3f deg\n', rmse_phi, rmse_theta, rmse_psi);

%% Maksimum Hatalar
if ~exist('tt','var') || numel(tt)~=size(est_pos,1), tt = t_imu(:); end
[max_pos_N, iN] = max(abs(pos_err(:,1)));  t_pos_N = tt(iN);
[max_pos_E, iE] = max(abs(pos_err(:,2)));  t_pos_E = tt(iE);
[max_pos_D, iD] = max(abs(pos_err(:,3)));  t_pos_D = tt(iD);
[max_vel_N, jN] = max(abs(vel_err(:,1)));  t_vel_N = tt(jN);
[max_vel_E, jE] = max(abs(vel_err(:,2)));  t_vel_E = tt(jE);
[max_vel_D, jD] = max(abs(vel_err(:,3)));  t_vel_D = tt(jD);
[max_phi_rad,   k1] = max(abs(ang_err(:,1)));  t_phi   = tt(k1);  max_phi_deg   = rad2deg(max_phi_rad);
[max_theta_rad, k2] = max(abs(ang_err(:,2)));  t_theta = tt(k2);  max_theta_deg = rad2deg(max_theta_rad);
[max_psi_rad,   k3] = max(abs(ang_err(:,3)));  t_psi   = tt(k3);  max_psi_deg   = rad2deg(max_psi_rad);

degChar = char(176);
line1 = repmat('=',1,54);
line2 = repmat('-',1,54);

fprintf('\nMaksimum Mutlak Hatalar (bileşen bazında) — NED\n%s\n', line2);
fprintf('%-10s %-14s %12s\n', 'Bileşen', 'Max Hata', 'Zaman [s]');
fprintf('%s\n', line2);
fprintf('%-10s %10.2f m   %12.3f\n', 'POS N', max_pos_N, t_pos_N);
fprintf('%-10s %10.2f m   %12.3f\n', 'POS E', max_pos_E, t_pos_E);
fprintf('%-10s %10.2f m   %12.3f\n', 'POS D', max_pos_D, t_pos_D);
fprintf('%-10s %10.2f m/s %12.3f\n', 'VEL N', max_vel_N, t_vel_N);
fprintf('%-10s %10.2f m/s %12.3f\n', 'VEL E', max_vel_E, t_vel_E);
fprintf('%-10s %10.2f m/s %12.3f\n', 'VEL D', max_vel_D, t_vel_D);
fprintf('%-10s %9.2f %s  %12.3f\n', 'ANG \phi',   max_phi_deg,   degChar, t_phi);
fprintf('%-10s %9.2f %s  %12.3f\n', 'ANG \theta', max_theta_deg, degChar, t_theta);
fprintf('%-10s %9.2f %s  %12.3f\n', 'ANG \psi',   max_psi_deg,   degChar, t_psi);

%% EKF Sonuç Grafikleri
lll=1;
figure;
subplot(3,1,1);
plot(t_imu, est_pos(:,1),'r', t_imu, pos_gt_imu(:,1),'k', 'LineWidth', lll);
legend('EKF','GT'); ylabel('N [m]'); xlabel('t [s]');
title('EKF Sonuçları (Konum)');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,2);
plot(t_imu, est_pos(:,2),'r', t_imu, pos_gt_imu(:,2),'k', 'LineWidth', lll);
legend('EKF','GT'); ylabel('E [m]'); xlabel('t [s]');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,3);
plot(t_imu, est_pos(:,3),'r', t_imu, pos_gt_imu(:,3),'k', 'LineWidth', lll);
legend('EKF','GT'); ylabel('D [m]'); xlabel('t [s]');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);

figure;
subplot(3,1,1);
plot(t_imu, est_vel(:,1),'r', t_imu, vel_gt_imu(:,1),'k', 'LineWidth', lll);
ylabel('vN [m/s]');legend('EKF','GT'); xlabel('t [s]');
title('EKF Sonuçları (Hız)');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,2);
plot(t_imu, est_vel(:,2),'r', t_imu, vel_gt_imu(:,2),'k', 'LineWidth', lll);
ylabel('vE [m/s]'); legend('EKF','GT'); xlabel('t [s]');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,3);
plot(t_imu, est_vel(:,3),'r', t_imu, vel_gt_imu(:,3),'k', 'LineWidth', lll);
ylabel('vD [m/s]'); legend('EKF','GT'); xlabel('t [s]');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);

figure;
subplot(3,1,1);
plot(t_imu, rad2deg(est_ang(:,1)),'r', t_imu, rad2deg(ang_gt_imu(:,1)),'k', 'LineWidth', lll);
ylabel('\phi [deg]'); legend('EKF','GT'); xlabel('t [s]');
title('EKF Sonuçları (Duruş Açıları)');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,2);
plot(t_imu, rad2deg(est_ang(:,2)),'r', t_imu, rad2deg(ang_gt_imu(:,2)),'k', 'LineWidth', lll);
ylabel('\theta [deg]'); legend('EKF','GT'); xlabel('t [s]');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,3);
plot(t_imu, rad2deg(est_ang(:,3)),'r', t_imu, rad2deg(ang_gt_imu(:,3)),'k', 'LineWidth', lll);
ylabel('\psi [deg]'); legend('GT','EKF'); xlabel('t [s]');
axis tight;
styleThisFigure(gcf);

%% Hata Grafikleri (+/−3σ)
pos_err = est_pos - pos_gt_imu;
vel_err = est_vel - vel_gt_imu;
ang_err = wrapPi(est_ang - ang_gt_imu);
ang_err_deg = rad2deg(ang_err);

rmse_pos_N = sqrt(mean(pos_err(:,1).^2));  rmse_pos_E = sqrt(mean(pos_err(:,2).^2));  rmse_pos_D = sqrt(mean(pos_err(:,3).^2));
rmse_vel_N = sqrt(mean(vel_err(:,1).^2));  rmse_vel_E = sqrt(mean(vel_err(:,2).^2));  rmse_vel_D = sqrt(mean(vel_err(:,3).^2));
rmse_phi   = sqrt(mean(ang_err_deg(:,1).^2));
rmse_theta = sqrt(mean(ang_err_deg(:,2).^2));
rmse_psi   = sqrt(mean(ang_err_deg(:,3).^2));

zline = @(tt) plot(tt, zeros(size(tt)), 'Color', 'r', 'LineStyle', '--');

figure;
subplot(3,1,1); hold on
zline(t);plot(t_imu, pos_err(:,1));
plot(t_imu,  3*ssigma_pos(:,1));
plot(t_imu, -3*ssigma_pos(:,1));
ylabel('e_N [m]');  xlabel('t [s]');legend('0','hata','\pm 3\sigma belirsizlik');
title(sprintf('Position Error — RMSE: N=%.2f m, E=%.2f m, D=%.2f m', rmse_pos_N, rmse_pos_E, rmse_pos_D));
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,2); hold on
zline(t);plot(t_imu, pos_err(:,2));
plot(t_imu,  3*ssigma_pos(:,2));
plot(t_imu, -3*ssigma_pos(:,2));
ylabel('e_E [m]');  xlabel('t [s]');legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,3); hold on
zline(t);plot(t_imu, pos_err(:,3));
plot(t_imu,  3*ssigma_pos(:,3));
plot(t_imu, -3*ssigma_pos(:,3));
ylabel('e_D [m]'); xlabel('t [s]');legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);

figure;
subplot(3,1,1); hold on
zline(t);plot(t_imu, vel_err(:,1));
plot(t_imu,  3*ssigma_vel(:,1));
plot(t_imu, -3*ssigma_vel(:,1));
ylabel('e_{vN} [m/s]'); xlabel('t [s]');legend('0','hata','\pm 3\sigma belirsizlik');
title(sprintf('Velocity Error — RMSE: vN=%.2f, vE=%.2f, vD=%.2f m/s', rmse_vel_N, rmse_vel_E, rmse_vel_D));
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,2); hold on
zline(t);plot(t_imu, vel_err(:,2));
plot(t_imu,  3*ssigma_vel(:,2));
plot(t_imu, -3*ssigma_vel(:,2));
ylabel('e_{vE} [m/s]'); xlabel('t [s]');legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,3); hold on
zline(t);plot(t_imu, vel_err(:,3));
plot(t_imu,  3*ssigma_vel(:,3));
plot(t_imu, -3*ssigma_vel(:,3));
ylabel('e_{vD} [m/s]'); xlabel('t [s]');legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);

figure;
subplot(3,1,1); hold on
zline(t);plot(t_imu, ang_err_deg(:,1));
plot(t_imu,  3*ssigma_ang_deg(:,1));
plot(t_imu, -3*ssigma_ang_deg(:,1));
ylabel('e_{\phi} [deg]'); xlabel('t [s]'); legend('0','hata','\pm 3\sigma belirsizlik');
title(sprintf('Attitude Error — RMSE: \\phi=%.2f°, \\theta=%.2f°, \\psi=%.2f°', rmse_phi, rmse_theta, rmse_psi));
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,2); hold on
zline(t);plot(t_imu, ang_err_deg(:,2));
plot(t_imu,  3*ssigma_ang_deg(:,2));
plot(t_imu, -3*ssigma_ang_deg(:,2));
ylabel('e_{\theta} [deg]');xlabel('t [s]'); legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);
subplot(3,1,3); hold on
zline(t);plot(t_imu, ang_err_deg(:,3));
plot(t_imu,  3*ssigma_ang_deg(:,3));
plot(t_imu, -3*ssigma_ang_deg(:,3));
ylabel('e_{\psi} [deg]'); xlabel('t [s]'); legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
styleThisFigure(gcf);

%% Bias Hata Grafikleri (Gyro)
gyro_bias_err=rad2deg(gyro_bias_err);
ssigma_gyro_bias =rad2deg(ssigma_gyro_bias);
figure;
subplot(3,1,1); hold on
zline(t);plot(t_imu, gyro_bias_err(:,1));
plot(t_imu,  3*ssigma_gyro_bias(:,1));
plot(t_imu, -3*ssigma_gyro_bias(:,1));
ylabel('error gyro bias p [deg/s]'); xlabel('t [s]'); legend('0','hata','\pm 3\sigma belirsizlik');
title(sprintf('Jiroskop Bias Hatası — RMSE: bg_p=%.3f°/s, bg_q=%.3f°/s, bg_r=%.3f°/s', rmse_b_p, rmse_b_q, rmse_b_r));
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.03)  max(yl(2),0.03)]);
styleThisFigure(gcf);
subplot(3,1,2); hold on
zline(t);plot(t_imu, gyro_bias_err(:,2));
plot(t_imu,  3*ssigma_gyro_bias(:,2));
plot(t_imu, -3*ssigma_gyro_bias(:,2));
ylabel('error gyro bias q [deg/s]');xlabel('t [s]'); legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.03)  max(yl(2),0.03)]);
styleThisFigure(gcf);
subplot(3,1,3); hold on
zline(t);plot(t_imu, gyro_bias_err(:,3));
plot(t_imu,  3*ssigma_gyro_bias(:,3));
plot(t_imu, -3*ssigma_gyro_bias(:,3));
ylabel('error gyro bias r [deg/s]');xlabel('t [s]'); legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.03)  max(yl(2),0.03)]);
styleThisFigure(gcf);

%% Bias Hata Grafikleri (Accel)
figure;
subplot(3,1,1); hold on
zline(t);plot(t_imu, acc_ba_err(:,1));
plot(t_imu,  3*ssigma_acc_bias(:,1));
plot(t_imu, -3*ssigma_acc_bias(:,1));
ylabel('error acc bias x [m/s^2]'); xlabel('t [s]'); legend('0','hata','\pm 3\sigma belirsizlik');
title(sprintf('İvmeölçer Bias Hatası — RMSE: ba_x=%.3fm/s^2, ba_y=%.3fm/s^2, ba_z=%.3fm/s^2', rmse_ba(1), rmse_ba(2), rmse_ba(3)));
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.06)  max(yl(2),0.06)]);
styleThisFigure(gcf);
subplot(3,1,2); hold on
zline(t);plot(t_imu, acc_ba_err(:,2));
plot(t_imu,  3*ssigma_acc_bias(:,2));
plot(t_imu, -3*ssigma_acc_bias(:,2));
ylabel('error acc bias y [m/s^2]');xlabel('t [s]'); legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.06)  max(yl(2),0.06)]);
styleThisFigure(gcf);
subplot(3,1,3); hold on
zline(t);plot(t_imu, acc_ba_err(:,3));
plot(t_imu,  3*ssigma_acc_bias(:,3));
plot(t_imu, -3*ssigma_acc_bias(:,3));
ylabel('error acc bias z [m/s^2]');xlabel('t [s]'); legend('0','hata','\pm 3\sigma belirsizlik');
axis tight;
xticks([0:60:540 617]);
yl = ylim;
ylim([min(yl(1),-0.06)  max(yl(2),0.06)]);
styleThisFigure(gcf);

%% Yardımcı Fonksiyonlar
function y = f_discrete_NED_BGa(x,u,dt,g)
% x = [pN pE pD vN vE vD phi theta psi  bg(3)  ba(3)]
% u = [acc_meas_b(3); gyro_meas_b(3)]
p  = x(1:3); v = x(4:6); ph=x(7); th=x(8); ps=x(9);
bg = x(10:12);  ba = x(13:15);
fb_meas = u(1:3); wb_meas = u(4:6);

RNB = Rz(ps)*Ry(th)*Rx(ph);
gn  = [0;0;+g];

wb = wb_meas - bg;
fb = fb_meas - ba;

M = [ 1, sin(ph)*tan(th),  cos(ph)*tan(th);
      0, cos(ph),         -sin(ph);
      0, sin(ph)/cos(th),  cos(ph)/cos(th) ];

p_next   = p + v*dt;
v_next   = v + (RNB*fb + gn)*dt;
ang_next = [ph;th;ps] + M*wb*dt;
ang_next = wrapPi(ang_next.');

bg_next  = bg;
ba_next  = ba;

y = [p_next; v_next; ang_next.'; bg_next; ba_next];
end

function Fk = F_analytic_NED_BGa(x,u,dt)
ph=x(7); th=x(8); ps=x(9);
bg=x(10:12); ba=x(13:15);
fbm=u(1:3); p=u(4); q=u(5); r=u(6);

cph=cos(ph); sph=sin(ph); cth=cos(th); sth=sin(th);
cps=cos(ps); sps=sin(ps); tth=tan(th); sech=1/cth;

RNB = Rz(ps)*Ry(th)*Rx(ph);
fb  = fbm - ba;

dC_dphi = [ 0,  sps*sph + cps*sth*cph,  sps*cph - cps*sth*sph;
            0, -cps*sph + sps*sth*cph, -cps*cph - sps*sth*sph;
            0,  cth*cph,               -cth*sph ];
dC_dth  = [ -cps*sth,  cps*cth*sph,  cps*cth*cph;
            -sps*sth,  sps*cth*sph,  sps*cth*cph;
            -cth,      -sth*sph,     -sth*cph ];
dC_dps  = [ -sps*cth, -cps*cph - sps*sth*sph,  cps*sph - sps*sth*cph;
             cps*cth, -sps*cph + cps*sth*sph,  sps*sph + cps*sth*cph;
             0,        0,                       0 ];
Jv = [ dC_dphi*fb, dC_dth*fb, dC_dps*fb ];

M = [ 1, sin(ph)*tan(th),  cos(ph)*tan(th);
      0, cos(ph),         -sin(ph);
      0, sin(ph)/cos(th),  cos(ph)/cos(th) ];
dphdot_dph =  cph*tth*q - sph*tth*r;
dphdot_dth =  sph*sech^2*q + cph*sech^2*r;
dthdot_dph = -sph*q - cph*r;    dthdot_dth = 0;
dpsdot_dph =  sech*(cph*q - sph*r);
dpsdot_dth =  sech*tth*(sph*q + cph*r);
Jeta = [ dphdot_dph, dphdot_dth, 0;
         dthdot_dph, dthdot_dth, 0;
         dpsdot_dph, dpsdot_dth, 0 ];

Fk = eye(15);
Fk(1:3,4:6)   = dt*eye(3);
Fk(4:6,7:9)   = dt*Jv;
Fk(4:6,13:15) = -dt*RNB;
Fk(7:9,7:9)   = eye(3) + dt*Jeta;
Fk(7:9,10:12) = -dt*M;
end

function y = f_discrete_NED_BG(x,u,dt,g)
% x = [pN pE pD vN vE vD phi theta psi bp bq br]
% u = [acc_b(3); gyro_b(3)]
p  = x(1:3); v = x(4:6); ph=x(7); th=x(8); ps=x(9);
bb = x(10:12);
fb = u(1:3); wb_meas = u(4:6);

R_NB = Rz(ps)*Ry(th)*Rx(ph);
gn   = [0;0;+g];

wb = wb_meas - bb;

M = [ 1, sin(ph)*tan(th),  cos(ph)*tan(th);
      0, cos(ph),         -sin(ph);
      0, sin(ph)/cos(th),  cos(ph)/cos(th) ];
eul_dot = M*wb;

p_next   = p + v*dt;
v_next   = v + (R_NB*fb + gn)*dt;
ang_next = [ph;th;ps] + eul_dot*dt;
ang_next = wrapPi(ang_next.');

b_next = bb;
y = [p_next; v_next; ang_next.'; b_next];
end


function Fk = F_analytic_NED_BG(x,u,dt)
% F = ∂f/∂x (12x12), gyro bias etkisi dahil
ph=x(7); th=x(8); ps=x(9);
fb=u(1:3); p=u(4); q=u(5); r=u(6);
bb=x(10:12);

cph=cos(ph); sph=sin(ph); cth=cos(th); sth=sin(th);
cps=cos(ps); sps=sin(ps);
tth = tan(th); sech = 1/cth;

% dCnb/d(angles) for ZYX
dC_dphi = [ 0,  sps*sph + cps*sth*cph,  sps*cph - cps*sth*sph;
    0, -cps*sph + sps*sth*cph, -cps*cph - sps*sth*sph;
    0,  cth*cph,               -cth*sph ];
dC_dth  = [ -cps*sth,  cps*cth*sph,  cps*cth*cph;
    -sps*sth,  sps*cth*sph,  sps*cth*cph;
    -cth,      -sth*sph,     -sth*cph ];
dC_dps  = [ -sps*cth, -cps*cph - sps*sth*sph,  cps*sph - sps*sth*cph;
    cps*cth, -sps*cph + cps*sth*sph,  sps*sph + cps*sth*cph;
    0,        0,                       0 ];
Jv = [ dC_dphi*fb, dC_dth*fb, dC_dps*fb ];   % 3x3

% Euler hızı M(φ,θ)
M = [ 1, sin(ph)*tan(th),  cos(ph)*tan(th);
    0, cos(ph),         -sin(ph);
    0, sin(ph)/cos(th),  cos(ph)/cos(th) ];

% ∂(Mω)/∂angles (ω=wb_meas-b)
dphdot_dph =  cph*tth*q - sph*tth*r;
dphdot_dth =  sph*sech^2*q + cph*sech^2*r;
dthdot_dph = -sph*q - cph*r;    dthdot_dth = 0;
dpsdot_dph =  sech*(cph*q - sph*r);
dpsdot_dth =  sech*tth*(sph*q + cph*r);
Jeta = [ dphdot_dph, dphdot_dth, 0;
    dthdot_dph, dthdot_dth, 0;
    dpsdot_dph, dpsdot_dth, 0 ];

Fk = eye(12);
Fk(1:3,4:6) = dt*eye(3);        % ṗ = v
Fk(4:6,7:9) = dt*Jv;            % v̇ açıya bağlı
Fk(7:9,7:9) = eye(3) + dt*Jeta; % açı dinamiği açılara bağlı
Fk(7:9,10:12) = -dt*M;          % *** yeni: ∂(açı_next)/∂b = -M*dt
% Bias dinamiği: b_{k+1}=b_k (GM istersen: (1-dt/τ)*I)
end
%%
function y = f_discrete_NED(x,u,dt,g)
% x = [pN pE pD vN vE vD phi theta psi]
% u = [ivmeölçer(3); gyro(3)]
p  = x(1:3); v = x(4:6); ph=x(7); th=x(8); ps=x(9);
fb = u(1:3); wb = u(4:6);
R_NB = Rz(ps)*Ry(th)*Rx(ph);     % body->nav (NED)
gn   = [0;0;+g];                 % NED

% Euler hızları (ZYX)
M = [ 1, sin(ph)*tan(th),  cos(ph)*tan(th);
    0, cos(ph),         -sin(ph);
    0, sin(ph)/cos(th),  cos(ph)/cos(th) ];
eul_dot = M*wb;

p_next   = p + v*dt;
v_next   = v + (R_NB*fb + gn)*dt;
ang_next = [ph;th;ps] + eul_dot*dt;
ang_next = wrapPi(ang_next.');

y = [p_next; v_next; ang_next.'];
end

function Fk = F_analytic_NED(x,u,dt)
% F = ∂f/∂x (NED)
ph=x(7); th=x(8); ps=x(9);
fb=u(1:3); p=u(4); q=u(5); r=u(6);

cph=cos(ph); sph=sin(ph); cth=cos(th); sth=sin(th);
cps=cos(ps); sps=sin(ps);
tth = tan(th); sech = 1/cth;

% dCnb/d(angles) for ZYX
dC_dphi = [ 0,  sps*sph + cps*sth*cph,  sps*cph - cps*sth*sph;
    0, -cps*sph + sps*sth*cph, -cps*cph - sps*sth*sph;
    0,  cth*cph,               -cth*sph ];
dC_dth  = [ -cps*sth,  cps*cth*sph,  cps*cth*cph;
    -sps*sth,  sps*cth*sph,  sps*cth*cph;
    -cth,      -sth*sph,     -sth*cph ];
dC_dps  = [ -sps*cth, -cps*cph - sps*sth*sph,  cps*sph - sps*sth*cph;
    cps*cth, -sps*cph + cps*sth*sph,  sps*sph + cps*sth*cph;
    0,        0,                       0 ];
Jv = [ dC_dphi*fb, dC_dth*fb, dC_dps*fb ];   % 3x3

% ∂(Mω)/∂angles
dphdot_dph =  cph*tth*q - sph*tth*r;
dphdot_dth =  sph*sech^2*q + cph*sech^2*r;
dthdot_dph = -sph*q - cph*r;    dthdot_dth = 0;
dpsdot_dph =  sech*(cph*q - sph*r);
dpsdot_dth =  sech*tth*(sph*q + cph*r);
Jeta = [ dphdot_dph, dphdot_dth, 0;
    dthdot_dph, dthdot_dth, 0;
    dpsdot_dph, dpsdot_dth, 0 ];

Fk = eye(9);
Fk(1:3,4:6) = dt*eye(3);
Fk(4:6,7:9) = dt*Jv;
Fk(7:9,7:9) = eye(3) + dt*Jeta;
end
%%
function z = h_mag_vec_NED(x,mn)
% z = m_b = Cbn(q)*m_n, NED
ph=x(7); th=x(8); ps=x(9);
Cbn_ = (Rz(ps)*Ry(th)*Rx(ph)).';
z = Cbn_*mn;
end

function H = H_mag_analytic_NED(x,mn)
ph=x(7); th=x(8); ps=x(9);
Rz_=Rz(ps); Ry_=Ry(th); Rx_=Rx(ph);
dRx = dRx_dphi(ph); dRy = dRy_dtheta(th); dRz = dRz_dpsi(ps);

dCnb_dphi  = Rz_*Ry_*dRx;
dCbn_dphi  = dCnb_dphi.';

dCnb_dth   = Rz_*dRy*Rx_;
dCbn_dth   = dCnb_dth.';

dCnb_dpsi  = dRz*Ry_*Rx_;
dCbn_dpsi  = dCnb_dpsi.';

H = zeros(3,9);
H(:,7) = dCbn_dphi  * mn;
H(:,8) = dCbn_dth   * mn;
H(:,9) = dCbn_dpsi  * mn;
end


% --- rotasyon ve türevleri ---
function R = Rx(a), ca=cos(a); sa=sin(a); R=[1 0 0; 0 ca -sa; 0 sa ca]; end
function R = Ry(a), ca=cos(a); sa=sin(a); R=[ca 0 sa; 0 1 0; -sa 0 ca]; end
function R = Rz(a), ca=cos(a); sa=sin(a); R=[ca -sa 0; sa ca 0; 0 0 1]; end
function dR = dRx_dphi(phi), c=cos(phi); s=sin(phi); dR=[0 0 0; 0 -s -c; 0 c -s]; end
function dR = dRy_dtheta(th), c=cos(th); s=sin(th); dR=[-s 0 c; 0 1 0; -c 0 -s]; end
function dR = dRz_dpsi(ps), c=cos(ps); s=sin(ps); dR=[-s -c 0; c -s 0; 0 0 0]; end

function ang = wrapPi(ang)
ang = mod(ang + pi, 2*pi) - pi;
end



function Q = Q_from_IMU_noise_NED(x, dt, fs, sigAcc, sigGyro, tune_a, tune_g)
% x = [pN pE pD vN vE vD phi theta psi]
if nargin<6, tune_a=1; end
if nargin<7, tune_g=1; end
ph=x(7); th=x(8); ps=x(9);

Cnb = Rz(ps)*Ry(th)*Rx(ph);
T   = [ 1, sin(ph)*tan(th),  cos(ph)*tan(th);
    0, cos(ph),         -sin(ph);
    0, sin(ph)/cos(th),  cos(ph)/cos(th) ];

Sa = diag(sigAcc.^2)  * fs;   % PSD
Sg = diag(sigGyro.^2) * fs;

Ra = Cnb * Sa * Cnb.';        % nav (NED) ivme PSD

Qpp = (dt^3/3)*Ra;
Qpv = (dt^2/2)*Ra;
Qvv =  dt     *Ra;
Qeta = dt * (T * Sg * T.');

Q = zeros(9);
Q(1:3,1:3) = tune_a*Qpp;
Q(1:3,4:6) = tune_a*Qpv;   Q(4:6,1:3) = tune_a*Qpv.';
Q(4:6,4:6) = tune_a*Qvv;
Q(7:9,7:9) = tune_g*Qeta;
end



function Q = Q_from_IMU_noise_NED_15(x, dt, fs, sigAcc, sigGyro, qb_g_rw, qb_a_rw, ...
    tune_a, tune_g, tune_bg, tune_ba)
% 15-durum (Euler) süreç kovaryansı
% x  : [p v euler bg ba]
% dt : zaman adımı [s]
% fs : IMU örnekleme [Hz]
% sigAcc, sigGyro : tek-örnek std (body)  [m/s^2], [rad/s]
% qb_g_rw, qb_a_rw: bias random-walk şiddeti (süreç) [rad/s/sqrt(s)], [m/s^2/sqrt(s)]
% tune_* : (opsiyonel) ölçekler

if nargin<8,  tune_a = 1; end
if nargin<9,  tune_g = 1; end
if nargin<10, tune_bg = 1; end
if nargin<11, tune_ba = 1; end

% --- Euler açıları ---
ph = x(7); th = x(8); ps = x(9);

% --- IMU PSD (body çerçevesinde) ---
Sa = diag(sigAcc.^2)  * fs;   % accel PSD
Sg = diag(sigGyro.^2) * fs;   % gyro  PSD

% --- Body->Nav dönüşümü ve Euler hız matrisi M(φ,θ) ---
Cnb = Rz(ps) * Ry(th) * Rx(ph);
M   = [ 1, sin(ph)*tan(th),  cos(ph)*tan(th);
    0, cos(ph),         -sin(ph);
    0, sin(ph)/cos(th),  cos(ph)/cos(th) ];

% --- İvmeden gelen nav-çerçevesi PSD ---
Ra  = Cnb * Sa * Cnb.';          % NED

% --- Süreç kovaryans blokları ---
Qpp = (dt^3/3) * Ra;             % konum
Qpv = (dt^2/2) * Ra;
Qvv =  dt       * Ra;            % hız
Qtt =  dt * (M * Sg * M.');      % Euler açı gürültüsü
Qbg = (qb_g_rw^2) * dt * eye(3); % gyro bias RW
Qba = (qb_a_rw^2) * dt * eye(3); % accel bias RW

% --- 15x15 Q matrisini kur ---
Q = zeros(15);
Q(1:3,1:3)   = tune_a * Qpp;
Q(1:3,4:6)   = tune_a * Qpv;          Q(4:6,1:3) = (tune_a*Qpv).';
Q(4:6,4:6)   = tune_a * Qvv;

Q(7:9,7:9)   = tune_g * Qtt;

Q(10:12,10:12) = tune_bg * Qbg;       % gyro bias
Q(13:15,13:15) = tune_ba * Qba;       % accel bias
end


function Q = Q_simple_Euler15(dt, q_p, q_v, q_ang, q_bg_rw, q_ba_rw)
% x = [p v euler bg ba]  (Euler 15)
% q_p [m/s^1.5], q_v [m/s^2], q_ang [rad/s^0.5]
% q_bg_rw [rad/s/sqrt(s)], q_ba_rw [m/s^2/sqrt(s)]

Qpp = (dt^3/3) * q_v^2 * eye(3);   % p  (white-acc driven)
Qpv = (dt^2/2) * q_v^2 * eye(3);
Qvv =  dt      * q_v^2 * eye(3);

Qtt =  dt * q_ang^2 * eye(3);      % Euler

Qbg = dt * (q_bg_rw^2) * eye(3);   % gyro bias RW
Qba = dt * (q_ba_rw^2) * eye(3);   % accel bias RW

Q = zeros(15);
Q(1:3,1:3)=Qpp; Q(1:3,4:6)=Qpv; Q(4:6,1:3)=Qpv.'; Q(4:6,4:6)=Qvv;
Q(7:9,7:9)=Qtt;
Q(10:12,10:12)=Qbg;
Q(13:15,13:15)=Qba;
end


function styleThisFigure(fig)
% figür rengi (defaultFigureColor)
set(fig,'Color',[1 1 1]);

ax = findall(fig,'Type','axes');
if ~isempty(ax)
    set(ax, ...
        'Box','off', ...
        'Color',[1  1 1], ...                 % defaultAxesColor
        'XGrid','on','YGrid','on','ZGrid','on', ...    % defaultAxes*Grid
        'XMinorTick','off','YMinorTick','off','ZMinorTick','off', ...
        'XMinorGrid','on','YMinorGrid','on','ZMinorGrid','on', ...
        'GridColor',[1 1 1],'GridAlpha',1, ...        % defaultAxesGrid*
        'TickDir','out','LineWidth',0.75, ...         % defaultAxes*
        'FontName','Arial','FontSize',11, ...         % defaultAxesFont*
        'TitleFontWeight','bold','TitleFontSizeMultiplier',0.9); % title ayarları
end

% Figürdeki eksenlerde 1.siyah, 2.mavi, 3.kırmızı
axh = findall(fig,'Type','axes');
cizgiKal =1;
for ax = axh.'
    ln = findall(ax,'Type','line'); ln = flipud(ln);
    if numel(ln)>=1, set(ln(1),'Color','r','LineWidth',cizgiKal); end
    if numel(ln)>=2, set(ln(2),'Color','k','LineWidth',cizgiKal,'LineStyle','-'); end
    if numel(ln)>=3, set(ln(3),'Color','b','LineWidth',cizgiKal); end
    if numel(ln)>=4, set(ln(4),'Color','b','LineWidth',cizgiKal); end

end

% legend (defaultLegendBox)
lgd = findall(fig,'Type','legend');
if ~isempty(lgd)
    set(lgd,'Box','off','Fontsize',9);
end




end




