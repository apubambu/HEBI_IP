3%% Balancing Robot
close all
clear all

% Define new group with HEBI modules
group = HebiLookup.newGroupFromNames('Robot',{'1'; '2'; '3'; '4'}); 
% Define new group with mobile IO app
%mobile = HebiLookup.newGroupFromNames('Projekt','MobileJohannes');

cmd = CommandStruct();

% Mecanum Wheeled Robot Parameters
[Rw,l1,l2,alpha,lv] = MecanumPendulum();

% PID Controller Gains
Kp = 1.2; % P element
Ki = 0.3; % I element
Kd = 0; % D element

% PID Controller Gains velocity
Kp_vx = 0.1; % P element
Ki_vx = 0.01; % I elements
Kd_vx = 0; % D element

% Maximum velocities
v_abs = 0.5;
omega_abs = 0.05;

% Declare error variables
esum = 0; % starting value for cumulative error
e = 0; % current error
eold = 0; % previous error

esum_vx = 0; % starting value for cumulative error
e_vx = 0; % current error
eold_vx = 0; % previous error

target = 0; % theta_p = 0 bzw. dtheta_p

% starting configuration
q = [0; 0; 0; 0];
q_log = [];
q_meas_log = [];

% inverse kinematic mapping 
% !! evtl. komplette 2. und 4. Zeile *-1 da Module entgegengesetzt orientiert
M = 1/Rw * [cot(alpha(1)) 1 l2 -Rw;... 
    -cot(alpha(2)) -1 -l1 Rw;...
    cot(alpha(3)) 1 -l1 -Rw;...
    -cot(alpha(4)) -1 l2 Rw];
M_pinv = pinv(M);

% Time constants for complementary filter
% !! tau wsl zu groß -> alpha_c zu groß (für Testfall anpassen)
tau = 0.04; %0.5; %[s]
Ts = 1/group.getFeedbackFrequency; %[s], default feedback frequency is 100 Hz 
alpha_c = tau/(tau+Ts);
% initial phi
phi = 0;
dphi = 0;

% module time at start
t_old = group.getNextFeedback.time; 

%fig = figure;

while 1
    fbk = group.getNextFeedback(); % get new feedback
    t = fbk.time; % get time form feedback struct
    dt = t-t_old; % compute time from last to current feedback
    t_old = t; % save current time
    
    % Read app inputs: 
    % A8 defines velocity in x, A7 velocity y direction,
    % A1 angular velocity in z, B1 stops program
    %fbkIO = mobile.getNextFeedbackIO();
    v_WOx = 0;%sum(v_abs*fbkIO.a8,'omitnan');
    v_WOy = 0;%sum(v_abs*fbkIO.a7,'omitnan');
    omega_WO = 0;%sum(-omega_abs*fbkIO.a1,'omitnan');
    
    % calculate dtheta_p with a complementary filter
    % !! können wir drehrate nich direkt mit Gyro messen? Komplementärfilter war ja nur nötig, da wir Gyrosignal intgrieren und dadurch ein Drift entsteht
    % !! mean(...) wsl falsch da z-Achsen der Module unterschiedlich orientiert sind -> es müsste ca. 0 rauskommen
    dphi = fbk.gyroZ(1);
    %dphi = alpha_c*(dphi + fbk.gyroZ(1)) + (1-alpha_c)*atan2(fbk.accelY(1),fbk.accelX(1)/dt);
    % !! vgl. VergleichCompFilter.fig vorheriger Filter war richtig, neuer berücksichtigt zum Teil die Winkelgeschwindigkeit und zum anderen den Winkel.
    phi = alpha_c*(phi + fbk.gyroZ(1)) + (1-alpha_c)*atan2(fbk.accelY(1),fbk.accelX(1)); 
    %phi = alpha_c*(phi + dt*fbk.gyroZ(1)) + (1-alpha_c)*atan2(fbk.accelY(1),fbk.accelX(1));
    
    % stop programm if phi > 15 degree
    if phi > deg2rad(15)
        break
    end
    
    %% PID cascade control
    %vx_meas = Rw * (fbk.velocity(1)-fbk.velocity(2)+fbk.velocity(3)-fbk.velocity(4))/4;
    % !! vx aus Drehrate berechnet
    vx_meas  = (lv + Rw) *  dphi; 
    % PID controller x_dot
    e_vx = v_WOx - vx_meas;
    esum_vx = esum_vx + e_vx; 
    y_vx = Kp_vx * e_vx + Ki_vx * dt * esum_vx + Kd_vx * (e_vx - eold_vx)/dt;
    eold_vx = e_vx;
    
    target = y_vx;
    % PID controller theta_p
    e = target - phi;
    esum = esum + e;
    y = Kp * e + Ki * dt * esum + Kd * (e - eold)/dt;
    eold = e;
    
    % robot balancing and following app input
    q_dot = [0; 0; 0; y] + [0; v_WOy; omega_WO; 0];
    q = q + q_dot*dt;
    q_log = [q_log q];
    
     % calculate transformation matrix robot body to world coordinate system
    T_WO = [cos(q(3)) sin(q(3)) 0 0; -sin(q(3)) cos(q(3)) 0 0; 0 0 1 0; 0 0 0 1];
    
    % calculate wheel velocities
    theta_dot = M * T_WO * q_dot;
    
    % Hebi updaten
    cmd.velocity = theta_dot';
    group.send(cmd);
    
    % !! pinv(M) hier wsl problematisch, evtl nur plotten der simulierten position
    q_meas = inv(T_WO) * M_pinv * fbk.velocity'; 
    q_meas_log = [q_meas_log q_meas];
    
    % plot configuration
%     figure(fig); clf; axis equal; hold on;
%     plot(q_meas_log(2,1),q_meas_log(1,1),'bo');
%     plot(q_meas_log(2,:),q_meas_log(1,:));
%     plot(q_meas_log(2,end),q_meas_log(1,end),'kx');
%     veh = [cos(q_meas_log(3,end)) -sin(q_meas_log(3,end)); ...
%         sin(q_meas_log(3,end)) cos(q_meas_log(3,end))]*[-2 -2 2 2; -1 1 1 -1]...
%         +[q_meas_log(2,end); q_meas_log(1,end)];
%     plot(veh(1,:),veh(2,:));
%     drawnow; hold off;
end

