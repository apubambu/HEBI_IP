%% Balancing Robot
close all
clear all

% Define new group with HEBI modules
group = HebiLookup.newGroupFromNames('Robot',{'LinksAußen'; 'LinksInnene'; 'RechtsInnen'; 'RechtsAußen'}); 
% Define new group with mobile IO app
mobile = HebiLookup.newGroupFromNames('Projekt','MobileJohannes');

cmd = CommandStruct();

% Mecanum Wheeled Robot Parameters
[Rw,l1,l2,alpha] = MecanumPendulum();

% PID Controller Gains
Kp = 3.5; % P element
Ki = 1.5; % I element
Kd = 0.06; % D element

% Maximum velocities
v_abs = 0.5;
omega_abs = 0.05;

% Declare error variables
esum = 0; % starting value for cumulative error
e = 0; % current error
eold = 0; % previous error

target = 0; % theta_p = 0

% starting configuration
q = [0; 0; 0; 0];

% inverse kinematic mapping
M = 1/Rw * [cot(alpha(1)) 1 l2 -Rw;...
    cot(alpha(2)) 1 l1 -Rw;...
    cot(alpha(3)) 1 -l1 -Rw;...
    cot(alpha(4)) 1 -l2 -Rw];
M_pinv = pinv(M);

% Time constants for complementary filter
tau = 0.5; %[s]
Ts = 1/group.getFeedbackFrequency; %[s], default feedback frequency is 100 Hz 
alpha_c = tau/(tau+Ts);
% initial phi
phi = 0;

% module time at start
t_old = group.getNextFeedback.time; 

fig = figure;

while 1
    fbk = group.getNextFeedback(); % get new feedback
    t = fbk.time; % get time form feedback struct
    dt = t-t_old; % compute time from last to current feedback
    t_old = t; % save current time
    
    % Read app inputs: 
    % A8 defines velocity in x, A7 velocity y direction,
    % A1 angular velocity in z, B1 stops program
    fbkIO = mobile.getNextFeedbackIO();
    v_WOx = sum(v_abs*fbkIO.a8,'omitnan');
    v_WOy = sum(v_abs*fbkIO.a7,'omitnan');
    omega_WO = sum(-omega_abs*fbkIO.a1,'omitnan');
    
    % calculate theta_p with a complementary filter
    phi = alpha_c*(phi + dt*fbk.gyroZ) + (1-alpha_c)*atan2(fbk.accelY,fbk.accelX); % Complementary filter (evtl. -fbk.gyroZ)
    
    e = target - phi;
    esum = esum + e; %esum = esum + e*dt;
    y = Kp * e + Ki * dt * esum + Kd * (e - eold)/dt; %y = Kp * e + Ki * esum + Kd * (e - eold)/dt;
    eold = e;
    
    % calculate transformation matrix robot body to world coordinate system
    T_WO = [cos(q(3)) sin(q(3)) 0 0; -sin(q(3)) cos(q(3)) 0 0; 0 0 1 0; 0 0 0 1];
    
    % robot balancing and following app input
    q_dot = [0; 0; 0; y/dt] + [v_WOx; v_WOy; omega_WO; 0];
    q = q + q_dot*dt;
    q_log = [q_log q];
    
    % calculate wheel velocities
    theta_dot = M * T_WO * q_dot;
    
    % Hebi updaten
    cmd.velocity = theta_dot';
    group.send(cmd);
    
    % plot configuration
    figure(h_q); clf; axis equal; hold on;
    plot(q_log(2,1),q_log(1,1),'bo');
    plot(q_log(2,:),q_log(1,:));
    plot(q_log(2,end),q_log(1,end),'kx');
    veh = [cos(q_log(3,end)) -sin(q_log(3,end)); ...
        sin(q_log(3,end)) cos(q_log(3,end))]*[-2 -2 2 2; -1 1 1 -1]...
        +[q_log(2,end); q_log(1,end)];
    plot(veh(1,:),veh(2,:));
    drawnow; hold off;
end

