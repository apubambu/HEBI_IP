%% Balancing Robot
close all
clear all

% Define new group with HEBI modules
group = HebiLookup.newGroupFromNames('Robot',{'1'; '2'; '3'; '4'}); 
mobile = HebiLookup.newGroupFromNames('Projekt','MobileJohannes');
% Define new group with mobile IO app
cmd = CommandStruct();

%% PARAMS and KINEMATICS
% Mecanum Wheeled Robot Parameters
[Rw,l1,l2,alpha,lv] = MecanumPendulum();

% inverse kinematic mapping
M = 1/Rw * [cot(alpha(1)) 1 l2 -Rw;...
    -cot(alpha(2)) -1 -l1 Rw;...
    cot(alpha(3)) 1 -l1 -Rw;...
    -cot(alpha(4)) -1 l2 Rw];

v_absx      = 0.2;
v_absy      = 2;
omega_abs   = 1;
%% COMPLEMENTARY FILTER
% Time constants for complementary filter
tau     = 0.5; %[s]
Ts      = 1/group.getFeedbackFrequency; %[s], default feedback frequency is 100 Hz 
alpha_c = tau/(tau+Ts);

%% INNER LOOP
% Declare error variables
esum    = 0; % starting value for cumulative error
e       = 0; % current error
eold    = 0; % previous error
% Gains
Kp      = 0.8; %0.8
Ki      = 15;  %15
Kd      = 0;
%SetPoint
SetPoint = 0;

%% OUTER LOOP 
Kp_theta_phi    = 0.005; %0.005
Ki_theta_phi    = 0.008; %0.008
SetPoint_Theta  = 0;
e_ol            = 0;
esum_ol         = 0;
%


%% INITIAL VALUES
t_old   = group.getNextFeedback.time; % module time at start
count   = 1;
phi     = 0;
phi_dot = 0;
dt_log  = [];
q       = [0; 0; 0; 0];



while 1
    
    fbk     = group.getNextFeedback();  % get new feedback
    t       = fbk.time;                 % get time form feedback struct
    dt      = t-t_old;                  % compute time from last to current feedback
    t_old   = t;                        % save current time

    %% READ INPUTS
    % A8 defines velocity in x, A7 velocity y direction,
    % A1 angular velocity in z, B1 stops program
    fbkIO       = mobile.getNextFeedbackIO();
    v_WOx       = sum(v_absx*fbkIO.a8,'omitnan')
    v_WOy       = sum(v_absy*fbkIO.a7,'omitnan')
    omega_WO    = sum(-omega_abs*fbkIO.a1,'omitnan');
    
    %% MEASUREMENT VARIABLES
    % phi
    phi         = alpha_c*(phi + fbk.gyroZ(1) * dt) + (1 - alpha_c) * atan(-fbk.accelY(1)/ fbk.accelX(1)); % Complementary filter (evtl. -fbk.gyroZ)
    phi_dot     = fbk.gyroZ(1);
    theta_dot   = -mean([-fbk.velocity(1), fbk.velocity(2), -fbk.velocity(3), fbk.velocity(4)]) ;
    
    %% OUTER LOOP
    SetPoint_Theta = v_WOy;
    e_ol = SetPoint_Theta - theta_dot...
           + (lv + Rw) * phi_dot;
    esum_ol = esum_ol + e_ol;
    SetPoint_ = Kp_theta_phi * (e_ol) + ...
                Ki_theta_phi * dt *(esum_ol);
    
    %% INNER LOOP CONTROL
    e       = SetPoint_  - phi;
    esum    = esum + e; 
    y       = Kp * e +...
              Ki * dt * esum +...
              Kd * (e - eold)/dt;
    eold    = e;
    
    %% OUTPUT
    q_dot     = [ 0; 0; 0; y/dt] + [v_WOx; 0; omega_WO; 0];
    % calculate wheel velocities
    theta_dot = M * q_dot;

    % Hebi updaten
    cmd.velocity = theta_dot';
    if abs(phi)>(deg2rad(30))
        break;
    end
    group.send(cmd);
    %% LOGS
    dt_log = [dt_log dt];
end

