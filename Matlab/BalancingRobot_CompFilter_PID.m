%% Balancing Robot
close all
clear all

% Define new group with HEBI modules
group = HebiLookup.newGroupFromNames('Robot',{'1'; '2'; '3'; '4'}); 

% Define new group with mobile IO app
cmd = CommandStruct();

%% PARAMS and KINEMATICS
% Mecanum Wheeled Robot Parameters
[Rw,l1,l2,alpha] = MecanumPendulum();

% inverse kinematic mapping
M = 1/Rw * [cot(alpha(1)) 1 l2 -Rw;...
    cot(alpha(2)) 1 l1 Rw;...
    cot(alpha(3)) 1 -l1 -Rw;...
    cot(alpha(4)) 1 -l2 Rw];

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
Kp      = 0.8; %1.1
Ki      = 15; %0.9
Kd      = 0;
%SetPoint
SetPoint = 0;

%% OUTER LOOP 
Kp_theta_phi    = 0.005;
Ki_theta_phi    = 0.008
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


while 1
    
    fbk     = group.getNextFeedback();  % get new feedback
    t       = fbk.time;                 % get time form feedback struct
    dt      = t-t_old;                  % compute time from last to current feedback
    t_old   = t;                        % save current time

    %% MEASUREMENT VARIABLES
    % phi
    phi         = alpha_c*(phi + fbk.gyroZ(1) * dt) + (1 - alpha_c) * atan(-fbk.accelY(1)/ fbk.accelX(1)) % Complementary filter (evtl. -fbk.gyroZ)
    phi_dot     = fbk.gyroZ(1);
    theta_dot   = -mean([-fbk.velocity(1), fbk.velocity(2), -fbk.velocity(3), fbk.velocity(4)]) ;
    
    %% OUTER LOOP
    e_ol = SetPoint_Theta - theta_dot;
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
    q_dot     = [ 0; 0; 0; y/dt];
    % calculate wheel velocities
    theta_dot = M * q_dot;

    % Hebi updaten
    cmd.velocity = theta_dot';
    group.send(cmd);
    
    %% LOGS
    dt_log = [dt_log dt];
end

