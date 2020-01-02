%% Balancing Robot
close all
clear all

% Define new group with HEBI modules
group = HebiLookup.newGroupFromNames('Robot',{'LinksAußen'; 'LinksInnene'; 'RechtsInnen'; 'RechtsAußen'}); 

cmd = CommandStruct();

% Mecanum Wheeled Robot Parameters
[Rw,l1,l2,alpha] = MecanumPendulum();

% PID Controller Gains
Kp = 3.5; % P element
Ki = 1.5; % I element
Kd = 0.06; % D element

% Declare error variables
esum = 0; % starting value for cumulative error
e = 0; % current error
eold = 0; % previous error

target = 0; % theta_p = 0

% inverse kinematic mapping
M = 1/Rw * [cot(alpha(1)) 1 l2 -Rw;...
    cot(alpha(2)) 1 l1 -Rw;...
    cot(alpha(3)) 1 -l1 -Rw;...
    cot(alpha(4)) 1 -l2 -Rw];

% Time constants for complementary filter
tau = 0.5; %[s]
Ts = 1/group.getFeedbackFrequency; %[s], default feedback frequency is 100 Hz 
alpha_c = tau/(tau+Ts);
% initial phi
phi = 0;

t_old = group.getNextFeedback.time; % module time at start

count = 1;

while 1
    fbk = group.getNextFeedback(); % get new feedback
    t = fbk.time; % get time form feedback struct
    dt = t-t_old; % compute time from last to current feedback
    t_old = t; % save current time
    
    % calculate theta_p with a complementary filter
    phi = alpha_c*(phi + dt*fbk.gyroZ) + (1-alpha_c)*atan2(fbk.accelY,fbk.accelX); % Complementary filter (evtl. -fbk.gyroZ)
    
    e = target - phi;
    esum = esum + e; %esum = esum + e*dt;
    y = Kp * e + Ki * dt * esum + Kd * (e - eold)/dt; %y = Kp * e + Ki * esum + Kd * (e - eold)/dt;
    eold = e;
    
    q_dot = [0; 0; 0; y/dt]; % robot only balancing y/dt -> velocity
    
    % calculate wheel velocities
    theta_dot = M * q_dot;
    
    count = count + 1;
    
    % Hebi updaten
    cmd.velocity = theta_dot';
    group.send(cmd);
end

