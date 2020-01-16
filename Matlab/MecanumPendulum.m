function [Rw,l1,l2,alpha,lv] = MecanumPendulum()
% wheel offset
<<<<<<< HEAD
l1 = 0.025;%0.06;
l2 = 0.20;%0.15;
% wheel radius
Rw = 0.045;%0.04;
% distance wheel axis to velocity controlled ponit on the robot (estimation 10 cm)
lv = 0.10;
% wheel roller angle
alpha = [-45 45 -45 45]*pi/180;
=======
l1 = 0.025;
l2 = 0.2;
% wheel radius
Rw = 0.045;
% wheel roller angle
alpha = [45 -45 45 -45]*pi/180;
>>>>>>> b26b82c90ae1d1a37e7abeb8902a8b5786f8a021
end

