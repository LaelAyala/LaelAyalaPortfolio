% Script shell for Lab 5 on vision and manipulation.
clear all;
rosshutdown;
rosinit;
% (The below assumes that the environment is already set up with the armCmd publisher to the arm controller that you've been using since lab 0)

% Set this variable to 0 while you're working on the program outside the lab, and change it to 1 when you're on the real robot:
InLab = 1;

%%First we need to set-up a intitializer for robot and armcmd. Kade being
%%an absolute genius used an if else statment that would senf to gazebo if
%%true robot if false . so thats what we gon do

if InLab == 0
    armCmd = rospublisher('/eff_joint_traj_controller/command');
    possub= rossubscriber('/gazebo/link_states');
    
    
else
    
    robotCmd = rospublisher('scaled_pos_joint_traj_controller/command');
    possub= rossubscriber('tf');
    
end


% (x,y) locations in robot coordinates of the top left and bottom right of the corresponding marker blocks:
topcorner = [340,-400];
bottomcorner = [715,425];


% Set up a publisher to send the gripper commands:
gripperpub = rospublisher('/gripper_service_es159','std_msgs/Float32MultiArray');
gripperMsg = rosmessage('std_msgs/Float32MultiArray');

% Create a message that corresponds to moving the arm out of the way of the camera field of view:
if InLab ==1 
   OriginalMsg = rosmessage(robotCmd); 
else
  OriginalMsg = rosmessage(armCmd);
end

OriginalMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
originalpoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
originalpoint.Positions = [0 -pi/2 0 0 0 0];
originalpoint.Velocities = [0 0 0 0 0 0];
originalpoint.TimeFromStart.Sec = 5;
OriginalMsg.Points =[originalpoint];


% A position subscriber, like we used in lab 1, can be helpful in letting us make sure a movement command has finished before doing the next thing:


% Tell the arm to move out of the way of the camera, and wait until it's finished moving to take a snapshot:
if InLab == 0
send(armCmd, OriginalMsg);
else
    send(robotCmd, OriginalMsg);
end 
pause(5)


%{

while ismoving(possub,InLab)
  % do nothing
end
%}




% If you're in the physical lab, set up a subscriber to the physical camera, and take a snapshot; otherwise, load the sample snapshot from the file we gave you:
if InLab == 1
  CamSub = rossubscriber('/usb_cam/image_raw');
  CamMsg = receive(CamSub);
else
  load Lab5snapshot.mat
end

% Now do the image processing described in the Lab 5 handout:

% First separate out the three color channels from the .Data field of CamMsg:

%Red

Width = CamMsg.Width;
Height = CamMsg.Height;

Red = zeros(Height,Width);
Blue = zeros(Height,Width);
Green = zeros(Height,Width);

Red_v = CamMsg.Data(1:3:end)';
Blue_v = CamMsg.Data(3:3:end)';
Green_v = CamMsg.Data(2:3:end)';



for h =1:Height
            

            Red(h,:) = Red_v(1:Width);
            Red_v(1:Width) =[];

            Blue(h,:) = Blue_v(1:Width);
            Blue_v(1:Width) =[];

            Green(h,:) = Green_v(1:Width);
            Green_v(1:Width) =[];            
        
end % height



% As you work through writing and debugging this script, you can display your various image matrices, e.g.
%imagesc(Red); colormap(gray);
% N.B. As described in the handout, if you display image matrices directly as in the above line, they'll appear sideways and mirrored; if you want them to match the snapshot you expect to see from the camera, you'll need to send something to the imagesc command that isn't exactly the raw matrix...

%NEED TO RESHAPE TO MAKE NIT DIMENSION OF HEIGHT AND FLIP AND MIRROR IT. 


% Crop the images:


%Red = Red(151:400,51:550);
Red = Red(151:400,51:550);
Green = Green(151:400,51:550);
Blue = Blue(151:400,51:550);

%reinitializing height and width

Height = height(Red);
Width = length(Red);



% Threshold the images to get binary-valued matrices indicating where in the image each color channel is bright:
RedThresh = zeros(Height,Width);
GreenThresh = zeros(Height,Width);
BlueThresh = zeros(Height,Width);

if InLab == 1 
    t_low = 125;
t_blue = 170;
t_red = 171;
t_green = 130;
t_med = 180;
else 
t_low = 125;
t_blue = 170;
t_red = 171;
t_green = 150;
t_med = 180;
end 

for i = 1:height(Red)
   for j = 1:length(Red)

        if Red(i,j) >= t_red && Blue(i,j) < t_blue && Green(i,j) < t_green
            RedThresh(i,j) = 1;
        else 
            RedThresh(i,j) = 0;
        end % end red

        if Red(i,j) < t_red && Blue(i,j) >= t_blue && Green(i,j) < t_green 
            BlueThresh(i,j) = 1;
        else 
            BlueThresh(i,j) = 0;
        end %end blue

        if Red(i,j) < t_red && Blue(i,j) < t_blue && Green(i,j) >= t_green 
            GreenThresh(i,j) = 1;
        else 
            GreenThresh(i,j) = 0;
        end %end green


         
   end %end length
end %end height



% Make binary-valued matrices that show where in the image each color appears (by itself, not as a component of a composite color):
RedOnly = zeros(Height,Width);
GreenOnly = zeros(Height,Width);
BlueOnly = zeros(Height,Width);



%blue pix 132 156  ----> real world 517 -170 
%--> What imn getting [139,158], real world [ 301,363]
%green pix 108, 306 ----> 459 162. 
%what im getting in pixels [122.313] , real world [264,719]




for i = 1:height(Red)
   for j = 1:length(Red)

        if Red(i,j) >= t_red && Blue(i,j) < t_low && Green(i,j) < t_low
            RedOnly(i,j) = 1;
        else 
            RedOnly(i,j) = 0;
        end % end red

        if Red(i,j) < t_low && Blue(i,j) >= t_blue && Green(i,j) < t_blue
            BlueOnly(i,j) = 1;
        else 
            BlueOnly(i,j) = 0;
        end %end blue

        if Red(i,j) < t_low  && Blue(i,j) < t_low && Green(i,j) >= t_green
            GreenOnly(i,j) = 1;
        else 
            GreenOnly(i,j) = 0;
        end %end green


         
   end %end length
end %end height




% Find the centers of the (blue) block and (green) platform:
BlockPosInPixels = [0,0];




for i = 2:height(BlueOnly)-1
    for j = 2:length(BlueOnly)
        if BlueOnly(i,j) == 1
            %if number of ones on the top row == number of ones on bottom
            %row
            if sum(BlueOnly(i-1,:)==1) == sum(BlueOnly(i+1,:)==1)
                if sum(BlueOnly(:, j-1)==1) == sum(BlueOnly(:, j+1))
                    BlockPosInPixels = [i,j-1];
                end %end finding if columns align 
            end %end finding if rows align 
        end% ends 1
    end% ends column
end %end row

BlockPosInPixels
PlatformPosInPixels = [0,0];

for i = 2:height(GreenOnly)
    for j = 2:length(GreenOnly)
        if GreenOnly(i,j) == 1
            %if number of ones on the top row == number of ones on bottom
            %row
            if sum(GreenOnly(i-1,:)==1) == sum(GreenOnly(i+1,:)==1)
                if sum(GreenOnly(:, j-1)==1) == sum(GreenOnly(:, j+1))
                    PlatformPosInPixels = [i,j-1];
                end %end finding if columns align 
            end %end finding if rows align 
        end% ends 1
    end% ends column
end %end row

PlatformPosInPixels
% Convert from pixels to robot coordinates, using the 'topcorner' and 'bottomcorner' variables:

tcPix = tcPixel(RedOnly) %topcorner in pixels
bcPix = bcPixel(RedOnly) %bottomcorner in pixels

%next find the distance in-between top and bottom corners in pixels 
xPix_dist= bcPix(1,1)-tcPix(1,1);
yPix_dist= bcPix(1,2)-tcPix(1,2);


%next find the distance in between top and bottom x and y in robot position
x_dist = bottomcorner(1,1)-topcorner(1,1);
y_dist = bottomcorner(1,2)- topcorner(1,2);

% finally find the scalar converter that coverts pixels to robot position 
xc = x_dist/xPix_dist;
yc = y_dist/yPix_dist;

%blue pix 132 156  ----> real world 517 -170 
%--> What imn getting [139,158], real world [ 526,-207]
%green pix 108, 306 ----> 459 162. 
%what im getting in pixels [122.313] , real world [489,148]

Block_x = (BlockPosInPixels(1,1)*xc) +225+50;
Block_y= (BlockPosInPixels(1,2) * yc)-517-120;
Platform_x = (PlatformPosInPixels(1,1)*xc) +225 +50;
Platform_y = (PlatformPosInPixels(1,2) * yc) -631;




BlockPos = [Block_x,Block_y]
PlatformPos = [Platform_x,Platform_y]

imagesc(BlueOnly)
colormap(gray);
figure

imagesc(GreenOnly)
colormap(gray);
figure


% Now we're up to section 4 of the pre-lab in the lab handout: Commanding the Robot

% Some useful values:
hoverz = 100; % Height at which arm will not hit objects
targetblockz = -130; % Height at which arm will grasp objects
platformz = -130; % Height at which arm and carried object will clear platform
closedgripper = 255; % Value to send the gripper if you want it to close fully
opengripper = 0; % Value to send the gripper if you want it to open fully
blockgripper = 147; % Value to send the gripper if you want it to close on the block
gripperspeed = 55; % Use this value for speed
gripperforce = 55; % Use this value for force

% Move the end-effector to the initialization position given:
Tinitial = [-1 0 0 665; 0 1 0 0; 0 0 -1 100; 0 0 0 1]; % from Lab 4
Rinitial = Tinitial(1:3,1:3);
guessinitial = [-10 -60 90 -120 -90 80]*(pi/180);

Tinitial(1:3,4)'

A = [BlockPos, hoverz,6]

%{
waypoints = [Tinitial(1:3,4)',10; 
    BlockPos,hoverz,15;
    BlockPos,targetblockz, 20;
    BlockPos,hoverz,25;
    PlatformPos, hoverz, 30; 
    PlatformPos, platformz, 35; 
    PlatformPos,hoverz, 40;
     Tinitial(1:3,4)',45];   

%}

start = [665,0];


%Isabel helped me with this 
points_start_blue = [Tinitial(1:3,4)',1;
    start + (BlockPos - start) * 1/3,hoverz,2;
    start + (BlockPos - start) * 2/3, hoverz,3;
    BlockPos, hoverz, 4;
    BlockPos, targetblockz,5];

points_blue_green = [
                     BlockPos,hoverz,2;
                     BlockPos + (PlatformPos - BlockPos) * 1/4,hoverz,3;
                     BlockPos + (PlatformPos - BlockPos) * 2/4, hoverz,4;
                     BlockPos + (PlatformPos - BlockPos) * 3/4, hoverz,5;
                     PlatformPos,hoverz,6;
                     PlatformPos,platformz,7];
points_green_start = [PlatformPos,platformz,1;
                     PlatformPos,hoverz,2;
                     PlatformPos + (start - PlatformPos) * 1/4,hoverz,2;
                     PlatformPos + (start - PlatformPos) * 2/4, hoverz,4;
                     PlatformPos + (start - PlatformPos) * 3/4, hoverz,5;
                     start,hoverz,30];
                     
                     
    

% Open the gripper:
gripperMsg.Data = [opengripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5) % wait for gripper to finish moving


[commandlist,theta] = Lael_IVKshell(points_start_blue(1:4,:),guessinitial);

OriginalMsg.Points = commandlist;
theta =theta;



if InLab == 0
    send(armCmd,OriginalMsg)
else 
    send(robotCmd,OriginalMsg)
end
pause(5)

% Open the gripper:
gripperMsg.Data = [opengripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5) % wait for gripper to finish moving

% Lower straight down to z=targetblockz:
% **Code is missing here ... use your path-planning code from Lab 4 to move the gripper along a vertical trajectory**
[commandlist,theta] = Lael_IVKshell(points_start_blue(4:5,:),theta);

OriginalMsg.Points = commandlist;
theta =theta;

if InLab == 0
    send(armCmd,OriginalMsg)
else 
    send(robotCmd,OriginalMsg)
end
pause(5)

% Close the gripper on the block:
gripperMsg.Data = [blockgripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5) % wait for gripper to finish moving

%Raise the block up 
[commandlist,theta] = Lael_IVKshell(points_blue_green(1:5,:),theta);

OriginalMsg.Points = commandlist;
theta =theta;

if InLab == 0
    send(armCmd,OriginalMsg)
else 
    send(robotCmd,OriginalMsg)
end
pause(10)

%move  gripper hovering over platform
[commandlist,theta] = Lael_IVKshell(points_blue_green(5:6,:),theta);

OriginalMsg.Points = commandlist;
theta =theta;

if InLab == 0
    send(armCmd,OriginalMsg)
else 
    send(robotCmd,OriginalMsg)
end
pause(10)




% Open the gripper:
gripperMsg.Data = [opengripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5) % wait for gripper to finish moving

% Raise straight up to z=hoverz:
% **Code is missing here ... use your path-planning code from Lab 4 to move the gripper along a vertical trajectory**
%send(armCmd,TrajMsg);
[commandlist,theta] = Lael_IVKshell(points_green_start(1:6,:),theta);

OriginalMsg.Points = commandlist;
theta =theta;

if InLab == 0
    send(armCmd,OriginalMsg)
else 
    send(robotCmd,OriginalMsg)
end
pause(5)


% Return the robot arm to the original position:
OriginalMsg.Points =[originalpoint];

if InLab == 0
    send(armCmd,OriginalMsg)
else 
    send(robotCmd,OriginalMsg)
end

pause(5)

% Close the gripper:
gripperMsg.Data = [closedgripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5) % wait for gripper to finish moving



function theta = Lael_IKshell(T,theta0)
% Numerical calculation of UR5e inverse kinematics for end-effector position described by the transformation matrix T, starting from initial guess theta0 for the angles.

% First just make sure theta0 is a column vector, and exit if not:
if size(theta0,1)==1, theta0 = theta0'; elseif size(theta0,2)~=1, disp('Initial guess needs to be a 1D vector'); return, end


   


% Repeating the arm geometry from the FK lab; all values in mm:
W2 = 259.6;
W1 = 133.3;
H2 = 99.7;
H1 = 162.5;
L1 = 425;
L2 = 392.2;

% Screw axes in the world frame:
% First find the axis of rotation for each joint: using values from lab1
w1 = [0;0;1;];
w2 = [0;1;0];
w3 = [0;1;0];
w4 = [0;1;0];
w5 = [0;0;-1];
w6 = [0;1;0];
% Choose a point along each axis:
q1 = [0;0;0];
q2 = [0;W1;H1];
q3 = [L1;W1;H1];
q4 = [L1+L2; W1; H1];
q5 = [L1+L2; W1; H1];
q6 = [L1+L2; W1+W2; H1-H2];
% That lets us calculate the associated v:
v1 = cross(-w1,q1);
v2 = cross(-w2,q2);
v3 = cross(-w3,q3);
v4 = cross(-w4,q4);
v5 = cross(-w5,q5);
v6 = cross(-w6,q6);
% And that gives the screw axes:
S1 = [w1 ; v1];
S2 = [w2 ; v2];
S3 = [w3 ; v3];
S4 = [w4 ; v4];
S5 = [w5 ; v5];
S6 = [w6 ; v6];

% By inspection, we can get the matrix M relating the world and body frames at the home position:
M = [-1,0,0, L1+L2;
     0,0,1, W1+W2;
     0,1,0,H1-H2;
     0,0,0,1;];

 
 
d = [0,-(H1-H2), W1+W2;
     H1-H2, 0,-L1-L2;
     -W1-W2,L1+L2, 0];

R= [-1,0,0;
    0,0,1;
    0,1,0];

%dR = d*R; 


dR = [0, W1 + W2, H2 - H1; 
     H2 - H1, - L1 - L2, 0;
     W1 + W2, 0 , L1 + L2];
% Using M and the S vectors, we can find the body screw axes B: S_i = [Ad_M] B_i.
% The first step in that is calculating the adjoint map associated with M, [Ad_M]:
Ad_M = [-1,0,0,0,0,0;
        0,0,1,0,0,0;
        0,1,0,0,0,0;
        0, W1 + W2, H2 - H1 -1,0,0;
        H2 - H1, - L1 - L2, 0, 0,0,1;
        W1 + W2, 0 , L1 + L2 , 0, 1, 0;];
        
% Now we can calculate the Bs:
B1 =  pinv(Ad_M)*S1;
B2 = pinv(Ad_M)*S2;
B3 = pinv(Ad_M)*S3;
B4 = pinv(Ad_M)*S4;
B5 = pinv(Ad_M)*S5;
B6 = pinv(Ad_M)*S6;

% It'll be useful also to have the matrix representation of omega for each of the screw axes, and v as a separate vector.
% Space frame:



wS1b = [ 0 -1 0; 1, 0, 0; 0, 0, 0];
wS2b = [ 0 0 1; 0, 0, 0; -1, 0, 0];
wS3b = [ 0 0 1; 0, 0, 0; -1, 0, 0];
wS4b = [ 0 0 1; 0, 0, 0; -1, 0, 0];
wS5b = [ 0 1 0; -1, 0, 0; 0, 0, 0];
wS6b = [ 0 0 1; 0, 0, 0; -1, 0, 0];
% And we already have the v vectors for the space frame calculated above.
% Body frame:(1], 0];
wB1b = [ 0 -B1(3) B1(2); B1(3), 0, -B1(1); -B1(2), B1(1), 0];
wB2b = [ 0 -B2(3) B2(2); B2(3), 0, -B2(1); -B2(2), B2(1), 0];
wB3b = [ 0 -B3(3) B3(2); B3(3), 0, -B3(1); -B3(2), B3(1), 0];
wB4b = [ 0 -B4(3) B4(2); B4(3), 0, -B4(1); -B4(2), B4(1), 0];
wB5b = [ 0 -B5(3) B5(2); B5(3), 0, -B5(1); -B5(2), B5(1), 0];
wB6b =[ 0 -B6(3) B6(2); B6(3), 0, -B6(1); -B6(2), B6(1), 0];

vB1 = B1(4:6);
vB2 = B2(4:6);
vB3 = B3(4:6);
vB4 = B4(4:6);
vB5 = B5(4:6);
vB6 = B6(4:6);

% With all of that infrastructure set up, now we can follow the iterative algorithm described above Example 6.1 in the textbook, starting "To modify this algorithm to work with a desired end-effector configuration represented as T_sd...":

% Initialize the current guess to the user-supplied value:
thguess = theta0;
% Set large initial values for wb and vb to ensure the loop is entered:
wb = 10*ones(3,1);
vb = 10*ones(3,1);

    while norm(wb) > 1e-3 || norm(vb) > 1e-3

        % For more concise code below, let's make each element of thguess a separate variable:
          th1 = thguess(1);
          th2 = thguess(2);
          th3 = thguess(3);
          th4 = thguess(4);
          th5 = thguess(5);
          th6 = thguess(6);

        % We have the desired pose T given by the user, and can get the current pose Tab from the forward kinematics.
        % With the PoE FK formulation, Tab = exp([S1]theta1)*...*M. To calculate exp([S]theta), we need exp([w]theta), so let's calculate the latter first:
          e_wS1 = eye(3) + sin(th1)*wS1b + (1-cos(th1))*wS1b*wS1b;
          e_wS2 = eye(3) + sin(th2)*wS2b + (1-cos(th2))*wS2b*wS2b;
          e_wS3 = eye(3) + sin(th3)*wS3b + (1-cos(th3))*wS3b*wS3b;
          e_wS4 = eye(3) + sin(th4)*wS4b + (1-cos(th4))*wS4b*wS4b;
          e_wS5 = eye(3) + sin(th5)*wS5b + (1-cos(th5))*wS5b*wS5b;
          e_wS6 = eye(3) + sin(th6)*wS6b + (1-cos(th6))*wS6b*wS6b;

         %%the asterik for the formula
          as1 = (eye(3)*th1 + (1-cos(th1))*wS1b + (th1 - sin(th1))* wS1b*wS1b)*v1;
          as2 = (eye(3)*th2 + (1-cos(th2))*wS2b + (th2 - sin(th2))* wS2b*wS2b)*v2;
          as3 = (eye(3)*th3 + (1-cos(th3))*wS3b + (th3 - sin(th3))* wS3b*wS3b)*v3;
          as4 = (eye(3)*th4 + (1-cos(th4))*wS4b + (th4 - sin(th4))* wS4b*wS4b)*v4;
          as5 = (eye(3)*th5 + (1-cos(th5))*wS5b + (th5 - sin(th5))* wS5b*wS5b)*v5;
          as6 = (eye(3)*th6 + (1-cos(th6))*wS6b + (th6 - sin(th6))* wS6b*wS6b)*v6;

          %use formula on formula sheet
        % Now use that for calculating the exponentials of the screw axes:
          e_S1 = [e_wS1, as1; 0,0,0,1];
          e_S2 = [e_wS2, as2; 0,0,0,1];
          e_S3 = [e_wS3, as3; 0,0,0,1];
          e_S4 = [e_wS4, as4; 0,0,0,1];
          e_S5 = [e_wS5, as5; 0,0,0,1];
          e_S6 = [e_wS6, as6; 0,0,0,1];
        % And now we can finally calculate the FK:
          Tab = e_S1*e_S2*e_S3*e_S4*e_S5*e_S6*M;

        % Working in the body frame, the twist that takes us from the current pose to the desired one is Tbd = Tba*T (since T given by the user should be understood as Tad, the desired pose in the world frame):
          Tbd = inv(Tab)*T;

        % But we want that in vector form, so we take the matrix log, using the algorithm in section 3.3.3.2 of the textbook:


        % formula states e[s]theta = [R, P; 0,1]; and the inverse matrix log of
        % that is [s]theta = [wmatrix(theta), v(theta), 0, 1];
        R_bd = Tbd(1:3,1:3);
        P_bd = Tbd(1:3,4);


            %we also need to find w and theta in conditions we've found before 
            if (R_bd-eye(3) < 10e-5)
                w_bd = [0,0,0];
                theta_bd = 0; 
                 w_bd_matrix = [ 0 -w_bd(3) w_bd(2); w_bd(3), 0, -w_bd(1); -w_bd(2), w_bd(1), 0];
            elseif (trace(R_bd)+1 < 10e-5)
                    theta_bd = pi; 
                 if( R_bd(3,3) ~= -1)
                     w_bd = 1/ sqrt(2 * (1+R_bd(3,3))) * [R_bd(1,3); R_bd(2,3); 1+R_bd(3,3)];
                 elseif( R_bd(2,2) ~= -1)
                     w_bd = 1/ sqrt(2 * (1+R_bd(2,2))) * [R_bd(1,2);1+R_bd(2,2); R_bd(3,2)];
                 elseif( R_bd(1,1) ~= -1)
                     w_bd = 1/ sqrt(2 * (1+R_bd(1,1))) * [1+ R_bd(1,1); R_bd(2,1); R_bd(3,1)];
                 end
                 w_bd_matrix = [ 0 -w_bd(3) w_bd(2); w_bd(3), 0, -w_bd(1); -w_bd(2), w_bd(1), 0];



            else
                theta_bd = acos(( trace(R_bd)-1)/2 );
                w_bd_matrix = 1/(2*sin(theta_bd)) * (R_bd- R_bd');
            end 






        Gin = (1/theta_bd) * eye(3) -.5*w_bd_matrix + ( 1/theta_bd - .5*cot(theta_bd/2))* w_bd_matrix^2;
        %algorthim according to the textbook
        % its never going ti be exactly the identity mtrix entries may be close to
        % 1 or 0 but never equalling it 
        if((R_bd - eye(3)) < 10e-5) 
            w_bd = [0,0,0];
            v_bd = P_bd/norm(P_bd);
            theta_bd = norm(P_bd);
        else 
            v_bd = Gin * P_bd;
        end  


        % That matrix log gives us the matrix form of the body twist, [Vb]:
          Vbb = [ w_bd_matrix, v_bd; 0, 0, 0, 0] *theta_bd;
        % Convert that to the vector form, Vb:
        % we want to find [S]theta = [ [w]theta, vtheta; 0,0,0,0]
          Vb = [ matrix2vector(Vbb(1:3,1:3))';Vbb(1:3,4)];
         
          %Vb= [(w_bd_matrix * theta_bd, v_bd*theta_bd; 0,0,0,0];
        % We can also split that up into the omega and v components, for use in the loop termination condition above:
          wb = Vb(1:3); vb = Vb(4:6);

        % We want to update theta by adding ((the pseudoinverse of Jb) * Vb) to the old guess. So next we calculate Jb.
        % We can do that using the equation for each column of Jb given in the lecture, which is equation (5.18) in the textbook.
        % For that, we'll need exp([B]theta) for each of the screw axes in the body frame. We do that as we did for exp([S]theta above):
          e_wB1 = eye(3) + sin(th1)*wB1b + (1-cos(th1))*wB1b*wB1b;
          e_wB2 = eye(3) + sin(th2)*wB2b + (1-cos(th2))*wB2b*wB2b;
          e_wB3 = eye(3) + sin(th3)*wB3b + (1-cos(th3))*wB3b*wB3b;
          e_wB4 = eye(3) + sin(th4)*wB4b + (1-cos(th4))*wB4b*wB4b;
          e_wB5 = eye(3) + sin(th5)*wB5b + (1-cos(th5))*wB5b*wB5b;
          e_wB6 = eye(3) + sin(th6)*wB6b + (1-cos(th6))*wB6b*wB6b;

          %calulate the asterik again 
          as1b = (eye(3)*th1 + (1-cos(th1))*wB1b + (th1 - sin(th1))* wB1b*wB1b)*vB1;      
          as2b = (eye(3)*th2 + (1-cos(th2))*wB2b + (th2 - sin(th2))* wB2b*wB2b)*vB2;
          as3b = (eye(3)*th3 + (1-cos(th3))*wB3b + (th3 - sin(th3))* wB3b*wB3b)*vB3;
          as4b = (eye(3)*th4 + (1-cos(th4))*wB4b + (th4 - sin(th4))* wB4b*wB4b)*vB4;
          as5b = (eye(3)*th5 + (1-cos(th5))*wB5b + (th5 - sin(th5))* wB5b*wB5b)*vB5;
          as6b = (eye(3)*th6 + (1-cos(th6))*wB6b + (th6 - sin(th6))* wB6b*wB6b)*vB6;


          e_B1 = [e_wB1, as1b; 0,0,0,1];
          e_B2 = [e_wB2, as2b; 0,0,0,1];
          e_B3 = [e_wB3, as3b; 0,0,0,1];
          e_B4 = [e_wB4, as4b; 0,0,0,1];
          e_B5 = [e_wB5, as5b; 0,0,0,1];
          e_B6 = [e_wB6, as6b; 0,0,0,1];
        % For each column of the Jacobian, we need to calculate some product of the exp([B]theta) matrices:
          prodJb1 = inv( e_B2 * e_B3 * e_B4 * e_B5 * e_B6);
          prodJb2 = inv(e_B3 * e_B4 * e_B5 * e_B6);
          prodJb3 = inv(e_B4 * e_B5 * e_B6);
          prodJb4 = inv(e_B5 * e_B6);
          prodJb5 = inv(e_B6);
          prodJb6 = eye(4);

          %%% Jacobian is a 6x6 matrix
        % and then take the adjoint representation of each resulting matrix, and multiply that by the i'th body screw axis B_i:
          Jb1 = [prodJb1(1:3,1:3), zeros(3); vector2matrix(prodJb1(1:3,4))* prodJb1(1:3,1:3), prodJb1(1:3,1:3)] * B1;  
          Jb2 = [prodJb2(1:3,1:3), zeros(3); vector2matrix(prodJb2(1:3,4))* prodJb2(1:3,1:3), prodJb2(1:3,1:3)] * B2;   
          Jb3 = [prodJb3(1:3,1:3), zeros(3); vector2matrix(prodJb3(1:3,4))* prodJb3(1:3,1:3), prodJb3(1:3,1:3)] * B3;  
          Jb4 = [prodJb4(1:3,1:3), zeros(3); vector2matrix(prodJb4(1:3,4))* prodJb4(1:3,1:3), prodJb4(1:3,1:3)] * B4;  
          Jb5 = [prodJb5(1:3,1:3), zeros(3); vector2matrix(prodJb5(1:3,4))* prodJb5(1:3,1:3), prodJb5(1:3,1:3)] * B5; 
          Jb6 = [prodJb6(1:3,1:3), zeros(3); vector2matrix(prodJb6(1:3,4))* prodJb6(1:3,1:3), prodJb6(1:3,1:3)] * B6; 

        % The body Jacobian is the matrix with those vectors as its columns:
        Jb = [Jb1 Jb2 Jb3 Jb4 Jb5 Jb6];

        % We now have everything we need to update the guess for the angle vector:
        thguess = thguess + pinv(Jb)*Vb;

    end

    % Finally, when the loop has terminated because the guess has converged, return that value:
    theta = real(thguess);
    
    
    %Modulate the angle 
  
    thetaMod = mod(theta, 2*pi);
    
    for anglecount = 1:length(thetaMod)
        if thetaMod(anglecount) > pi 
            thetaMod(anglecount) = thetaMod(anglecount) -(2*pi);
        else
            thetaMod(anglecount) = thetaMod(anglecount);
        end
    end
    
    theta=thetaMod;
    
end

function v=matrix2vector(matrix)
   v= [matrix(3,2), matrix(1,3), matrix(2,1)];
end

function m = vector2matrix(v)

m = [ 0 -v(3) v(2); v(3), 0, -v(1); -v(2), v(1), 0];

end










function top_corn_pix = tcPixel(RedOnly)

top_corn_pix = [0,0];
for i = 1:height(RedOnly) 
    for j = 1:length(RedOnly)-1
        if RedOnly(i,j) == 1
            top_corn_pix = [i,j];
            return  
        end%endbif loop
    end %end length 
end %end height 
end %end function 






function bot_corn_pix = bcPixel(RedOnly)
%trying to find the bottom corner. from bottom up and from right to left
bot_corn_pix = [0,0];
for i = height(RedOnly):-1:(height(RedOnly)/2)
    for j = length(RedOnly)-1:-1:1
        if RedOnly(i,j) == 1
            bot_corn_pix = [i,j];
            return
        end%end if loop
    end %end length 
end %end height 

end%end fucntion

function [commandlist,theta_p] = Lael_IVKshell(waypoints,guessinitial)
% Inverse velocity kinematics to trace out some trajectory.


    



% (This shell doesn't include the code for starting ROS, setting up the publisher and message variables, etc.)

% Number of waypoints in your trajectory:
N = height(waypoints); 

% Set a series of waypoints with desired poses and associated times:


% Use your inverse kinematics function to determine corresponding joint angles:
theta = zeros(6,N);
% For the first waypoint, we provide a guess for the corresponding pose; for the rest, the previous pose is a good guess for the next one
theta(:,1) = Lael_IKshell([ -1, 0,0, waypoints(1,1); 0, 1, 0, waypoints(1,2);0,0, -1, waypoints(1,3); 0,0,0,1],guessinitial);

for i=2:N
  theta(:,i) = Lael_IKshell([ -1, 0,0, waypoints(i,1); 0, 1, 0, waypoints(i,2);0,0, -1, waypoints(i,3); 0,0,0,1],theta(:,i-1));  
 
end

theta_p=theta(:,N);


% Now there are the two ways to proceed described in the lab handout. To use the usual position controller:
for i=1:N
 commandlist(i) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
 commandlist(i).Positions = theta(:,i);
 commandlist(i).TimeFromStart.Sec = waypoints(i,4);
 %commandlist(i).TimeFromStart.Nsec = waypoints(i,4);
 % Note that both Sec and Nsec need to be integers, and Nsec must be less than 1e9
 if i<N
% ** Set the desired joint velocities at each waypoint up until the last:
% The equation for velocity is v = theta_desired - theta_current/ change in
% time. according to the lab. I dont have this in my note tho
   commandlist(i).Velocities = ((theta(:, i+1)- theta(:,i))/ (waypoints(i+1,4)-waypoints(i,4)));
 else
% (but at the last waypoint, the joint velocity should be 0)
   commandlist(i).Velocities = zeros(6,1);
 end
end

commandlist;
end

function ismoving = ismoving(possub,InLab)
% Returns true if the end effector has a nonzero velocity

  posmsg1 = receive(possub);
  
  if InLab
    x1 = posmsg1.Transforms(1).Transform.Translation.X;
    y1 = posmsg1.Transforms(1).Transform.Translation.Y;
    z1 = posmsg1.Transforms(1).Transform.Translation.Z;
    rx1 = posmsg1.Transforms(1).Transform.Rotation.X;
    ry1 = posmsg1.Transforms(1).Transform.Rotation.Y;
    rz1 = posmsg1.Transforms(1).Transform.Rotation.Z;
    
    pause(.02);
    
    posmsg2 = receive(possub);
    x2 = posmsg2.Transforms(1).Transform.Translation.X;
    y2 = posmsg2.Transforms(1).Transform.Translation.Y;
    z2 = posmsg2.Transforms(1).Transform.Translation.Z;
    rx2 = posmsg2.Transforms(1).Transform.Rotation.X;
    ry2 = posmsg2.Transforms(1).Transform.Rotation.Y;
    rz2 = posmsg2.Transforms(1).Transform.Rotation.Z;

    deltapos = [abs(x2-x1),abs(y2-y1),abs(z2-z1),abs(rx2-rx1),abs(ry2-ry1),abs(rz2-rz1)];
    if sum(deltapos) >= 1e-4 % Make stricter if necessary
      ismoving = true;
    else
      ismoving = false;
    end
  
  else  % using Gazebo
    x1 = posmsg.Pose(6).Position.X;
    y1 = posmsg.Pose(6).Position.Y;
    z1 = posmsg.Pose(6).Position.Z;

    pause(.02);

    posmsg2 = receive(possub);
    x2 = posmsg2.Transforms(1).Transform.Translation.X;
    y2 = posmsg2.Transforms(1).Transform.Translation.Y;
    z2 = posmsg2.Transforms(1).Transform.Translation.Z;
    rx2 = posmsg2.Transforms(1).Transform.Rotation.X;
    ry2 = posmsg2.Transforms(1).Transform.Rotation.Y;
    rz2 = posmsg2.Transforms(1).Transform.Rotation.Z;

    deltapos = [abs(x2-x1),abs(y2-y1),abs(z2-z1)];
    if sum(deltapos) >= 1e-4 % Make stricter if necessary
      ismoving = true;
    else
      ismoving = false;
    end

  end
end
function [WaypointOutput] = aStar(p1,p2)
        %Status: INCOMPLETE
        
        %Runs A* to find a path and return waypoints
        %Useful Resource:
        %https://www.geeksforgeeks.org/a-search-algorithm/
        %Outputs: 
            % wayPoints: a matrix of waypoints [w1 w2 w3 ...] that the end
            % effector should travel between. Each waypoint is given in the
            % form [x; y]

        %TODO (Start early, this will take a while)
       
        
        %Initializing starting point 

      
        
        goal_x = p2(1);
        goal_y = p2(2);
        
        start_x = p1(1);
        start_y = p2(2);
        %node = [x,y,g,h,f,px,py]
        
        

        start = [start_x, start_y,0,0,0,0,0];
        
        openList = start;
        closedList = [];
   
        
        while isempty(openList) == false
           
       %g is movement cost from starting point to point on grid 
        
        %h is estimated movement cost to move from given square to final
        %destination 
        %should be distance using formula with robot positions 
             %h= sqrt((openList(i)- goal_x)^2 + (openList(i)- goal_y)^2
             
             %finding the minimum f and the corresponding node f is 5th
             %ebtry in my node 
             
             
             
             [F,min_index]= min(openList(:,5));
             
             q=openList(min_index,:);
             
             
           
            
             children = genKids(q,p2);
       
             %generating children 
             %to calculate g. calulate g at start then add ecladian
             %distance between 2 points . i do this in function below
             
             
         
             %%checking to see if child is  already at the goal point 
             for child = 1:height(children)

                 skip = false;
                 
                 if children(child,4) < gridSize %end the loop if at goal
                     closedList(1,:) = start; %make the first tow of closedList the starter node
                     closedList = [closedList;q; children(child,:)];
                     
                     WaypointOutput = children(child, :);
                     parents = children(child,6:7);
                     
                     
                     
      
                  
                     
                     %loop that works backwards and tracksdown the parents 
                     
                     while ~(parents(1) == start_x && parents(2) == start_y)
                         
                         %checks to see if there is a matching parent in
                         %the closed List. parent is index
                         parent = equal_coords(closedList(:,1:2),parents);
                         WaypointOutput = [closedList(parent,:);WaypointOutput];
                         parents = closedList(parent,6:7);
                     end %end while loop
                     
                     WaypointOutput(1,:) = start;
                     WaypointOutput(:,3:7) = []
                     lengthPath = children(child,3);
                     
                     return
                 
                         
               
                  
                         
                           %checking to see if it already on openList
                             
                             for j = 1: height(openList) 
                                 
                                 %if f is less in child that what is
                                 %currently on openList then we are going
                                 %to add it to open list 
                                 
                                 %checking to see if a child with the same
                                 %x and y as a node on openList has a less
                                 %f than child 
                         
                                 
                                 if (children(child,1) == openList(j,1)) && (children(child,2) == openList(j,2)) 
                                     if openList(j,5) <= children(child,5)
                                     skip = true;
                                     else 
                                       openList(j,:) = children(child,:);
                                       skip = true;
                                     end
                                     
                                 end%end if statement
                             end %end for loop
                   
                          
                              
                    %checking to see if in closedList 
                              
                             for c = 1:height(closedList)
                                 
                                 %if f is less in child that what is
                                 %currently on openList then we are going
                                 %to add it to open list 
                                 if (children(child,1) == closedList(c,1)) && (children(child,2) == closedList(c,2))
                                     
                                    if closedList(c,5) <= children(child,5)
                                        skip = true;
                                   
                                    end
                                 end

                             end% end closed for loop
                   
             
             %adding it to the list 
                 if skip == false 
                 openList = [openList;children(child,:)];
             end


            
 
                 end  %ends if loop
             end %ends for loop for children
             
              %removing q from open list
             
             openList(min_index,:) = [];

             closedList = [closedList; q];

    

                                       
                                
    end %ends while loop
   end  % ends function
        
   
  function children = genKids(q,p2)
             
             %generating 8 children 
             
             goal_x = p2(1);
             goal_y = p2(2);


             % upper right diagonal
             
             x1 = q(1) + gridSize;
             y1 = q(2) +gridSize;
             g1 = sqrt( (x1-q(1))^2 + (y1-q(2))^2) + q(3);
             h1 = sqrt((x1-goal_x)^2 + (y1-goal_y)^2);
             f1= g1+h1;
             p1x = q(1);
             p1y= q(2);
             
             child_1 = [x1,y1,g1,h1,f1,p1x,p1y];
             
             %right 
             
             x2 = q(1) + gridSize;
             y2 = q(2);
             g2 = sqrt( (x2-q(1))^2 + (y2-q(2))^2) + q(3);
             h2 = sqrt((x2-goal_x)^2 + (y2-goal_y)^2);
             f2= g2+h2;
             p2x = q(1);
             p2y= q(2);
             
             child_2 = [x2,y2,g2,h2,f2,p2x,p2y];
             
             
             %lower right diagonal 
             
             x3 = q(1) + gridSize;
             y3 = q(2) - gridSize;
             g3 = sqrt( (x3-q(1))^2 + (y3-q(2))^2) + q(3);
             h3 = sqrt((x3-goal_x)^2 + (y3-goal_y)^2);
             f3= g3+h3;
             p3x = q(1);
             p3y= q(2);
             
             child_3 = [x3,y3,g3,h3,f3,p3x,p3y];
             
             %straight up
             
             x4 = q(1);
             y4 = q(2) + gridSize;
             g4 = sqrt( (x4-q(1))^2 + (y4-q(2))^2)+q(3);
             h4 = sqrt((x4-goal_x)^2 + (y4-goal_y)^2);
             f4= g4+h4;
             p4x = q(1);
             p4y= q(2);
             
             child_4 = [x4,y4,g4,h4,f4,p4x,p4y];
             
             
             %straight down
             
             x5 = q(1);
             y5 = q(2) - gridSize;
             g5 = sqrt( (x5-q(1))^2 + (y5-q(2))^2)+q(3);
             h5 = sqrt((x5-goal_x)^2 + (y5-goal_y)^2);
             f5= g5+h5;
             p5x = q(1);
             p5y= q(2);
             
             child_5 = [x5,y5,g5,h5,f5,p5x,p5y];
             
             % upper left 
             
             
             x6 = q(1)-gridSize;
             y6 = q(2) + gridSize;
             g6 = sqrt( (x6-q(1))^2 + (y6-q(2))^2)+q(3);
             h6 = sqrt((x6-goal_x)^2 + (y6-goal_y)^2);
             f6= g6+h6;
             p6x = q(1);
             p6y= q(2);
             
             child_6 = [x6,y6,g6,h6,f6,p6x,p6y];
             
             
             %left
             
             x7 = q(1)-gridSize;
             y7 = q(2);
             g7 = sqrt( (x7-q(1))^2 + (y7-q(2))^2)+q(3);
             h7 = sqrt((x7-goal_x)^2 + (y7-goal_y)^2);
             f7= g7+h7;
             p7x = q(1);
             p7y= q(2);
             
             child_7 = [x7,y7,g7,h7,f7,p7x,p7y];
             
             %lower left diagonal 
             
             x8 = q(1)-gridSize;
             y8 = q(2)-gridSize;
             g8 = sqrt( (x8-q(1))^2 + (y8-q(2))^2)+q(3);
             h8 = sqrt((x8-goal_x)^2 + (y8-goal_y)^2);
             f8= g8+h8;
             p8x = q(1);
             p8y= q(2);
             
             child_8 = [x8,y8,g8,h8,f8,p8x,p8y];
           
             %may need to check identation here 
             
             children = [child_1;child_2;child_3;child_4;child_5;child_6;child_7;child_8];
             
             
             
    end  
    

