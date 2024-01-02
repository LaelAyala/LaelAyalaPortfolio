
Lael_Lab4('maze_medium.txt')

function Lael_Lab4(filename)
    %When run, this function should devise a route through the given
    % obstacles and command the UR5e to move along that path. 

    %Inputs: 
        % filename: string with the name of a text file describing obstacle positions.
        % The file should be formatted as: 

        % robot-start-x robot-start-y robot-goal-x robot-goal-y
        % obstacle1-x-min obstacle1-y-min obstacle1-x-max obstacle1-y-max
        % obstacle2-x-min obstacle2-y-min obstacle2-x-max obstacle2-y-max
        % ...
        
        % Note that the first 4 obstacles (lines 2,3,4,5), will be the
        % outer boundaries of the table in the lab.

    %Returns:
        %none


    % Function-Wide Variables
    obstacleOffset = 5; % Virtual padding around obstacles to account for end-effector width
    gridSize = 15; % Size of side of each grid square in mm, smaller number means higher resolution
   


    %Plot to Help Visualize Map (Not Necessary)
    plotRectangle(filename)

    %Determine Shortest Path Through Maze
    [WaypointOutput,lengthPath] = aStar(filename);
    disp(['Length of Path: ',num2str(lengthPath),'mm'])



    plotPath(WaypointOutput)

    % Move The Robot Along Returned Path
    executeTrajectory(WaypointOutput,filename);

    %Functions

function plotpath = plotPath(waypoints)
        figure()
        axis equal
        hold on
        for i = 1:length(waypoints(:,1))
            plot(waypoints(i,1), waypoints(i,2), 'g*')
        end
    end

    

    function [startPos, endPos, obstacles] = processFile(filename)
        %Status: COMPLETE

        %Converts the raw strings in "filename" to a start point, end point,
        %and obstacle matrix.
        %Inputs:
            %filename: a string with the name of the file
        %Returns:
            % startPos: a 1x2 vector of the robot's start position (x,y)
            % endPos: a 1x2 vector of the robot's end position (x,y)
            %obstacles: a matrix containing all the obstacles in the file
            %in the format:
            %[ c1 c2 c3 c4;
            %... ]
            %where c1...c4 are the coordinates of each rectangular
            %obstacle, starting in the upper leftmost corner (when facing
            %the robot, sitting at the computer) and going clockwise
            
        % Open file
        file = splitlines(fileread(filename));

        % the delimiter for each value is 1 space.
        numMat = str2double(split(file, ' '));
        startPos = numMat(1, 1:2);
        endPos = numMat(1, 3:4);
        obstacles = numMat(2:end, :);        
    end

    function plotit = plotRectangle(filename)
        %Status: COMPLETE

        %This function helps for visualizations, but is not used for
        %anything else
        [startPos, endPos, Obstacles] = processFile(filename);
        
        % plot the obstacles as seen in the workspace
        figure()
        axis equal
        hold on
        for i = 1:size(Obstacles, 1)
            lowCorner = Obstacles(i, 1:2);
            width = Obstacles(i, 3) - Obstacles(i, 1);
            height = Obstacles(i, 4) - Obstacles(i, 2);
            pos = [lowCorner width height];
            rectangle('Position', pos,'FaceColor',[0 0.5 1])
        end
        
        % plot the start and end positon (center of robot)
        plot(startPos(1, 1), startPos(1, 2), 'r*')
        plot(endPos(1, 1), endPos(1, 2), 'g*')
        legend('start', 'goal')
    
    end

    function inObstacle = isInObstacle(x,y, obstacleList)
        %Status: COMPLETE

        %Checks if a point is inside of an obstacle or not
        %Inputs:
            %x (double): an x coordinate in the frame of the table
            %y (double): a y coordinate in the frame of the table
	    %obstacleList: a matrix in the same format used elsewhere in this file
        %Outputs:
            %inObstacle (boolean): true if the point is in obstacle
            x_minimums = obstacleList(:,1);
            y_minimums = obstacleList(:,2);
            x_maximums = obstacleList(:,3);
            y_maximums = obstacleList(:,4);
            
           inObstacle = false;
            
            for i = 1:length(x_minimums)
                if (x >= x_minimums(i)- obstacleOffset && x <= x_maximums(i)+obstacleOffset) && (y >= y_minimums(i)-obstacleOffset && y <= y_maximums(i)+obstacleOffset)
                    inObstacle = true; 
                end 
            end 
                    
                    

    end

    function index = equal_coords(A,pos)
        %Status: COMPLETE

        %Potentially useful helper function for returning first index from a list that matches
        %the desired position

        %Inputs: A (2xn matrix of n points [x1,x2,...,xn;y1,y2,...yn])
        %        pos (1x2 vector [x,y])
        %Returns: index (integer) corresponding to the index (values range 
        % from 1 to n, inclusive) of the first position in the list that has the same
        % value as pos.
        index = [];
        for j=1:height(A)
            if abs(pos(1)-A(j,1)) < 1e-6 && abs(pos(2)-A(j,2)) < 1e-6
                index = j;
                return
            end
        end
    end

    function [WaypointOutput,lengthPath] = aStar(filename)
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

      
        [startpos, endpos, obstacles] = processFile(filename);
        goal_x = endpos(1);
        goal_y = endpos(2);
        
        start_x = startpos(1);
        start_y = startpos(2);
        %node = [x,y,g,h,f,px,py]
        
        obstacleList = obstacles

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
             
             
           
            
             children = genKids(q);
       
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
                 
                         
                 elseif isInObstacle(children(child,1),children(child,2),obstacleList) == 1
                         %skip need to create a skip boolea
                         skip = true;
                 else
                  
                         
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
        
    
        
        

   

    function executeTrajectory(wayPoints,filename)
        %Status: INCOMPLETE
        
        %Moves the robot to the initialization position.
        %Translates the robot to the start position [x,y] while remaining
        %in the raised z plane.
        %Lowers the robot straight down to the specified z plane.
        %Moves the robot through the list of provided waypoints in the
        %specified z plane.
        %Raises the robot straight up to the raised z plane.
        %Moves the robot to the initialization position.

        %TODO

        N= height(wayPoints);
        T = [-1 0 0 665;
            0 1 0 0;
            0 0 -1 100;
            0 0 0 1];

        theta_0 = [-pi/18, -pi/3,pi/2,-2 *pi/3, -pi/2, 4*pi/9];

        

        %moving it to x and y of start

        [startpos, endpos, obstacles] = processFile(filename);
        
        
        start_x = startpos(1);
        start_y = startpos(2);
        
        
        %Isabel helped me with this
        start= [665 0 100 3];
        new_start= [wayPoints(1,:),-80,6];
        new_end = [wayPoints(N,:), 0 ,0];
        
        time = zeros(N,1);
        low_z = -80 * ones(N,1);
        waypoints = [wayPoints,low_z, time]; % making wayPoints  have 4 columns 
        
        waypoints = [start;new_start;waypoints;new_end] ;
        for i = 3:N+3 %adding time to each waypoint
            waypoints(i,4) = i+5; %extra seconds for the initial move
        end
        waypoints(N+3,4) = N+3+10;
        
        waypoints = waypoints';


        
        % Inverse velocity kinematics to trace out some trajectory.

rosinit;

arm = 0;
%%First we need to set-up a intitializer for robot and armcmd. Kade being
%%an absolute genius used an if else statment that would senf to gazebo if
%%true robot if false . so thats what we gon do

if arm == 1
    armCmd = rospublisher('/eff_joint_traj_controller/command');
    possub= rossubscriber('/gazebo/link_states');
    testMsg = rosmessage(armCmd);
    testMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint','wrist_1_joint','wrist_2_joint', 'wrist_3_joint'};
    
else
    
    robotCmd = rospublisher('scaled_pos_joint_traj_controller/command');
    possub= rossubscriber('tf');
    testMsg = rosmessage(robotCmd);
    testMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint','wrist_1_joint','wrist_2_joint', 'wrist_3_joint'};
    
end
    



% (This shell doesn't include the code for starting ROS, setting up the publisher and message variables, etc.)

% Number of waypoints in your trajectory:



%}
% Use your inverse kinematics function to determine corresponding joint angles:
theta = zeros(6,N);
% For the first waypoint, we provide a guess for the corresponding pose; for the rest, the previous pose is a good guess for the next one
%theta(:,1) = Lael_IKshell([ 0, -.0565,.77793, waypoints(1,1); -.76, .1879, -.622, waypoints(1,2);.1816,.9806, .0743, waypoints(1,3); 0,0,0,1],[0 0 0 0 0 0]);
theta(:,1) = Lael_IKshell([ -1, 0,0, waypoints(1,1); 0, 1, 0, waypoints(1,2);0, 0, -1, waypoints(1,3); 0,0,0,1],theta_0);

for i=2:N +3
  theta(:,i) = Lael_IKshell([ -1, 0,0, waypoints(i,1); 0, 1, 0, waypoints(i,2);0,0, -1, waypoints(i,3); 0,0,0,1],theta(:,i-1));  
  %theta(:,i) = Lael_IKshell([ -1, 0,0, waypoints(i,1); 0, 0, 1, waypoints(i,2);0,1, 0, waypoints(i,3); 0,0,0,1],theta(:,i-1));
  %theta(:,i) = Lael_IKshell([ .624, -.0565,.77793, waypoints(1,1); -.76, .1879, -.622, waypoints(1,2);.1816,.9806, .0743, waypnts(1,3); 0,0,0,1     ],theta(:,i-1));
end

% Now there are the two ways to proceed described in the lab handout. To use the usual position controller:
for i=1:N+3
 commandlist(i) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
 commandlist(i).Positions = theta(:,i);
 commandlist(i).TimeFromStart.Sec = waypoints(i,4);
 commandlist(i).TimeFromStart.Nsec = waypoints(i,4);
 % Note that both Sec and Nsec need to be integers, and Nsec must be less than 1e9
 if i<N+3
% ** Set the desired joint velocities at each waypoint up until the last:
% The equation for velocity is v = theta_desired - theta_current/ change in
% time. according to the lab. I dont have this in my note tho
   commandlist(i).Velocities = ((theta(:, i+1)- theta(:,i))/ (waypoints(i+1,4)-waypoints(i,4)));
 else
% (but at the last waypoint, the joint velocity should be 0)
   commandlist(i).Velocities = zeros(6,1);
 end
end

testMsg.Points = commandlist;

if arm == 1
    
    send(armCmd, testMsg)
else 
    send(robotCmd, testMsg)
end 
% Now "send(armCmd,shapeMsg)" will execute your trajectory, if shapeMsg.Points = commandlist and shapeMsg.JointNames is set appropriately as usual.
%{
% Alternatively, to use the pure velocity controller:

% Make sure velCmd, velMsg, robotvelCmd, robotvelMsg, and jSub have been created, as described in the handout:
% velCmd = rospublisher('/eff_joint_traj_controller/command');
% velMsg = rosmessage(velCmd);
% robotvelCmd = rospublisher('/joint_group_vel_controller/command');
% robotvelMsg = rosmessage(robotvelCmd);
% jSub = rossubscriber('joint_states');

for i=1:size(theta,2)-1
  % get current position:
  jMsg = receive(jSub);
  % switch the order of the joints in the message that returns, to match the actual joint order:
  angles = jMsg.Position([3 2 1 4 5 6]);
  % set the velocity based on the next desired position and the actual current position:
  velMsg.Data =
  robotvelMsg.Data = velMsg.Data;
  % execute the velocity command:
  send(velCmd,velMsg);
  send(robotvelCmd,robotvelMsg);
  % wait until the next time step:
  pause(
end

% stop the robot!
velMsg.Data = zeros(6,1);
robotvelMsg.Data = velMsg.Data;
send(velCmd,velMsg);
send(robotvelCmd,robotvelMsg);

end
%}





    end
     %g is movement cost from starting point to point on grid 
            
            
        
        %h is estimated movement cost to move from given square to final
        %destination 
        %should be distance using formula with robot positions 
         %h= sqrt((openList(i)- goal_x)^2 + (openList(i)- goal_y)^2
             
             
             
        
             
             
             
             
             
             
             %generating children 
             %to calculate g. calulate g at start then add ecladian
             %distance between 2 points 
             
             
    function children = genKids(q)
             
             %generating 8 children 
             [startpos, endpos, obstacles] = processFile(filename);
             goal_x = endpos(1);
             goal_y = endpos(2);


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


end


