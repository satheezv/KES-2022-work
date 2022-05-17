
clear all

figure

syms L1 L2 L3 L4 L5 L6 Q1 Q2 Q3 Q4 Q5 Q6


dhparams = [0.3   	-pi/2   -.6     0;
            -.700   0       0       0;
            -.185   +pi/2   0       0;
            0       -pi/2	0.624   0;
            0       +pi/2	0       0;
            0       0       0.255   0;
            -0.120  0       0       0];

        
alphaa=[-90,0,90,-90,90, 0, 0]; % this is the alpha value for all  the link
a=[0.3,-.70, -.185, 0, 0, 0, -0.120]; % Length of the Link
d=[-.6,0,0,.624,0, .255, 0], 0; %Offset
Q=[Q1,Q2,Q3,Q4,Q5,Q6, 0 ]; % joint angle variation
%init_guess=[0,-pi/2,0,0,-pi/2,0, 0]' % home position
%init_guess=[-2.7672,1.7752,0,0,0,0,0]'
init_guess=[0,0,-pi/2,0,0,0,0]'
%%Transformation Matrices
for i=1:7
    switch i
        case 1
            T01= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 2
            T12= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 3
            T23= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 4
            T34= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 5
            T45= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 6
            T56= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 7
            T67= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
    end
end
T01 % first link with respect to base
T02= T01*T12
%you are looking for T05 then
T07= T01*T12*T23*T34*T45*T56*T67
        
%%
        
robot = robotics.RigidBodyTree;
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
jnt1.HomePosition=pi;
jnt1.PositionLimits=[-3.14159,3.14159]
setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base')

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
jnt2.HomePosition=0;
jnt2.PositionLimits=[-0.5,2.70526]

body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
jnt3.HomePosition=-pi/2;
jnt3.PositionLimits=[-2.96706,1.91986]

body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
jnt4.PositionLimits=[-12.5664,12.5664]
%jnt4.HomePosition=-pi/4;

body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
jnt5.PositionLimits=[-2.0944,2.0944]
jnt5.HomePosition=0;

body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');
%jnt6.HomePosition=pi;
jnt6.PositionLimits=[-12.5664,12.5664]
body7 = robotics.RigidBody('body7');
jnt7 = robotics.Joint('jnt7','fixed');
%jnt6.HomePosition=pi;
%jnt6.PositionLimits=[-6.98132,6.98132]

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');
setFixedTransform(jnt7,dhparams(7,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;

addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')
addBody(robot,body7,'body6')

%%
showdetails(robot)

show(robot);
axis([-1.5,1.5,-1.5,1.5,-1.5,1.5])
axis on
%%
%robot=loadrobot('abbirb120','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig= randomConfiguration(robot);
numJoints=numel(currentRobotJConfig)
eeName="body7"
%%
%Waypoint creation
%waypoints = [-0.7 0 -1.50 ; -0.7 -0.2 -1.50 ; -0.7 -0.2 -1.60 ; -0.7 -0.2 -1.50 ;
%             -0.7 0 -1.50 ; -0.7 0.2 -1.50 ;  -0.7 0.2 -1.60 ; -0.7 0.2 -1.50 ;
%             -0.7 0 -1.50 ; -0.7 0 -1.50 ; -0.7 0 -1.60 ; -0.7 0 -1.50 ;
%             -0.7 0 -1.50 ; -0.7 0 -1.50 ; -0.7 0 -1.60 ; -0.7 0 -1.50 ;
%             -0.7 0 -1.50]';
%waypoints = [-0.58204 -0.050745 -0.665 ; -0.33494 0.478555 -0.665 ; 0.24676 0.529385 -0.665 ; 0.58204 0.050855 -0.665 ; 0.3349 -0.478485 -0.665 ;  -0.24676 -0.52985 -0.665 ;-0.58204 -0.050745 -0.665 ;]'; % 

waypoints = [-0.33494 0.478555 -0.6 ; -0.33494 0.478555 -0.665 ; -0.33494 0.478555 -0.6 ; 
    0.24676 0.529385 -0.6 ; 0.24676 0.529385 -0.665 ; 0.24676 0.529385 -0.6 ; 
    0.58204 0.050855 -0.6 ; 0.58204 0.050855 -0.665 ;0.58204 0.050855 -0.6 ;
    0.3349 -0.478485 -0.6 ; 0.3349 -0.478485 -0.665 ; 0.3349 -0.478485 -0.6 ;
    -0.24676 -0.52985 -0.6 ; -0.24676 -0.52985 -0.665 ;-0.24676 -0.52985 -0.6 ;]'; % 

%waypoints = [ -0.55643;0.0041120;-0.665;-0.33494 0.478555 -0.665 ; 0.24676 0.529385 -0.665 ; 0.58204 0.050855 -0.665 ; 0.3349 -0.478485 -0.665 ;  -0.24676 -0.52985 -0.665 ;]'; % 
%waypointTimes = 0:4:24;
%t=0.2;
waypointTimes = 0:116.8:1635.2;
%waypointTimes = 0:116.8:467.2;
t=5;
trajTimes=0:t:waypointTimes(end);
% Additional parameters

% Boundary conditions (for polynomial trajectories)
% Velocity (cubic and quintic)
waypointVels = 0 *[ 0  1  0;
                     -1  0  0;
                      0 -1  0;
                      1  0  0;
                      0  1  0;
                     -1  0  0;
                      0 -1  0;]'; %

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));

% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;
%%
ik = inverseKinematics('RigidBodyTree',robot);
ikWeights = [1 1 1 1 1 1];
ikInitGuess=robot.homeConfiguration;

show(robot,ikInitGuess,'Frames','off','PreservePlot',false);
xlim([-2 2]),ylim([-2 2]),zlim([-2 2])
hold on
hTraj=plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'*','LineWidth',0.5);

%% Generate trajectory
% Cartesian Motion only
trajType = 'trap';
switch trajType
    case 'trap'
        [q,qd,qdd] = trapveltraj(waypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[3 1]));
                            
    case 'cubic'
        [q,qd,qdd] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels);
        
    case 'quintic'
        [q,qd,qdd] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels, ...
            'AccelerationBoundaryCondition',waypointAccels);
        
    case 'bspline'
        ctrlpoints = waypoints; % Can adapt this as needed
        [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

% Show the full trajectory with the rigid body tree
% To visualize the trajectory, run the following line
% plotTrajectory(trajTimes,q,qd,qdd,'Names',["X","Y","Z"],'WaypointTimes',waypointTimes)

%%
angles_joint1=[];
angles_joint2=[];
angles_joint3=[];
angles_joint4=[];
angles_joint5=[];
angles_joint6=[];
iterations=[];
position_error=[];
%inverse kinematics
for idx = 1:numel(trajTimes)
    tgtPose = trvec2tform(q(:,idx)');
    tgtPose(1,1)=0;    tgtPose(1,2)=0;    tgtPose(1,3)=-1;
    tgtPose(2,1)=0;    tgtPose(2,2)=1;    tgtPose(2,3)=0;
    tgtPose(3,1)=1;    tgtPose(3,2)=0;    tgtPose(3,3)=0;
    [config,info]=ik(eeName,tgtPose,ikWeights,ikInitGuess);
    iterations(idx)=info.Iterations;
    position_error(idx)=info.PoseErrorNorm;
    angles_joint1(idx)=radtodeg(config(1).JointPosition);
    angles_joint2(idx)=radtodeg(config(2).JointPosition);
    angles_joint3(idx)=radtodeg(config(3).JointPosition);
    angles_joint4(idx)=radtodeg(config(4).JointPosition);
    angles_joint5(idx)=radtodeg(config(5).JointPosition);
    angles_joint6(idx)=radtodeg(config(6).JointPosition);
    anglesrad_joint1(idx)=(config(1).JointPosition);
    anglesrad_joint2(idx)=(config(2).JointPosition);
    anglesrad_joint3(idx)=(config(3).JointPosition);
    anglesrad_joint4(idx)=(config(4).JointPosition);
    anglesrad_joint5(idx)=(config(5).JointPosition);
    anglesrad_joint6(idx)=(config(6).JointPosition);
    ikInitGuess = config;
    plot3(q(1,idx),q(2,idx),q(3,idx),"b.-")
    %show the robot
    show(robot,config,'Frames','on','PreservePlot',false)
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow
end
   
