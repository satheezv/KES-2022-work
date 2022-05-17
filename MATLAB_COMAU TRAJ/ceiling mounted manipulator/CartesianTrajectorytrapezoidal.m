clc
clear all




%dhparams = [0   	0       -.165   0;
%            0       pi/2   -0.125   0;
%            -.270   0       0       0;
%            -.070   pi/2	0       0;
%            0       pi/2	0.302   0;
%            0      -pi/2      0     0];
dhparams = [0   	-pi/2    -.29   0;
            -.270   0       0       0;
            -.070   pi/2    0       0;
            0       -pi/2	0.302   0;
            0       pi/2	0       0;
            0       0       0.144   0;
            -0.072  0       0       0];

        
robot = robotics.RigidBodyTree;
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
jnt1.HomePosition=0;
jnt1.PositionLimits=[-2.87979,2.87979]
setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base')

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
jnt2.HomePosition=-pi/2;
jnt2.PositionLimits=[-1.91986,1.91986]

body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
jnt3.HomePosition=0;
jnt3.PositionLimits=[-1.91986,1.22173]

body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
jnt4.PositionLimits=[0,2.79253]
%jnt4.HomePosition=-pi/4;

body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
jnt5.PositionLimits=[-2.0944,2.0944]
jnt5.HomePosition=0;

body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');
%jnt6.HomePosition=pi;
jnt6.PositionLimits=[-6.98132,6.98132]

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
%axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
axis on

%%
%robot=loadrobot('abbirb120','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig= randomConfiguration(robot);
numJoints=numel(currentRobotJConfig)
eeName="body7"
%%
%Waypoint creation
%waypoints = [0.374,0.008131,-0.6365]' + ... 
%waypoints = [-0.3 0 -0.702 0 -1.5708 0;...
%             -0.3 -0.1 -0.702 0 -1.5708 0;...
%             -0.3 0 -0.702 0 -1.5708 0;...
%             -0.3 0.1 -0.702 0 -1.5708 0; ...
%             -0.3 0 -0.702 0 -1.5708 0]';
waypoints = [-0.3 0 -0.68 ; -0.3 -0.1 -0.68 ; -0.3 0 -0.68 ; -0.3 0.1 -0.68 ; -0.3 0 -0.68 ; -0.4 0 -0.68 ; -0.4 0 -0.68 ; -0.2 0 -0.68 ; -0.3 0 -0.68]';
%waypoints = [-0.3 0 -0.78 ; -0.3 -0.1 -0.78 ; -0.3 0 -0.78 ; -0.3 0.1 -0.78 ; -0.3 0 -0.78 ;  -0.3 -0.2 -0.78 ; -0.3 0 -0.78 ; -0.3 0.2 -0.78 ; -0.3 0 -0.78 ;]';
waypointTimes = 0:4:32;
t=0.2;
trajTimes=0:t:waypointTimes(end);
% Additional parameters

% Boundary conditions (for polynomial trajectories)
% Velocity (cubic and quintic)
waypointVels = 0 *[ 0  1  0;
                     -1  0  0;
                      0 -1  0;
                      1  0  0;
                      0  1  0]';

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));

% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;
%%
ik = inverseKinematics('RigidBodyTree',robot);
ikWeights = [1 1 1 0.25 0.25 0.25];
ikInitGuess=robot.homeConfiguration;

show(robot,ikInitGuess,'Frames','off','PreservePlot',false);
xlim([-1 1]),ylim([-1 1]),zlim([-1 1.2])
hold on
hTraj=plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-')
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

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
angles=[]
%inverse kinematics
for idx = 1:numel(trajTimes)
    
    tgtPose = trvec2tform(q(:,idx)');
    tgtPose(1,1)=0
    tgtPose(1,2)=0
    tgtPose(1,3)=-1
    tgtPose(2,1)=0
    tgtPose(2,2)=1
    tgtPose(2,3)=0
    tgtPose(3,1)=1
    tgtPose(3,2)=0
    tgtPose(3,3)=0
    [config,info]=ik(eeName,tgtPose,ikWeights,ikInitGuess);
    angles(idx)=config.JointPosition
    ikInitGuess = config;
    plot3(q(1,idx),q(2,idx),q(3,idx),"b.-")
    %show the robot
    show(robot,config,'Frames','on','PreservePlot',false)
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow
end
   


    


%%
%Simulation/plotting