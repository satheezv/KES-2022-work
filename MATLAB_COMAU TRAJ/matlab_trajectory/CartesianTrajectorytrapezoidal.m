clc
clear all

dhparams = [0   	pi/2       -.29   0;
            0       pi/2   -0.125   0;
            -.270   0       0       0;
            -.070   pi/2	0       0;
            0       pi/2	0.302   0;
            0      -pi/2      0     0];

        
robot = robotics.RigidBodyTree;
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base')

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')


showdetails(robot)

show(robot);
%axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
axis on

%%
%robot=loadrobot('abbirb120','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig= randomConfiguration(robot);
numJoints=numel(currentRobotJConfig)
eeName="body6"
%%
%Waypoint creation
%waypoints = [0.374,0.008131,-0.6365]' + ... 
 waypoints = [-0.4 0 -0.6365 ; -0.4 0.2 -0.6365 ; -0.4 0 -0.6365 ; -0.4 -0.2 -0.6365 ; -0.4 0 -0.6365]';
waypointTimes = 0:4:16;
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
ikWeights = [1 1 1 1 1 1];
ikInitGuess=robot.homeConfiguration;

show(robot,ikInitGuess,'Frames','off','PreservePlot',false);
xlim([-1 1]),ylim([-1 1]),zlim([-1 1.2])
hold on
hTraj=plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-')
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Generate trajectory
% Cartesian Motion only
trajType = 'cubic';
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
%inverse kinematics
for i = 1:5
    for idx = 1:numel(trajTimes)
        tgtPose = trvec2tform(q(:,idx)');
        [config,info]=ik(eeName,tgtPose,ikWeights,ikInitGuess);
        ikInitGuess = config;
        plot3(q(1,idx),q(2,idx),q(3,idx),"b.-")
        %show the robot
        show(robot,config,'Frames','on','PreservePlot',false)
        title(['Trajectory at t = ' num2str(trajTimes(idx))])
        drawnow
    end
    i=i+1
end


    


%%
%Simulation/plotting