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
aik = analyticalInverseKinematics(robot);
showdetails(aik)
generateIKFunction(aik,'robotIK');

eePosition = [0 0.5 0.5];
eePose = trvec2tform(eePosition);
hold on
plotTransforms(eePosition,tform2quat(eePose))
hold off

ikConfig = robotIK(eePose); % Uses the generated file

show(robot,ikConfig(1,:));
hold on
plotTransforms(eePosition,tform2quat(eePose))
hold off

figure;
numSolutions = size(ikConfig,1);

for i = 1:size(ikConfig,1)
    subplot(1,numSolutions,i)
    show(robot,ikConfig(i,:));
end