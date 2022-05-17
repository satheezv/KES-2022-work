clc
clear all

syms L1 L2 L3 L4 L5 L6 Q1 Q2 Q3 Q4 Q5 Q6


dhparams = [0   	-pi/2    -.29   0;
            -.270   0       0       0;
            -.070   pi/2    0       0;
            0       -pi/2	0.302   0;
            0       pi/2	0       0;
            0       0       0.144   0;
            -0.072  0       0       0];

        
alphaa=[-90,0,90,-90,90, 0, 0]; % this is the alpha value for all  the link
a=[0,-.270, -.070, 0, 0, 0, -0.072]; % Length of the Link
d=[-.290,0,0,.302,0, .144], 0; %Offset
Q=[Q1,Q2,Q3,Q4,Q5,Q6, 0 ]; % joint angle variation
init_guess=[0,-pi/2,0,0,-pi/2,0, 0]' % home position
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
%Jacobian
f1=T06(1,4)
f2=T06(2,4)
f3=T06(3,4)
f4=((T06(1,1).^2)+(T06(2,1).^2)+(T06(3,1).^2))
f5=((T06(1,2).^2)+(T06(2,2).^2)+(T06(3,2).^2))
f6=((T06(1,3).^2)+(T06(2,3).^2)+(T06(3,3).^2))



f1_upd=([subs(f1,{Q1,Q2,Q3,Q4,Q5,Q6},{init_guess(1),init_guess(2),init_guess(3),init_guess(4),init_guess(5),init_guess(6)})])
f2_upd=([subs(f2,{Q1,Q2,Q3,Q4,Q5,Q6},{init_guess(1),init_guess(2),init_guess(3),init_guess(4),init_guess(5),init_guess(6)})])
f3_upd=([subs(f3,{Q1,Q2,Q3,Q4,Q5,Q6},{init_guess(1),init_guess(2),init_guess(3),init_guess(4),init_guess(5),init_guess(6)})])
f4_upd=([subs(f4,{Q1,Q2,Q3,Q4,Q5,Q6},{init_guess(1),init_guess(2),init_guess(3),init_guess(4),init_guess(5),init_guess(6)})])
f5_upd=([subs(f5,{Q1,Q2,Q3,Q4,Q5,Q6},{init_guess(1),init_guess(2),init_guess(3),init_guess(4),init_guess(5),init_guess(6)})])
f6_upd=([subs(f6,{Q1,Q2,Q3,Q4,Q5,Q6},{init_guess(1),init_guess(2),init_guess(3),init_guess(4),init_guess(5),init_guess(6)})])



jac=[diff(f1,Q1) diff(f1,Q2) diff(f1,Q3) diff(f1,Q4) diff(f1,Q5) diff(f1,Q6);
     diff(f2,Q1) diff(f2,Q2) diff(f2,Q3) diff(f2,Q4) diff(f2,Q5) diff(f2,Q6);
     diff(f3,Q1) diff(f3,Q2) diff(f3,Q3) diff(f3,Q4) diff(f3,Q5) diff(f3,Q6);
     diff(f4,Q1) diff(f4,Q2) diff(f4,Q3) diff(f4,Q4) diff(f4,Q5) diff(f4,Q6);
     diff(f5,Q1) diff(f5,Q2) diff(f5,Q3) diff(f5,Q4) diff(f5,Q5) diff(f5,Q6);
     diff(f6,Q1) diff(f6,Q2) diff(f6,Q3) diff(f6,Q4) diff(f6,Q5) diff(f6,Q6);]
 
jac_upd=([subs(jac,{Q1,Q2,Q3,Q4,Q5,Q6},{init_guess(1),init_guess(2),init_guess(3),init_guess(4),init_guess(5),init_guess(6)})]);
        
%%
iteration=0
Q_updated=init_guess
while 1
    
    f_pos=[f1_upd, f2_upd, f3_upd, f4_upd, f5_upd, f6_upd]'
    jac_pseudo_inv=pinv(jac_upd)
    e=abs([-303, 0, -703, 1, 1, 1]' - f_pos)

    if (e(1)<0.1 & e(2)<0.1 & e(3)<0.1)
        Q_final = Q_updated
        break
    else
        Q_updated=vpa(Q_updated+jac_pseudo_inv*e)
        f1_upd=([subs(f1,{Q1,Q2,Q3,Q4,Q5,Q6},{Q_updated(1),Q_updated(2),Q_updated(3),Q_updated(4),Q_updated(5),Q_updated(6)})])
        f2_upd=([subs(f2,{Q1,Q2,Q3,Q4,Q5,Q6},{Q_updated(1),Q_updated(2),Q_updated(3),Q_updated(4),Q_updated(5),Q_updated(6)})])
        f3_upd=([subs(f3,{Q1,Q2,Q3,Q4,Q5,Q6},{Q_updated(1),Q_updated(2),Q_updated(3),Q_updated(4),Q_updated(5),Q_updated(6)})])
        f4_upd=([subs(f4,{Q1,Q2,Q3,Q4,Q5,Q6},{Q_updated(1),Q_updated(2),Q_updated(3),Q_updated(4),Q_updated(5),Q_updated(6)})])
        f5_upd=([subs(f5,{Q1,Q2,Q3,Q4,Q5,Q6},{Q_updated(1),Q_updated(2),Q_updated(3),Q_updated(4),Q_updated(5),Q_updated(6)})])
        f6_upd=([subs(f6,{Q1,Q2,Q3,Q4,Q5,Q6},{Q_updated(1),Q_updated(2),Q_updated(3),Q_updated(4),Q_updated(5),Q_updated(6)})])
        
        jac_upd=([subs(jac,{Q1,Q2,Q3,Q4,Q5,Q6},{Q_updated(1),Q_updated(2),Q_updated(3),Q_updated(4),Q_updated(5),Q_updated(6)})])
    end    
    iteration=iteration+1
end
%%
robot = robotics.RigidBodyTree;
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base')

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
jnt2.HomePosition=-pi/2;
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
jnt3.HomePosition=0;
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
jnt5.HomePosition=-pi/2;
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

%%
showdetails(robot)

show(robot);
axis([-1,1,-1,1,-1,1])
axis on