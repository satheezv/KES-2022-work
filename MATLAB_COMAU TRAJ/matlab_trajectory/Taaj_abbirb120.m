rbtree = robotics.RigidBodyTree;
body1 = robotics.RigidBody('b1');
jnt1 = robotics.Joint('jnt1','revolute');
body1.Joint = jnt1;
basename = rbtree.BaseName;
addBody(rbtree,body1,basename)
showdetails(rbtree)