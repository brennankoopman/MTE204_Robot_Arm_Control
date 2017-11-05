# MTE204_Robot_Arm_Control
This is a numerical methods project, testing both the Jacobian transpose and Inverse Method as they apply to robot arm pathing and control

The Major organization of the following files is as follows (11/5/2017)

--------------------------------------------------
Code implementing the Jacobian transpose method:
--------------------------------------------------
Jacoiban.m 		~ calcualte Jac of the robot system
getQ.m			~ implements the jacobian Transpose method, and returns the joint angles for the target position
armFunction.m		~ calculates the position of the end effector based on inputted the current joint angles, used to evaluate error
armFunction_midJjoint.m	~ calculates the position of the midjoint of the arm based on q1 and q3

armSim.m 		~ just a test
MoveToTarget.m		~ used to evaluate the convergence of the transpose method and visualize the arm before the simulation

--------------------------------------------------
Code Implementing linear pathing
--------------------------------------------------
armLinMove.m		~almost identical to sectionPath, but has one les parameter, used to section a linear path into a list of subTargets on that path
armLinMoveScript.m	~uses armLinMove.m and outputs a graph of the true and actual paths of the robot


--------------------------------------------------
Analysis of Jacobian tranpose method
--------------------------------------------------
sectionPath.m 		~almsot identical to armLinMove, but sections based a number of subintervals, instead of a fixed length
ExTimeVsSubintervals.m	~similair to linArmMoveScript but doesnt have code for graphoutputs, and visualization to accurately evaluate the time complexity of the algorithm

--------------------------------------------------
Final applicable Function
--------------------------------------------------
