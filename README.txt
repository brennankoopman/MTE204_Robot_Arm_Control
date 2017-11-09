# MTE204_Robot_Arm_Control
This is a numerical methods project, testing both the Jacobian transpose Method as they apply to robot arm pathing and control

The Major organization of the following files is as follows (11/9/2017)

------------------------------------------------------------------
Code implementing the Jacobian transpose method and model of robot:
-------------------------------------------------------------------
Jacoiban.m 		~ calcualte Jac of the robot system
getQ.m			~ implements the jacobian Transpose method, and returns the joint angles for the target position
armFunction.m		~ calculates the position of the end effector based on inputted the current joint angles, used to evaluate error
armFunction_midJjoint.m	~ calculates the position of the midjoint of the arm based on q1 q2 q3
JacobianConvergence.m	~ prints out the end effector at each iteration of the tranpose method, to test the convergence
JacobianConergenceSim.m	~ simulation of the above, looks nicer, and can see the continuous effect 


--------------------------------------------------
Code Implementing linear pathing
--------------------------------------------------
Pathing.m		~ Plots the linear path, and conpares it to the linearization of the joint angles, resulting in arcs
PathingSim.m		~ Simmulates the above but ALSO has user input to find the optimal number of subintervals based on our analysis
sectionPath.m 		~ section a given linear path into n subintervals and returns all the target points


--------------------------------------------------
Analysis of time computation
--------------------------------------------------
ExTimeVsSubintervals.m	~ uses timoutputtest.m to calcuate the time for each subinterval value.
TimeOutputTest.m	~ similair to Pathing but doesnt have code for graph outputs, and visualization to accurately evaluate the time complexity of the algorithm 


--------------------------------------------------
Final most applicable Function
--------------------------------------------------
PathingSim.m
