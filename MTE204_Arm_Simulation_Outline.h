//RELEVANT NOTATION FROM THE DOCUMENT THAT WE SHOULD USE IN CODE
//Information on the jacobian mathod can be found here: https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
{

{t} = target positions of the end effectors, we only have 1, so this is not a vector

{s}(theta) = current position of the end effector as a function of theta

{e} = {t} - {s} //the current desired change in position of the end effector 

theta = [ q1 : q2 : q3] //collumn vector of our joint angles

//when we reach the desires position   t - acceptableError <= s(theta) <= t + acceptable error 

J = jacobain matrix of our system


/*
i think it would be neat if we coded both methods and tested them against eachother for this mostly simple system.
I hypothesize that the approximation for the inverse method will be closer, but will also be slower because we have the calculate the inverse
and that the transpose method will be faster but will have more error.
*/


//------------INVERSE METHOD--------------------------------------------------------------------------------------------------------

DeltaTheta [ dq1 : dq2 : dq3] // the minute change we will be changing Theta by the is defined by the fellowing eqn

delta_S = J * deltaTheta //which implies
e = J * deltaTheta   //because we want our path delta_S to be about equal to e

//derived from the actual equation
s'(theta) = J(theta)*Theta_dot 

//^^^THIS IS WHERE OUR ERROR WILL COME FROM^^^

//---------------------------------------------------------------------------------------------------

//------------------TRANSPOSE METHOD----------------------------------------------------------------

//we define deltaTheta differently here
deltaTheta = a*J_tranpose*e

//where a is where the error will come from, and is deffined as

a = (e (dotprod) J*J_transpose*e) / ((J*J_transpose*e) (dotprod) (J*J_transpose*e))

//-------------------------------------------------------------------------------------------------
}

script Main program
{
	
	setup code
	-----------------------------------------------------------------------------------------------------------------------------------
	//RIGHT AT THE TOP HERE WE COULD HAVE THE JACOBIAN MATRIX FOR OUR FUNCTIONS ALREADY CALCULATED OR MAPPED OUT, I THINK AT THIS POINT 
	//IT IS APPROAPRIATE THAT JORDAN DOES THIS SINCE HE MADE THE EQUATIONS
	
	//we will also have a default position for the robot which we define at the begigning here
	
	
	//this should directly imput the path that we write manually
	//or we could think of a way to upload a path from a file or something???
	//this will be the main file that will use all of our functions to run the simmulation.
	
	Option A: tranpose method
	____________________________________________________________
	J_Transpose= function transpose( matrix J)   //with trig functions a tranpose just reqires us to switch elements around, so this we can do
	(Lower_matrix,Upper_Matrix) = function Upper_Lower(J_Transpose) //somehow this will have to solve the system with trig identities, I DO NOT KNOW HOW TO DO THIS
	//if we do LU decomposition, our J matrix may need pivoting, so we will likely have to code LU decomposition with partial pivoting, or we do this by hand once and ignore it???
	_
	___________________________________________________________
	
	
	Option B: Inverse Method
	______________________________________________________________
	//NOTE!!! not sure how to solve matrices with trigonometric functions in them
	J_inverse = function inverse( matrix J)   //I DO NOT KNOW IF WE ARE EXPECTED TO CODE AN INVERSE FUNCTION OURSELVES, OTHERWSIE WE ONLY NEED TO USE IT ONCE BECAUSE OUR ARM SYSTEM DOESNT CHANGE
	(Lower_matrix,Upper_Matrix) = function Upper_Lower(J_Inverse) //somehow this will have to solve the system with trig identities, I DO NOT KNOW HOW TO DO THIS
	
	
	______________________________________________________________
	
	-----------------------------------------------------------------------------------------------------------------------------------
	
	list_of_points[] Optimal_Pathing(path,curve,function I dont know)
	{
		//this could somehow analyze our input curve and then optimize each specific point we will be solving for
		//since the jacobian method could get wonky if we want curves over a large distance
		
		//we will be using the notation for variables from the Samuel R.Buss document.
		//thus the list of points will represent by 't' values
	}
	
	//OPTION A: the jacobian transpose
	
	
	while(there are still target points)
	{
		
		while(there are still points in the list of the curve we run the jacobian method)
		{      

			while( s(theta) is not close enough to the target)
			{
				a = (e (dotprod) J*J_transpose*e) / ((J*J_transpose*e) (dotprod) (J*J_transpose*e))
			
				delta_theta = LU_decomposition(a, Lower_matrix, Upper_matrix, e		//again not sure how to do this with trig functions, we also have to calculate a new a after every iteration
				{
					//this is hypothetic LU-decomp code
					//there going to have to be something in here that will be frther approximating our answer because of the trig functions
				}
				
				//use delta_theta to iterate our current end effector position as well as the joint angles
				//calculate a new e
			
				//plots the points or records the data to be analzyed later
			}
		
	
	
	}
		
			
	}
	
	//OPTION B: the jacobian inverse"
	
	while(there are still points in the list of the curve we run the jacobian method)
	{      

		while( s(theta) is not close enough to the target)
		{
			//e = J * deltaTheta  ==> [J_inverse]{e} = delta_theta //solve for delta theta using this formula.
			//this implies using something like this, currently form what we know LU decomposition seems nice because J inv doesnt change
		
			delta_theta = LU_decomposition(Lower_matrix, Upper_matrix, e);		//again not sure how to do this with trig functions
			{
				//this is hypothetical LU-decomposition code
				//there going to have to be something in here that will be frther approximating our answer because of the trig functions
			}
			
			//use delta_theta to iterate our current end effector position as well as the joint angles
			//calculate a new e 
		
			//plots the points or records the data to be analzyed later
		}
		
	
	
	}
}
