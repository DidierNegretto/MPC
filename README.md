# MPC
MPC Project

Help me I m not gonna survive all of this...

For the moment I took "inspiration" (that's how to you politely say that you copy pasted) from exercice 4 and 3 to fill up the function "function ctrl_opt = setup_controller(mpc)". Now it seems to work, the invariant set seems to be OK, the input signal have the right sign and seems to change sign as expected. Unfortunately if you use u_x = mpc_x.get_u([0,0,0,2]'); The control signal is infeasible and so the control cast an error. by doing u_x = mpc_x.get_u([0,0,0,2]'); I think I'm asking for the control signal for the x directio when the drone is 2 meters far from the center in the x direction. To verify that you can see what is in the mpc_x variable.

Jerome Update (12/30/2019) : 
I found one error in the code (corrected now)
I started 3.2, which is very similar to exercice 5 (so I will look again at the exercice to better understand what must be done exactly)
Fow now I have an error : file MPC_control_x.m bc matlab can't solve the lyaponov equation P = dlyap(mpc.A, Q)
                          so either I'm doing bad sth before, either we must change our Q

DIDIER update (2/1/2020) :
The  dlyap function in matlab solves: A*X*A' - X + Q = 0, while if you read the slides on constrained system (one of the last ones) the Lyapunov equation we need to solve is: A'*X*A - X = -Q, with Q > 0.
So in theory we would need to do dlyap(mpc.A', Q) and not dlyap(mpc.A, Q).
One disadvantage of the previously mentionned equation is that the the space of the avaible controllers in much limited, this might cause matlab to not find a solution, but maybe not. 

Now reading the slides on practical MPC (slide 7) it seems we can use a different equation the Ricatti equation.

X = Q + A'*X*A - A'*X*B*(R+B'*X*B)^-1*B'*X*A  (Ricatti)

Fortunately for us MATLAB comes with a function to solve:

E'XE = A'XA - (A'XB + S)(B'XB + R)^-1*(A'XB + S)' + Q
[X,K,L] = idare(A,B,Q,R,S,E)

If E is not defined it is set to identity matrix, which that we get the solutio to:

X = A'XA - (A'XB + S)(B'XB + R)^-1*(A'XB + S)' + Q

If S is not defined it is set to 0, which means that we get the solution to:

X = Q + A'XA - (A'XB)( R + B'XB )^-1*(A'XB)'

After reading again what the setup_steady_state_target(mpc) does it seems that the compute_sp() in the solution of exercice 5 is the way to take !
I have therefore changed the funciton and seems to be fine.