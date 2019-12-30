# MPC
MPC Project

Help me I m not gonna survive all of this...

For the moment I took "inspiration" (that's how to you politely say that you copy pasted) from exercice 4 and 3 to fill up the function "function ctrl_opt = setup_controller(mpc)". Now it seems to work, the invariant set seems to be OK, the input signal have the right sign and seems to change sign as expected. Unfortunately if you use u_x = mpc_x.get_u([0,0,0,2]'); The control signal is infeasible and so the control cast an error. by doing u_x = mpc_x.get_u([0,0,0,2]'); I think I'm asking for the control signal for the x directio when the drone is 2 meters far from the center in the x direction. To verify that you can see what is in the mpc_x variable.

Jerome Update (12/30/2019) : 
I found one error in the code (corrected now)
I started 3.2, which is very similar to exercice 5 (so I will look again at the exercice to better understand what must be done exactly)
Fow now I have an error : file MPC_control_x.m bc matlab can't solve the lyaponov equation P = lyap(mpc.A, Q)
                          so either I'm doing bad sth before, either we must change our Q
