package core.test;

import core.model.Constants;
import core.model.Dynamics;
import core.util.MPC;

public class CentrlMPCTest {
		public static void main(String[] args){
			/* Set up initial state and final state */
			Constants cons = new Constants();
			double[] xset = new double[Constants.ODE], xk = new double[Constants.ODE];
			xset = cons.getSetPoint(); // set point
			xk = cons.getInitPoint(); // initial point

			/* Set up simulation environment */
			int cc = 119; // simulation time
			double[] uk = new double[5], uk_f = new double[2], t = new double[cc+1]; 
			double[][] x = new double[cc+1][Constants.ODE], u = new double[cc+1][7];
			Dynamics dyn = new Dynamics();
			
			/* Set up MPC */
			int nc = 60, N = 2;
			double[] Qc = Constants.Qc, Rc = Constants.Rc, Rc2 = Constants.Rc2;  
			
			// initial guess of inputs
			double[] u0 = new double[7*N];
			for(int i=0; i<N; i++){
				u0[0 + i*7] = 0; u0[1 + i*7] = 0;
				u0[2 + i*7] = 5e5; u0[3 + i*7] = 2e5; u0[4 + i*7] = 1e5;
				u0[5 + i*7] = 5e5; u0[6 + i*7] = 5e5;
			}
			int n = 7*N, m = 1;           // horizon n/5, constraints m one step constraint
			double[] u_L = new double[n]; // input constraint vector, lower bounds
			double[] u_U = new double[n]; // input constraint vector, upper bounds
			double[] g_L = new double[m]; // state constraint vector, lower bounds
			double[] g_U = new double[m]; // state constraint vector, upper bounds
			double[] u_opt = new double[n]; // optimal inputs returned by IPOPT
			// set u_L and u_U
			for(int i=0; i<n/7; i++){
				u_L[i*7 + 0] = -Constants.umax[0];
				u_U[i*7 + 0] = Constants.umax[0];
				u_L[i*7 + 1] = -Constants.umax[1];
				u_U[i*7 + 1] = Constants.umax[1];
				u_L[i*7 + 2] = -Constants.umax[2];
				u_U[i*7 + 2] = Constants.umax[2];
				u_L[i*7 + 3] = -Constants.umax[3];
				u_U[i*7 + 3] = Constants.umax[3];
				u_L[i*7 + 4] = -Constants.umax[4];
				u_U[i*7 + 4] = Constants.umax[4];
				u_L[i*7 + 5] = -Constants.umax[5];
				u_U[i*7 + 5] = Constants.umax[5];
				u_L[i*7 + 6] = -Constants.umax[6];
				u_U[i*7 + 6] = Constants.umax[6];
			}
			// set g_L and g_U
			for(int i=0; i<1; i++){
				g_L[i] = -1e11;
				g_U[i] = 1e11;
			}
			MPC mpc = new MPC(n, m, u_L, u_U, g_L, g_U); // create IPOPT
			mpc.addIntOption("print_level",-2); // do not print on screen
			mpc.addIntOption("max_iter",10000);   // maximum iteration number
			mpc.addStrOption("hessian_approximation", "limited-memory");
			mpc.addNumOption("tol", 1e-5); mpc.addNumOption("dual_inf_tol", 1e-5); 
			mpc.addNumOption("acceptable_tol", 1e-5);
			mpc.addStrOption("nlp_scaling_method", "none");
			mpc.setNc(nc); // set sampling time
			mpc.setXset(xset); // set set-point
			mpc.setXk(xk);
			
			for(int i=0; i<Constants.ODE; i++){
				x[0][i] = xk[i];
			}
			t[0] = 0;

			try{				
				System.out.print("Status at "+0 +" is ");
				u_opt = mpc.solve(u0);
				System.out.println(""+mpc.getStatus());
				int count = 0;
				for(int i=0; i<cc; i++){
					if(i%nc == 0){
						for(int l=0; l<5; l++){
							uk[l] = u_opt[count*7 + l+2];
						}
						for(int l=0; l<2; l++){
							uk_f[l] = u_opt[count*7 + l];
						}
						count += 1;
					}else{
						for(int k=0; k<5; k++){
							uk[k] = u[i-1][k+2];
						}
						for(int k=0; k<2; k++){
							uk_f[k] = u[i-1][k];
						}
					}
					for(int k=0; k<5; k++){
						u[i][k+2] = uk[k];
					}
					for(int k=0; k<2; k++){
						u[i][k] = uk_f[k];
					}
					
					xk = dyn.solve(xk, uk, uk_f);
					for(int k=0; k<Constants.ODE; k++){
						x[i+1][k] = xk[k];
					}
					t[i+1] = (i+1)*Constants.h;
				}
				for(int i=0; i<7; i++){
					u[cc][i] = u[cc-1][i];
				}
				double cost = 0;
				for(int i=0; i<x.length; i+=nc){
					for(int j=0; j<Constants.ODE; j++){
						cost = cost + (x[i][j] - xset[j])*Qc[j]*(x[i][j] - xset[j]);
					}
					for(int j=0; j<2; j++){
						cost =  cost + u[i][j]*Rc2[j]*u[i][j];
					}
					for(int j=0; j<5; j++){
						cost =  cost + (u[i][j+2])*Rc[j]*(u[i][j+2]);
					}
				}
				System.out.println("Total cost is : "+cost);
			}
			catch(Exception e){
				e.printStackTrace();
			}
		}
}
