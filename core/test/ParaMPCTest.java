package core.test;

import core.model.Constants;
import core.model.Dynamics;
import core.util.DMPC;
import core.util.Sontag;

public class ParaMPCTest {
	public static void main(String[] args){
		/* Set up initial state and final state */
		Constants cons = new Constants();
		double[] xset = new double[Constants.ODE], xk = new double[Constants.ODE];
		xk = cons.getInitPoint(); // set point
		xset = cons.getSetPoint(); // initial point

		/* Set up simulation environment */
		int cc = 119; // simulation time
		double[] uk = new double[5], uk_f = new double[2], t = new double[cc+1];
		double[][] x = new double[cc+1][Constants.ODE], u = new double[cc+1][7];
		Dynamics dyn = new Dynamics();
		Sontag son = new Sontag(xset);

		/* Set up MPC */
		int nc = 60, N = 2, ite = 1;
		double[] Qc = Constants.Qc, Rc = Constants.Rc, Rc2 = Constants.Rc2;
		double[][] u_prev = new double[N][7];

		/* Set up Ipopt */
		int nn1 = 2, nn2 = 3, nn3 = 2, n1 = nn1*N, n2 = nn2*N, n3 = nn3*N, m = 1;
		double[] u1_L = new double[n1], u1_U = new double[n1], u2_L = new double[n2], u2_U = new double[n2],
				 u3_L = new double[n3], u3_U = new double[n3];
		double[] g_L = new double[m], g_U = new double[m], u1_opt = new double[n1], u2_opt = new double[n2],
				 u3_opt = new double[n3];
		for(int i=0; i<N; i++){
			u1_L[i*nn1 + 0] = -Constants.umax[0]; u1_U[i*nn1 + 0] = Constants.umax[0];
			u1_L[i*nn1 + 1] = -Constants.umax[1]; u1_U[i*nn1 + 1] = Constants.umax[1];
		}
		for(int i=0; i<N; i++){
			u2_L[i*nn2 + 0] = -Constants.umax[2]; u2_U[i*nn2 + 0] = Constants.umax[2];
			u2_L[i*nn2 + 1] = -Constants.umax[3]; u2_U[i*nn2 + 1] = Constants.umax[3];
			u2_L[i*nn2 + 2] = -Constants.umax[4]; u2_U[i*nn2 + 2] = Constants.umax[4];
		}
		for(int i=0; i<N; i++){
			u3_L[i*nn3 + 0] = -Constants.umax[5]; u3_U[i*nn3 + 0] = Constants.umax[5];
			u3_L[i*nn3 + 1] = -Constants.umax[6]; u3_U[i*nn3 + 1] = Constants.umax[6];
		}
		for(int i=0; i<m; i++){
			g_L[i] = -1e11;
			g_U[i] = 1e11;
		}
		DMPC mpc1 = new DMPC(n1, m, u1_L, u1_U, g_L, g_U); // create IPOPT
		DMPC mpc2 = new DMPC(n2, m, u2_L, u2_U, g_L, g_U); // create IPOPT
		DMPC mpc3 = new DMPC(n3, m, u3_L, u3_U, g_L, g_U); // create IPOPT
		mpc1.setMPCID(1); mpc2.setMPCID(2); mpc3.setMPCID(3);
		mpc1.addIntOption("max_iter",400); mpc1.addIntOption("print_level",-2); mpc1.addStrOption("hessian_approximation", "limited-memory");
		mpc1.addNumOption("tol", 1e-5); mpc1.addNumOption("dual_inf_tol", 1e-5); mpc1.addNumOption("acceptable_tol", 1e-5);
		mpc2.addIntOption("max_iter",1000); mpc2.addIntOption("print_level",-2); mpc2.addStrOption("hessian_approximation", "limited-memory");
		mpc2.addNumOption("tol", 1e-5); mpc2.addNumOption("dual_inf_tol", 1e-5); mpc2.addNumOption("acceptable_tol", 1e-5);
		mpc3.addIntOption("max_iter",400); mpc3.addIntOption("print_level",-2); mpc3.addStrOption("hessian_approximation", "limited-memory");
		mpc3.addNumOption("tol", 1e-5); mpc3.addNumOption("dual_inf_tol", 1e-5); mpc3.addNumOption("acceptable_tol", 1e-5);
		mpc1.addStrOption("nlp_scaling_method", "none");
		mpc2.addStrOption("nlp_scaling_method", "none");
		mpc3.addStrOption("nlp_scaling_method", "none");

		mpc1.setNc(nc); mpc1.setXset(xset);
		mpc2.setNc(nc); mpc2.setXset(xset);
		mpc3.setNc(nc); mpc3.setXset(xset);
		mpc1.setXk(xk); mpc2.setXk(xk); mpc3.setXk(xk);

		for(int i=0; i<Constants.ODE; i++){
			x[0][i] = xk[i];
		}
		t[0] = 0;

		try{
			double[][] uk_prev = son.predict(xk, N, nc);
			for(int l=0; l<N; l++){
				for(int k=2; k<7; k++){
					u_prev[l][k] = uk_prev[l][k-2];
				}
			}
			mpc1.setUPrev(u_prev); mpc1.setXk(xk);
			mpc2.setUPrev(u_prev); mpc2.setXk(xk);
			mpc3.setUPrev(u_prev); mpc3.setXk(xk);
			for(int p=0; p<ite; p++)
			{
				mpc1.setURef(u_prev[0]); mpc2.setURef(u_prev[0]); mpc3.setURef(u_prev[0]);
				double[] u10 = new double[nn1*N];
				for(int s=0; s<N; s++){
					u10[s*nn1 + 0] = u_prev[s][0];
					u10[s*nn1 + 1] = u_prev[s][1];
				}
				if(p==0)
					System.out.print("At " + 0 + "\t ite #"+p+": status of MPC 1 is ");
				else
					System.out.print("   \t ite #"+p+": status of MPC 1 is ");
				u1_opt = mpc1.solve(u10);
				System.out.print(""+mpc1.getStatus()+"\t");
				double[] u20 = new double[nn2*N];
				for(int s=0; s<N; s++){
					u20[s*nn2 + 0] = u_prev[s][2];
					u20[s*nn2 + 1] = u_prev[s][3];
					u20[s*nn2 + 2] = u_prev[s][4];
				}
				System.out.print("status of MPC 2 is ");
				u2_opt = mpc2.solve(u20);
				System.out.print(""+mpc2.getStatus()+"\t");
				double[] u30 = new double[nn3*N];
				for(int s=0; s<N; s++){
					u30[s*nn3 + 0] = u_prev[s][5];
					u30[s*nn3 + 1] = u_prev[s][6];
				}
				System.out.print("status of MPC 3 is ");
				u3_opt = mpc3.solve(u30);
				System.out.println(""+mpc3.getStatus());
			}
			int count = 0;
			for(int i=0; i<cc; i++){
				if(i%nc == 0)
				{
					for(int l=0; l<nn1; l++){
						uk_f[l] = u1_opt[nn1*count + l];
					}
					for(int l=0; l<nn2; l++){
						uk[l] = u2_opt[nn2*count + l];
					}
					for(int l=nn2; l<5; l++){
						uk[l] = u3_opt[nn3*count + l-nn2];
					}
					count += 1;
				}else{
					for(int k=0; k<2; k++){
						uk_f[k] = u[i-1][k];
					}
					for(int k=0; k<5; k++){
						uk[k] = u[i-1][k+2];
					}
				}
				for(int k=0; k<2; k++){
					u[i][k] = uk_f[k];
				}
				for(int k=0; k<5; k++){
					u[i][k+2] = uk[k];
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
					cost =  cost + u[i][j+2]*Rc[j]*u[i][j+2];
				}
			}
			System.out.println("Total cost is : "+cost);

		}
		catch(Exception e){
			e.printStackTrace();
		}
	}
}
