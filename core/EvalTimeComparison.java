package core;

import java.util.Date;
import core.model.Constants;
import core.util.DMPC;
import core.util.MPC;
import core.util.Sontag;

public class EvalTimeComparison {
	public static void main(String[] args){
		EvalTimeComparison eval = new EvalTimeComparison();
		//eval.run('s');
		eval.run('c');
	}
	
	public void run(char ccc){
		char MPC_Test_ID = ccc;
		int tt = 100;
		int nc = 30, N = 10;

		
		if(MPC_Test_ID == 'c'){
			Constants cons = new Constants();
			double[] xset = new double[Constants.ODE], xk = new double[Constants.ODE];
			xset = cons.getSetPoint(); // set point
			double[] u0 = new double[7*N];
			int n = 7*N, m = 1;           // horizon n/5, constraints m one step constraint
			double[] u_L = new double[n]; // input constraint vector, lower bounds
			double[] u_U = new double[n]; // input constraint vector, upper bounds
			double[] g_L = new double[m]; // state constraint vector, lower bounds
			double[] g_U = new double[m]; // state constraint vector, upper bounds
			for(int i=0; i<n/7; i++){
				for(int j=0; j<7; j++){
					u_L[i*7 + j] = -1;
					u_U[i*7 + j] = 1;
				}
			}
			for(int i=0; i<1; i++){
				g_L[i] = -1e11;
				g_U[i] = 1e-3;
			}
			double step = 1e-4;
			MPC mpc = new MPC(n, m, u_L, u_U, g_L, g_U); // create IPOPT
			mpc.setStep(step);
			step = 1e-4;
			mpc.addIntOption("print_level",-2); // do not print on screen
			mpc.addIntOption("max_iter",2000);   // maximum iteration number
			mpc.addStrOption("hessian_approximation", "limited-memory");
			mpc.addNumOption("tol", step); mpc.addNumOption("dual_inf_tol", step); 
			mpc.addNumOption("acceptable_tol", step);
			mpc.setNc(nc); // set sampling time
			mpc.setXset(xset); // set set-point
			double[] cet = new double[tt];
			double average = 0;
			for(int i=0; i<tt; i++){
				xk = cons.getNewInitPoint(i/tt);
				mpc.setXk(xk);
				long d1 = new Date().getTime();
				mpc.solve(u0);
				long d2 = new Date().getTime();
				cet[i] = d2 - d1;
				average = average + cet[i];
				System.out.println("At "+ i + ": status is: "+mpc.getStatus());
			}
			System.out.println("Average evaluation time of centralized MPC with N="+N+", nc="+nc+" in "+ tt +" runs is: " + average/tt + " ms");
		}
		else if(MPC_Test_ID == 's'){
			Constants cons = new Constants();
			double[] xset = new double[Constants.ODE], xk = new double[Constants.ODE];
			xset = cons.getSetPoint(); // set point
			Sontag son = new Sontag(xset);
			double[][] u_prev = new double[N][7];
			int nn1 = 2, nn2 = 3, nn3 = 2, n1 = nn1*N, n2 = nn2*N, n3 = nn3*N, m = 1;
			double[] u1_L = new double[n1], u1_U = new double[n1], u2_L = new double[n2], u2_U = new double[n2],
					 u3_L = new double[n3], u3_U = new double[n3];
			double[] g_L = new double[m], g_U = new double[m];
			for(int i=0; i<N; i++){
				u1_L[i*nn1 + 0] = -1; u1_U[i*nn1 + 0] = 1;
				u1_L[i*nn1 + 1] = -1; u1_U[i*nn1 + 1] = 1;
			}
			for(int i=0; i<N; i++){
				u2_L[i*nn2 + 0] = -1; u2_U[i*nn2 + 0] = 1;
				u2_L[i*nn2 + 1] = -1; u2_U[i*nn2 + 1] = 1;
				u2_L[i*nn2 + 2] = -1; u2_U[i*nn2 + 2] = 1;
			}
			for(int i=0; i<N; i++){
				u3_L[i*nn3 + 0] = -1; u3_U[i*nn3 + 0] = 1;
				u3_L[i*nn3 + 1] = -1; u3_U[i*nn3 + 1] = 1;
			}
			for(int i=0; i<m; i++){
				g_L[i] = -1e11;
				g_U[i] = 1e-3;
			}
			double step = 1e-4;
			DMPC mpc1 = new DMPC(n1, m, u1_L, u1_U, g_L, g_U); // create IPOPT
			DMPC mpc2 = new DMPC(n2, m, u2_L, u2_U, g_L, g_U); // create IPOPT
			DMPC mpc3 = new DMPC(n3, m, u3_L, u3_U, g_L, g_U); // create IPOPT
			mpc1.setMPCID(1); mpc2.setMPCID(2); mpc3.setMPCID(3);
			mpc1.setStep(step); mpc2.setStep(step); mpc3.setStep(step);
			step = 1e-4;
			mpc1.addIntOption("max_iter",400); mpc1.addIntOption("print_level",-2); mpc1.addStrOption("hessian_approximation", "limited-memory");
			mpc1.addNumOption("tol", step); mpc1.addNumOption("dual_inf_tol", step); mpc1.addNumOption("acceptable_tol", step);
			mpc2.addIntOption("max_iter",800); mpc2.addIntOption("print_level",-2); mpc2.addStrOption("hessian_approximation", "limited-memory");
			mpc2.addNumOption("tol", step); mpc2.addNumOption("dual_inf_tol", step); mpc2.addNumOption("acceptable_tol", step);
			mpc3.addIntOption("max_iter",400); mpc3.addIntOption("print_level",-2); mpc3.addStrOption("hessian_approximation", "limited-memory");
			mpc3.addNumOption("tol", step); mpc3.addNumOption("dual_inf_tol", step); mpc3.addNumOption("acceptable_tol", step);
			mpc1.setNc(nc); mpc1.setXset(xset);
			mpc2.setNc(nc); mpc2.setXset(xset);
			mpc3.setNc(nc); mpc3.setXset(xset);
			double[][] et = new double[tt][3];
			double[] average = new double[3];
			for(int i=0; i<tt; i++){
				xk = cons.getNewInitPoint(i/tt);
				double[][] uk_prev = son.predict(xk, N, nc);
				for(int l=0; l<N; l++){
					for(int k=2; k<7; k++){
						u_prev[l][k] = uk_prev[l][k-2];
					}
				}
				mpc1.setUPrev(u_prev); 
				mpc1.setXk(xk);
				mpc1.setURef(u_prev[0]);
				double[] u10 = new double[nn1*N];
				for(int s=0; s<N; s++){
					u10[s*nn1 + 0] = u_prev[s][0];
					u10[s*nn1 + 1] = u_prev[s][1];
				}
				System.out.print("At "+ i + ": status is: ");
				long d1 = new Date().getTime();
				mpc1.solve(u10);
				long d2 = new Date().getTime();
				System.out.print(""+mpc1.getStatus()+",\t");
				et[i][0] = d2 - d1; 
				for(int l=0; l<N; l++){
					u_prev[l][0] = u10[l*nn1 + 0];
					u_prev[l][1] = u10[l*nn1 + 1];
				}
				mpc2.setUPrev(u_prev);
				mpc2.setXk(xk);
				mpc2.setURef(u_prev[0]);
				double[] u20 = new double[nn2*N];
				for(int s=0; s<N; s++){
					u20[s*nn2 + 0] = u_prev[s][2];
					u20[s*nn2 + 1] = u_prev[s][3];
					u20[s*nn2 + 2] = u_prev[s][4];
				}
				long d3 = new Date().getTime();
				mpc2.solve(u20);
				long d4 = new Date().getTime();
				System.out.print(""+mpc2.getStatus()+",\t");
				et[i][1] = d4 - d3; 
				for(int l=0; l<N; l++){
					u_prev[l][2] = u20[l*nn2 + 0];
					u_prev[l][3] = u20[l*nn2 + 1];
					u_prev[l][4] = u20[l*nn2 + 2];
				}
				mpc3.setUPrev(u_prev);
				mpc3.setXk(xk);
				mpc3.setURef(u_prev[0]);
				double[] u30 = new double[nn3*N];
				for(int s=0; s<N; s++){
					u30[s*nn3 + 0] = u_prev[s][5];
					u30[s*nn3 + 1] = u_prev[s][6];
				}
				long d5 = new Date().getTime();
				mpc3.solve(u30);
				long d6 = new Date().getTime();
				System.out.println(""+mpc3.getStatus());
				et[i][2] = d6 - d5; 
				average[0] = average[0] + et[i][0];
				average[1] = average[1] + et[i][1];
				average[2] = average[2] + et[i][2];
			}
			System.out.println("Average evaluation time of seqential MPCs with N="+N+", nc="+nc+" in "+ tt +" runs are: " + average[1]/tt + " ms\t"+average[2]/tt+" ms\t"+average[0]/tt+ "ms");
		}
		else if(MPC_Test_ID == 'p'){
			Constants cons = new Constants();
			double[] xset = new double[Constants.ODE], xk = new double[Constants.ODE];
			xset = cons.getSetPoint(); // set point
			Sontag son = new Sontag(xset);
			double[][] u_prev = new double[N][7];
			int nn1 = 2, nn2 = 3, nn3 = 2, n1 = nn1*N, n2 = nn2*N, n3 = nn3*N, m = 1;
			double[] u1_L = new double[n1], u1_U = new double[n1], u2_L = new double[n2], u2_U = new double[n2],
					 u3_L = new double[n3], u3_U = new double[n3];
			double[] g_L = new double[m], g_U = new double[m];
			for(int i=0; i<N; i++){
				u1_L[i*nn1 + 0] = -1; u1_U[i*nn1 + 0] = 1;
				u1_L[i*nn1 + 1] = -1; u1_U[i*nn1 + 1] = 1;
			}
			for(int i=0; i<N; i++){
				u2_L[i*nn2 + 0] = -1; u2_U[i*nn2 + 0] = 1;
				u2_L[i*nn2 + 1] = -1; u2_U[i*nn2 + 1] = 1;
				u2_L[i*nn2 + 2] = -1; u2_U[i*nn2 + 2] = 1;
			}
			for(int i=0; i<N; i++){
				u3_L[i*nn3 + 0] = -1; u3_U[i*nn3 + 0] = 1;
				u3_L[i*nn3 + 1] = -1; u3_U[i*nn3 + 1] = 1;
			}
			for(int i=0; i<m; i++){
				g_L[i] = -1e11;
				g_U[i] = 1e-3;
			}
			double step = 1e-4;
			DMPC mpc1 = new DMPC(n1, m, u1_L, u1_U, g_L, g_U); // create IPOPT
			DMPC mpc2 = new DMPC(n2, m, u2_L, u2_U, g_L, g_U); // create IPOPT
			DMPC mpc3 = new DMPC(n3, m, u3_L, u3_U, g_L, g_U); // create IPOPT
			mpc1.setMPCID(1); mpc2.setMPCID(2); mpc3.setMPCID(3);
			mpc1.setStep(step); mpc2.setStep(step); mpc3.setStep(step);
			step = 1e-4;
			mpc1.addIntOption("max_iter",400); mpc1.addIntOption("print_level",-2); mpc1.addStrOption("hessian_approximation", "limited-memory");
			mpc1.addNumOption("tol", step); mpc1.addNumOption("dual_inf_tol", step); mpc1.addNumOption("acceptable_tol", step);
			mpc2.addIntOption("max_iter",800); mpc2.addIntOption("print_level",-2); mpc2.addStrOption("hessian_approximation", "limited-memory");
			mpc2.addNumOption("tol", step); mpc2.addNumOption("dual_inf_tol", step); mpc2.addNumOption("acceptable_tol", step);
			mpc3.addIntOption("max_iter",400); mpc3.addIntOption("print_level",-2); mpc3.addStrOption("hessian_approximation", "limited-memory");
			mpc3.addNumOption("tol", step); mpc3.addNumOption("dual_inf_tol", step); mpc3.addNumOption("acceptable_tol", step);
			mpc1.setNc(nc); mpc1.setXset(xset);
			mpc2.setNc(nc); mpc2.setXset(xset);
			mpc3.setNc(nc); mpc3.setXset(xset);
			double[][] et = new double[tt][3];
			double[] average = new double[3];
			for(int i=0; i<tt; i++){
				xk = cons.getNewInitPoint(i/tt);
				double[][] uk_prev = son.predict(xk, N, nc);
				for(int l=0; l<N; l++){
					for(int k=2; k<7; k++){
						u_prev[l][k] = uk_prev[l][k-2];
					}
				}
				mpc1.setUPrev(u_prev); 
				mpc1.setXk(xk);
				mpc1.setURef(u_prev[0]);
				double[] u10 = new double[nn1*N];
				for(int s=0; s<N; s++){
					u10[s*nn1 + 0] = u_prev[s][0];
					u10[s*nn1 + 1] = u_prev[s][1];
				}
				System.out.print("At "+ i + ": status is: ");
				long d1 = new Date().getTime();
				mpc1.solve(u10);
				long d2 = new Date().getTime();
				System.out.print(""+mpc1.getStatus()+",\t");
				et[i][0] = d2 - d1; 
				mpc2.setUPrev(u_prev);
				mpc2.setXk(xk);
				mpc2.setURef(u_prev[0]);
				double[] u20 = new double[nn2*N];
				for(int s=0; s<N; s++){
					u20[s*nn2 + 0] = u_prev[s][2];
					u20[s*nn2 + 1] = u_prev[s][3];
					u20[s*nn2 + 2] = u_prev[s][4];
				}
				long d3 = new Date().getTime();
				mpc2.solve(u20);
				long d4 = new Date().getTime();
				System.out.print(""+mpc2.getStatus()+",\t");
				et[i][1] = d4 - d3; 
				mpc3.setUPrev(u_prev);
				mpc3.setXk(xk);
				mpc3.setURef(u_prev[0]);
				double[] u30 = new double[nn3*N];
				for(int s=0; s<N; s++){
					u30[s*nn3 + 0] = u_prev[s][5];
					u30[s*nn3 + 1] = u_prev[s][6];
				}
				long d5 = new Date().getTime();
				mpc3.solve(u30);
				long d6 = new Date().getTime();
				System.out.println(""+mpc3.getStatus());
				et[i][2] = d6 - d5; 
				average[0] = average[0] + et[i][0];
				average[1] = average[1] + et[i][1];
				average[2] = average[2] + et[i][2];
			}
			System.out.println("Average evaluation time of parallel MPCs with N="+N+", nc="+nc+" in "+ tt +" runs are: " + average[1]/tt + " ms\t"+average[2]/tt+" ms\t"+average[0]/tt+ "ms");
		}
	}

}
