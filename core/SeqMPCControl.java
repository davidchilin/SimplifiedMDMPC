package core;

import java.io.BufferedWriter;
import java.util.Date;

import core.model.Constants;
import core.model.Dynamics;
import core.util.DMPC;
import core.util.FileIO;
import core.util.GenerateBoundedNoise;
import core.util.NicePlot;
import core.util.Sontag;

public class SeqMPCControl {
	public static void main(String[] args){
		/* Set up initial state and final state */
		Constants cons = new Constants();
		double[] xset = new double[Constants.ODE], xk = new double[Constants.ODE];
		xk = cons.getInitPoint(); // set point
		xset = cons.getSetPoint(); // initial point

		/* Set up simulation environment */
		int cc = 500; // simulation time
		int lcc = 400; //?
		double[] uk = new double[5], uk_f = new double[2], t = new double[cc+1];
		double[][] et = new double[50][3]; //?
		int cet = 0;
		double[][] x = new double[cc+1][Constants.ODE], u = new double[cc+1][7];
		GenerateBoundedNoise gen = new GenerateBoundedNoise();
		gen.setSeed(0);
		double[][] pnoise = gen.generateNoise(cc);
		Dynamics dyn = new Dynamics();
		Sontag son = new Sontag(xset);

		/* Set up MPC */
		int nc = 10, N = 1;
		int coc = 30; //?
		double[] Qc = Constants.Qc, Rc = Constants.Rc, Rc2 = Constants.Rc2;
		double[][] u_prev = new double[N][7];
		String xfile = "./data/newsim2/seq_x_n"+N+"_nc"+nc+".data";
		String ufile = "./data/newsim2/seq_u_n"+N+"_nc"+nc+".data";

		/* Set up Ipopt */
		int nn1 = 2, nn2 = 3, nn3 = 2, n1 = nn1*N, n2 = nn2*N, n3 = nn3*N, m = 1;
		double[] u1_L = new double[n1], u1_U = new double[n1], u2_L = new double[n2], u2_U = new double[n2],
				 u3_L = new double[n3], u3_U = new double[n3];
		double[] g_L = new double[m], g_U = new double[m], u1_opt = new double[n1], u2_opt = new double[n2],
				 u3_opt = new double[n3];
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
		//mpc1.addStrOption("nlp_scaling_method", "none");
		//mpc2.addStrOption("nlp_scaling_method", "none");
		//mpc3.addStrOption("nlp_scaling_method", "none");

		mpc1.setNc(nc); mpc1.setXset(xset);
		mpc2.setNc(nc); mpc2.setXset(xset);
		mpc3.setNc(nc); mpc3.setXset(xset);

		for(int i=0; i<Constants.ODE; i++){
			x[0][i] = xk[i];
		}
		t[0] = 0;

		try{
			BufferedWriter out = new FileIO().getFileBufferedWriter(xfile);
			BufferedWriter uout = new FileIO().getFileBufferedWriter(ufile);

			String firstline = t[0]+"";
			for(int j=0; j<xk.length; j++){
				firstline = firstline + "\t" + xk[j];
			}
			out.write(firstline);
			out.newLine();
			out.flush();

			for(int i=0; i<cc; i++){
				if(i%nc == 0){
					double[][] uk_prev = son.predict(xk, N, nc);
					for(int l=0; l<N; l++){
						for(int k=2; k<7; k++){
							u_prev[l][k] = uk_prev[l][k-2]; //?
						}
					}
					mpc1.setUPrev(u_prev); 
					mpc1.setXk(xk);
					mpc1.setURef(u_prev[0]);
					double[] u10 = new double[nn1*N];
					for(int s=0; s<N; s++){
						u10[s*nn1 + 0] = u_prev[s][0]; //?
						u10[s*nn1 + 1] = u_prev[s][1];
					}
					System.out.print("At " + i + "\t : status of MPC 1 is ");
					long d1 = new Date().getTime();
					u1_opt = mpc1.solve(u10);
					long d2 = new Date().getTime();
					System.out.print(""+mpc1.getStatus()+",\t");
					if(cet < 50){ //?
						et[cet][0] = d2 - d1; 
					}
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
					System.out.print("MPC 2 is ");
					long d3 = new Date().getTime();
					u2_opt = mpc2.solve(u20);
					long d4 = new Date().getTime();
					System.out.print(""+mpc2.getStatus()+",\t");
					if(cet < 50){
						et[cet][1] = d4 - d3; 
					}
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
					System.out.print("MPC 3 is ");
					long d5 = new Date().getTime();
					u3_opt = mpc3.solve(u30);
					long d6 = new Date().getTime();
					System.out.println(""+mpc3.getStatus());
					if(cet < 50){
						et[cet][2] = d6 - d5; 
					}
					cet = cet + 1;
					for(int s=0; s<N; s++){
						u_prev[s][5] = u3_opt[0+nn3*s];
						u_prev[s][6] = u3_opt[1+nn3*s];
					}
					for(int l=0; l<nn1; l++){
						uk_f[l] = u1_opt[l];
					}
					for(int l=0; l<nn2; l++){
						uk[l] = u2_opt[l];
					}
					for(int l=nn2; l<5; l++){
						uk[l] = u3_opt[l-nn2];
					}
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
				String uline = t[i] + "";
				for(int k=0; k<2; k++){
					uline = uline + "\t" + uk_f[k];
				}
				for(int k=0; k<5; k++){
					uline = uline + "\t" + uk[k];
				}
				uout.write(uline);
				uout.newLine();
				uout.flush();

				//xk = dyn.solve(xk, uk, uk_f);
				xk = dyn.solveWithNoise(xk, uk, uk_f, pnoise[i]);
				for(int k=0; k<Constants.ODE; k++){
					x[i+1][k] = xk[k];
				}
				t[i+1] = (i+1)*Constants.h;
				String line = t[i+1]+"";
				for(int j=0; j<xk.length; j++){
					line = line + "\t" + xk[j];
				}
				out.write(line);
				out.newLine();
				out.flush();
			}
			out.close();
			uout.close();
			for(int i=0; i<7; i++){
				u[cc][i] = u[cc-1][i];
			}
			if(cet>50){
				double tsum1 = 0, tsum2 = 0, tsum3 = 0;
				for(int i=0; i<50; i++){
					tsum1 = tsum1 + et[i][0];
					tsum2 = tsum2 + et[i][1];
					tsum3 = tsum3 + et[i][2];
				}
				System.out.println("Average evaluation time of MPC1 in " + 50 + " runs is : "+ tsum1/50 +" ms");
				System.out.println("Average evaluation time of MPC2 in " + 50 + " runs is : "+ tsum2/50 +" ms");
				System.out.println("Average evaluation time of MPC3 in " + 50 + " runs is : "+ tsum3/50 +" ms");
			}else if(cet <=50){
				double tsum1 = 0, tsum2 = 0, tsum3 = 0;
				for(int i=0; i<cet; i++){
					tsum1 = tsum1 + et[i][0];
					tsum2 = tsum2 + et[i][1];
					tsum3 = tsum3 + et[i][2];
				}
				System.out.println("Average evaluation time of MPC1 in " + cet + " runs is : "+ tsum1/50 +" ms");
				System.out.println("Average evaluation time of MPC2 in " + cet + " runs is : "+ tsum2/50 +" ms");
				System.out.println("Average evaluation time of MPC3 in " + cet + " runs is : "+ tsum3/50 +" ms");
			}

			int[] display = {4,9,14,15,20};
			int[] udisplay = {0,1,2,3,4};
			new NicePlot("Temperatures", x, t, display).plot();
			new NicePlot("Control Inputs", u, t, udisplay).plot();
			double cost = 0;
			for(int i=0; i<lcc; i+=coc){
				for(int j=0; j<Constants.ODE; j++){
					cost = cost + (x[i][j] - xset[j])*Qc[j]*(x[i][j] - xset[j]);
				}
				for(int j=0; j<2; j++){
					cost =  cost + u[i][j]*Rc2[j]*u[i][j]*Constants.umax[j]*Constants.umax[j];
				}
				for(int j=0; j<5; j++){
					cost =  cost + u[i][j+2]*Rc[j]*u[i][j+2]*Constants.umax[j+2]*Constants.umax[j+2];
				}
			}
			System.out.println("Total cost is : "+cost);

		}
		catch(Exception e){
			e.printStackTrace();
		}
	}
}
