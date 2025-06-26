package core;

import java.io.BufferedWriter;
import java.util.Date;

import core.model.Constants;
import core.model.Dynamics;
import core.util.*;
import core.util.FileIO;
import core.util.GenerateBoundedNoise;
import core.util.NicePlot;

public class CentralizedMPC {
	public static void main(String[] args){
		/* Set up initial state and final state */
		Constants cons = new Constants();
		double[] xset = new double[Constants.ODE], xk = new double[Constants.ODE];
		xset = cons.getSetPoint(); // set point
		xk = cons.getInitPoint(); // initial point
		double[] xinit = cons.getInitPoint();
		double[] xkk2 = cons.getInitPoint();
		//xk = cons.getNewInitPoint();

		/* Set up simulation environment */
		int cc = 1000; // simulation time
		int lcc = 900; //?
		double[] uk = new double[5], uk_f = new double[2], t = new double[cc+1]; 
		double[] et = new double[50];
		int cet = 0; //?
		double[][] x = new double[cc+1][Constants.ODE], u = new double[cc+1][7];
		GenerateBoundedNoise gen = new GenerateBoundedNoise();
		gen.setSeed(7);
		//double[][] pnoise = gen.generateNoise(cc);
		Dynamics dyn = new Dynamics();
		
		/* Set up MPC */
		int nc = 30, N = 1;
		int coc = 30;
		double[] Qc = Constants.Qc, Rc = Constants.Rc, Rc2 = Constants.Rc2;  
		//String xfile = "./data/sim/c_x_n"+N+"_nc"+nc+"_pn.data";
		//String ufile = "./data/sim/c_u_n"+N+"_nc"+nc+"_pn.data";
		String xfile = "./data/newsim2/c_x_n"+N+"_nc"+nc+".data";
		String ufile = "./data/newsim2/c_u_n"+N+"_nc"+nc+".data";
		
		// initial guess of inputs
		double[] u0 = new double[7*N];
		/*
		for(int i=0; i<N; i++){
			u0[0 + i*7] = 0; 
			u0[1 + i*7] = 0;
			u0[2 + i*7] = (Constants.Q2[0] - Constants.Q[0])/Constants.umax[2]; 
			u0[3 + i*7] = (Constants.Q2[1] - Constants.Q[1])/Constants.umax[3];
			u0[4 + i*7] = (Constants.Q2[2] - Constants.Q[2])/Constants.umax[4];
			u0[5 + i*7] = (Constants.Q2[3] - Constants.Q[3])/Constants.umax[5];
			u0[6 + i*7] = (Constants.Q2[4] - Constants.Q[4])/Constants.umax[6];
		}
		*/
		int n = 7*N, m = 1;           // horizon n/5, constraints m one step constraint
		double[] u_L = new double[n]; // input constraint vector, lower bounds
		double[] u_U = new double[n]; // input constraint vector, upper bounds
		double[] g_L = new double[m]; // state constraint vector, lower bounds
		double[] g_U = new double[m]; // state constraint vector, upper bounds
		double[] u_opt = new double[n]; // optimal inputs returned by IPOPT
		// set u_L and u_U
		for(int i=0; i<n/7; i++){
			for(int j=0; j<7; j++){
				u_L[i*7 + j] = -1;
				u_U[i*7 + j] = 1;
			}
		}
		// set g_L and g_U
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
		//mpc.addStrOption("nlp_scaling_method", "none");
		mpc.setNc(nc); // set sampling time
		mpc.setXset(xset); // set set-point
		
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
					mpc.setXk(xk);
					System.out.print("Status at "+i +" is ");
					long d1 = new Date().getTime();
					u_opt = mpc.solve(u0);
					long d2 = new Date().getTime();
					System.out.println(""+mpc.getStatus());
					if(cet < 50){
						et[cet] = d2 - d1; 
					}
					cet = cet + 1;
					for(int l=0; l<5; l++){
						uk[l] = u_opt[l+2];
					}
					for(int l=0; l<2; l++){
						uk_f[l] = u_opt[l];
					}
					double V=0, V2=0;
					for(int j=0; j<xk.length; j++){
						V = V + (xk[j]-xset[j])*Constants.Pv[j]*(xk[j]-xset[j]);
						V2 = V2 + (xkk2[j]-xset[j])*Constants.Pv[j]*(xkk2[j]-xset[j]);
					}
					System.out.println("V= "+ V + "\t V2 = " + V2 +"\t" + (V-V2));
					xinit = xk;
					xkk2 = xk;
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
				
				Sontag son = new Sontag(xset);
				double[] uson = son.solve(xinit);
				double[] uk_ftest = {0,0};
				xkk2 = dyn.solve(xkk2, uson, uk_ftest);
				xk = dyn.solve(xk, uk, uk_f);
				//xk = dyn.solveWithNoise(xk, uk, uk_f,pnoise[i]);
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
				double tsum = 0;
				for(int i=0; i<50; i++){
					tsum = tsum + et[i];
				}
				System.out.println("Average evaluation time of MPC in " + 50 + " runs is : "+ tsum/50 +" ms");
			}else if(cet <=50){
				double tsum = 0;
				for(int i=0; i<cet; i++){
					tsum = tsum + et[i];
				}
				System.out.println("Average evaluation time of MPC1 in " + cet + " runs is : "+ tsum/50 +" ms");
			}
			
			int[] display = {0,1,2,3,4};
			int[] display2 = {5,6,7,8,9};
			int[] display3 = {10,11,12,13,14};
			int[] display4 = {15,16,17,18,19};
			int[] display5 = {20,21,22,23,24};
			int[] udisplay = {0,1,2,3,4,5,6};
			new NicePlot("Temperatures", x, t, display).plot();
			new NicePlot("Temperatures", x, t, display2).plot();
			new NicePlot("Temperatures", x, t, display3).plot();
			new NicePlot("Temperatures", x, t, display4).plot();
			new NicePlot("Temperatures", x, t, display5).plot();
			new NicePlot("Control Inputs", u, t, udisplay).plot();
			//double[][] ranges = {{485, 495},{500, 505}, 
			//		{490, 520}, {450, 470}, {420, 490}};
			
			double cost = 0;
			for(int i=0; i<lcc; i+=coc){
				for(int j=0; j<Constants.ODE; j++){
					cost = cost + (x[i][j] - xset[j])*Qc[j]*(x[i][j] - xset[j]);
				}
				for(int j=0; j<2; j++){
					cost =  cost + u[i][j]*Rc2[j]*u[i][j]*Constants.umax[j]*Constants.umax[j];
				}
				for(int j=0; j<5; j++){
					cost =  cost + (u[i][j+2])*Rc[j]*(u[i][j+2])*Constants.umax[j+2]*Constants.umax[j+2];
				}
			}
			System.out.println("Total cost is : "+cost);
			
		}
		catch(Exception e){
			e.printStackTrace();
		}
	}
}
