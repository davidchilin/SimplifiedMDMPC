package core.test;

import java.io.*;

import core.model.*;
import core.test.util.PID;
import core.util.FileIO;
import core.util.GenerateBoundedNoise;
import core.util.NicePlot;

public class PIControl {
	public static void main(String[] args){
		Constants cons = new Constants();
		double[] xset = new double[Constants.ODE];
		double[] xk = new double[Constants.ODE];
		xk = cons.getSetPoint(); // set point
		xset = cons.getSetPoint(); // initial point
		
		double[] K = Constants.K;
		double[] tau = Constants.tau;
		double[] err_sum = new double[5];
		double h = Constants.h;
		
		double[] uk = new double[5];
		double nc = 1; // hc = nc*h controller hold time
		int cc = 5000; // simulation time
		int sc = 50;
		PID pid = new PID(xset, K, tau);
		double[] t = new double[cc+1]; 
		double[][] x = new double[cc+1][Constants.ODE];
		double[][] u = new double[cc+1][5];
		GenerateBoundedNoise gen = new GenerateBoundedNoise();
		gen.setSeed(0);
		//double[][] pnoise = gen.generateNoise(cc);
		
		for(int i=0; i<Constants.ODE; i++){
			x[0][i] = xk[i];
		}
		t[0] = 0;

		try{
			BufferedWriter out = new FileIO().getFileBufferedWriter("./data/sim/pid_x_pn.data");
			BufferedWriter uout = new FileIO().getFileBufferedWriter("./data/sim/pid_u_pn.data");
		
			String firstline = t[0]+"";
			for(int j=0; j<xk.length; j++){
				firstline = firstline + "\t" + xk[j];
			}
			out.write(firstline);
			out.newLine();
			out.flush();
			
			Dynamics dyn = new Dynamics();
			int dc = 0;
		
			for(int i=0; i<cc; i++){
				err_sum[0] = err_sum[0] + (xset[4] - xk[4])*h;
				err_sum[1] = err_sum[1] + (xset[9] - xk[9])*h;
				err_sum[2] = err_sum[2] + (xset[14] - xk[14])*h;
				err_sum[3] = err_sum[3] + (xset[19] - xk[19])*h;
				err_sum[4] = err_sum[4] + (xset[24] - xk[24])*h;
				if(i%nc == 0){
					uk = pid.solve(xk,err_sum);
					//for(int k = 0; k<5; k++){
						//uk[k] = Constants.Q2[k] - Constants.Q[k] ;
					//}
				}else{
					for(int k=0; k<5; k++){
						uk[k] = u[i-1][k];
					}
				}
				for(int k=0; k<5; k++){
					u[i][k] = uk[k];
				}
				String uline = t[i] + "";
				for(int k=0; k<5; k++){
					uline = uline + "\t" + uk[k];
				}
				uout.write(uline);
				uout.newLine();
				uout.flush();
				
				double[] uk_f={0, 0};
				xk = dyn.solve(xk, uk, uk_f);
				//xk = dyn.solveWithNoise(xk, uk, uk_f, pnoise[i]);
				for(int k=0; k<Constants.ODE; k++){
					x[i+1][k] = xk[k];
				}
				t[i+1] = (i+1)*h;
				String line = t[i+1]+"";
				for(int j=0; j<xk.length; j++){
					line = line + "\t" + xk[j];
				}
				out.write(line);
				out.newLine();
				out.flush();
				
				dc = dc + 1;
				if(dc == sc)
				{
					System.out.println(i+1);
					dc = 0;
				}
			}
			out.close();
			uout.close();
			for(int i=0; i<5; i++){
				u[cc][i] = u[cc-1][i];
			}
			
			System.out.println("U: "+uk[0]+"\t"+uk[1]+"\t"+uk[2]+"\t"+uk[3]+"\t"+uk[4]);
			int[] display = {0,1,2,3,4};
			int[] display2 = {5,6,7,8,9};
			int[] display3 = {10,11,12,13,14};
			int[] display4 = {15,16,17,18,19};
			int[] display5 = {20,21,22,23,24};
			int[] udisplay = {0,1,2,3,4};
			new NicePlot("Temperatures", x, t, display).plot();
			new NicePlot("Temperatures", x, t, display2).plot();
			new NicePlot("Temperatures", x, t, display3).plot();
			new NicePlot("Temperatures", x, t, display4).plot();
			new NicePlot("Temperatures", x, t, display5).plot();
			new NicePlot("Control Inputs", u, t, udisplay).plot();
		}
		catch(Exception e){
			e.printStackTrace();
		}
	}
}
