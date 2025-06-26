package core.util;

import core.model.*;

/** @author Jinfeng 
 *  @version 1.0 
 *  @category Sontag formula for Styrene */

public class Sontag {
	private double[] xset;
	private double[] Pv = Constants.Pv;
	
	public Sontag(double[] xset){
		this.xset = xset;
	}
	
	public static void main(String[] args){
			Constants cons = new Constants();
			double[] xset = new double[Constants.ODE];
			double[] xk = new double[Constants.ODE];
			// x_set
			xset = cons.getSetPoint(); // set point
			xk = cons.getInitPoint();
			Sontag sontag = new Sontag(xset);
			double[] uk = sontag.solve(xk);
			for(int i=0; i<uk.length; i++){
				System.out.println(uk[i]);
			}
		
	}
	public double[] solve(double[] xk){
		double[] dV = evalGrad(xk);
		Dynamics dyn = new Dynamics();
		double[] f = dyn.eval_f(xk);
		double LfV = 0;
		for(int i=0; i<f.length; i++){
			LfV = LfV + dV[i]*f[i];
		}
		double[][] g1 = dyn.eval_g1(xk);
		double[] LgV = new double[5];
		LgV[0] = dV[4]*g1[0][4];
		LgV[1] = dV[9]*g1[1][9];
		LgV[2] = dV[14]*g1[2][14];
		LgV[3] = dV[19]*g1[3][19];
		LgV[4] = dV[24]*g1[4][24];
		double[] uk = new double[5];
		for(int i=0; i<uk.length; i++){
			if(Math.abs(LgV[i]) < 1e-4){
				uk[i] = 0;
			} else {
				//uk[i] = -(LfV + Math.sqrt(LfV*LfV + Math.pow(LgV[i], 4)))/LgV[i];
				
				double temp = (1e-4+LgV[i])*(1+Math.sqrt(1+Math.pow(1*LgV[i], 2)));
				uk[i] = -(LfV + Math.sqrt(LfV*LfV+Math.pow(1*LgV[i], 4)))/temp;
				if(uk[i] > 1)
					uk[i] = 1;
				else if(uk[i]<-1)
					uk[i] = -1;
				
			}
			uk[i] = uk[i];// + (Constants.Q2[i] - Constants.Q[i])/Constants.umax[i+2];
			//uk[i] = (Constants.Q2[i] - Constants.Q[i])/Constants.umax[i+2];
			if(uk[i] > 1)
				uk[i] = 1;
			else if(uk[i]<-1)
				uk[i] = -1;
		}
		return uk;
	}
	
	public double[][] predict(double[] xk, int N, int nc){
		double[][] pred = new double[N][5];
		double[] xk2 = new double[xk.length];
		for(int i=0; i<xk.length; i++){
			xk2[i] = xk[i];
		}
		Dynamics dyn = new Dynamics();
		pred[0] = solve(xk);
		double[] uk_f = {0, 0};
		for(int i=1; i<N; i++){
			xk2 = dyn.solve(nc, xk2, pred[0], uk_f); 
			pred[i] = solve(xk2);
		}
		return pred;
	}
	
	
	private double evalV(double[] xk){
		double V = 0;
		for(int i=0; i<xk.length; i++){
			V = V + (xk[i]-xset[i])*Pv[i]*(xk[i]-xset[i]);
		}
		return V;
	}

	public double[] evalGrad(double[] xk) {
		double step = 1e-4;
		int n = xk.length;
		double[] xpstep = new double[n];
		double[] xmstep = new double[n];
		double[] grad = new double[n];
		double objp = 0;
		double objm = 0;
		for(int i=0; i<n; i++)
		{
			for(int j=0; j<n; j++)
			{
				if(j==i)
				{
					xpstep[j] = xk[j] + step;
					xmstep[j] = xk[j] - step;
				}
				else
				{
					xpstep[j] = xk[j];
					xmstep[j] = xk[j];
				}
			}
			objp = evalV(xpstep);
			objm = evalV(xmstep);
			grad[i] = (objp-objm)/step/2;
		}
		return grad;
	}
}
