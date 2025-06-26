package core.test.util;

import core.model.*;

public class PID {
	public double[] xset;
	public double[] K;
	public double[] tau;
	
	public PID(double[] xset, double[] K, double[] tau){
		this.xset = xset;
		this.K = K;
		this.tau = tau;
	}
	
	public double[] solve(double[] xk, double[] err_sum){
		double[] err = new double[5];
		double[] uk = new double[5];
		
		err[0] = xset[4] - xk[4];
		err[1] = xset[9] - xk[9];
		err[2] = xset[14] - xk[14];
		err[3] = xset[19] - xk[19];
		err[4] = xset[24] - xk[24];
		for(int k=0; k<5; k++){
			uk[k] =  K[k]*(err[k] + err_sum[k]/tau[k]);
			if(uk[k] > Constants.umax[k+2])
				uk[k] = Constants.umax[k+2];
			else if(uk[k]<-Constants.umax[k+2])
				uk[k] = -Constants.umax[k+2];
		}				
		return uk;
	}
	
	public double[][] predict(double[] xk, double[] err_sum, int N, int nc){
		double[][] pred = new double[N][5];
		double h = Constants.h;
		Dynamics dyn = new Dynamics();
		pred[0] = solve(xk, err_sum);
		double[] uk_f = {0, 0};
		for(int i=1; i<N; i++){
			for(int j=0; j<nc; j++){
				xk = dyn.solve(xk, pred[i], uk_f);
				err_sum[0] = err_sum[0] + (xset[4] - xk[4])*h;
				err_sum[1] = err_sum[1] + (xset[9] - xk[9])*h;
				err_sum[2] = err_sum[2] + (xset[14] - xk[14])*h;
				err_sum[3] = err_sum[3] + (xset[19] - xk[19])*h;
				err_sum[4] = err_sum[4] + (xset[24] - xk[24])*h;
			}
			pred[i] = solve(xk, err_sum);
		}
		return pred;
	}
}
