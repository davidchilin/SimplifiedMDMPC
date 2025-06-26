package core.test.util;

import Jampack.*;
import java.io.*;

import core.util.FileIO;

public class Newton {
	protected NewtonFunction fun;
	protected int MAX_ITERATION = 30;
	protected double ERR_TOLERANCE = 1e-11;
	protected int ITER_COUNT = 0;
	
	public Newton(NewtonFunction fun){
		this.fun = fun;
	}
	
	public int getMaxIteration(){
		return MAX_ITERATION;
	}
	public void setMaxIteration(int m){
		this.MAX_ITERATION = m;
	}
	public double getErrTolerance(){
		return ERR_TOLERANCE;
	}
	public void setErrTolerance(double e){
		this.ERR_TOLERANCE = e;
	}
	public int getIterCount(){
		return ITER_COUNT;
	}
	
	public double[][] evalGradient(double[] xk) {
		double step = 1e-11;
		int n = xk.length;
		double xpstep[] = new double[n];
		double xmstep[] = new double[n];
		double objp[] = new double[n];
		double objm[] = new double[n];
		double[][] grad = new double[n][n];
		
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
			objp = fun.eval_f(xpstep);
			objm = fun.eval_f(xmstep);
			for(int k=0; k<n; k++){
				grad[k][i] = (objp[k]-objm[k])/step/2;
			}
		}
		return grad;
	}		
	
	public double[] findRoot(double[] xk){
		try{
			int n = xk.length;
			double[] xk0 = new double[n];
			double[] xk1 = new double[n];
			for(int i=0; i<n; i++){
				xk0[i] = xk[i];
			}
			xk1 = fun.eval_f(xk0);
			double err = 1;
			for(int i=0; i<n; i++){
				err += Math.abs(xk1[i]);
			}	
			int cc = 0;
			while((err>ERR_TOLERANCE)&&(cc<MAX_ITERATION)){
				double[][] grad = evalGradient(xk0);
				try{
					BufferedWriter out = new FileIO().getFileBufferedWriter("./data/grad.data");
					for(int i=0; i<grad.length; i++){
						String line = "";
						for(int j=0; j<grad[0].length; j++){
							line = line + grad[i][j] +"\t";
						}
						out.write(line);
						out.newLine();
						out.flush();
					}
					out.close();
				}catch(Exception e){
					e.printStackTrace();
				}
				Zmat Delta = new Zmat(grad);
				Zmat invDelta = Inv.o(Delta);
				double[] f = fun.eval_f(xk0);
				for(int i=0; i<n; i++){
					xk1[i] = 0; // reset xk1
					for(int j=0; j<n; j++){
						xk1[i] -= invDelta.get(i+1, j+1).re*f[j];
					}
					xk1[i] += xk0[i];
				}
				err = 0;
				cc = cc + 1;
				for(int i=0; i<n; i++){
					err += Math.abs(xk1[i]-xk0[i]);
					xk0[i] = xk1[i];
				}	
			}	
			// System.out.print(cc+"\n");
			this.ITER_COUNT = cc;
			return xk1;
		}catch(Exception e){
			e.printStackTrace();
			return xk;
		}
	}
}