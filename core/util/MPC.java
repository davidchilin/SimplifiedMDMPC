package core.util;

import core.model.*;

import org.coinor.Ipopt;

/** 
 * MPC extends Ipopt and implements all the necessary functions. 
 * It provides a functional box of MPC. In order to use it, all the constant 
 * arguments should be set immediately after the creation of an MPC object, 
 * and xk - the current state - should be updated each time before the evaluation
 * of MPC. 
 * */

public class MPC extends Ipopt{
	private int n, m, nele_jac, nele_hess;
	private double[] xk, xset;
	private double[] Qc = Constants.Qc;
	private double[] Rc = Constants.Rc;
	private double[] Rc2 = Constants.Rc2;
	private double[] Pv = Constants.Pv;
	private int nc;
	PEb dyn;

	/** Create an object of MPC. 
	 * n, length of the control inputs, it equals N*dim(u) 
	 * m, number of constraints 
	 * u_L, u_U, bounds on control inputs
	 * g_L, g_U, bounds on constraints 
	 * */
	public MPC(int n, int m, double[] u_L, double[] u_U, double[] g_L, double[] g_U){
		/* Part 1: Do not change part 1*/
		this.n = n; this.m = m; nele_jac = m*n;	nele_hess = n*(n+1)/2;
		create(this.n, u_L, u_U, this.m, g_L, g_U, nele_jac, nele_hess, Ipopt.C_STYLE);
		/* Part 2 */
		dyn = new PEb();
	}

	/** Calculate cost function
	 * n, length of the control inputs
	 * u, control inputs returned by Ipopt
	 * obj_value, value of the cost function under the returned u, 
	 * 			this obj_value is returned to Ipopt to determine if the 
	 * 			optimal u has been found */
	protected boolean eval_f(int n, double[] u, boolean new_x, double[] obj_value) {
		/* Part 1: do not change part 1 */
		/* Part 2: the cost function under the current u is calculated below */
		double[] xkk = new double[Constants.ODE];
		for(int i=0; i<Constants.ODE; i++){
			xkk[i] = this.xk[i]; // Note: xkk and FTkk will be changed below, a for loop is needed
		}
		double[] xset = this.xset; // Note: xset, Qc and Rc will not be changed
		double[] Qc = this.Qc; 	   // so we can define a variable by one equality 
		double[] Rc = this.Rc;	   // instead of a for loop
		double[] Rc2 = this.Rc2;
		double cost = 0;
		
		// Add up the first term: cost = xk'Qcxk;
		for(int i=0; i<xkk.length; i++){
			cost = cost + (xkk[i]-xset[i])*Qc[i]*(xkk[i]-xset[i]);
		}
		double[] uk_f = new double[2]; // 2 flows: F2, F3
		double[] uk = new double[5]; // 5 coolant or heat: Q1, Q2, Q3, Q4, Q5
		for(int k=0; k<n/7; k++){
			for(int l=0; l<2; l++){
				uk_f[l] = u[k*7+l];
			}
			for(int l=2; l<7; l++){
				uk[l-2] = u[k*7+l];
			}
			// apply control
			xkk = dyn.solve(nc, xkk, uk, uk_f);
			for(int j=0; j<xkk.length; j++){
				cost = cost + (xkk[j]-xset[j])*Qc[j]*(xkk[j]-xset[j]);
			}
			for(int i=0; i<5; i++){
				cost = cost + (uk[i]*Constants.umax[i+2])*Rc[i]*(uk[i]*Constants.umax[i+2]);	
			}
			for(int i=0; i<2; i++){
				cost = cost + uk_f[i]*Constants.umax[i]*Rc2[i]*uk_f[i]*Constants.umax[i];
			}
		}
		obj_value[0] = cost;
		return true;
	}

	/** Define stability constraints and other constraints
	 * g, constraints are returned through this array */
	protected boolean eval_g(int n, double[] u, boolean new_x, int m, double[] g) {
		/* Part 1, do not change */
		/* Part 2, 1 stability constraint */
		// NOTE: m = 1
		
		Sontag son = new Sontag(xset);
		double[] xkk = new double[Constants.ODE];
		double[] xkk2 = new double[Constants.ODE];
		for(int i=0; i<Constants.ODE; i++){
			xkk[i] = this.xk[i];
			xkk2[i] = this.xk[i];
		}
		double[] uson = son.solve(xk);
		double[] uk = new double[5];
		for(int i=0; i<5; i++){
			uk[i] = u[i+2];
		}
		double[] uk_f = new double[2];
		for(int i=0; i<2; i++){
			uk_f[i] = u[i];
		}
		
		double[] uk_f2 = {0, 0};
		xkk = dyn.solve(nc, xkk, uk, uk_f);
		xkk2 = dyn.solve(nc, xkk2, uson, uk_f2);
		double V=0, V2=0;
		for(int j=0; j<xkk.length; j++){
			V = V + (xkk[j]-xset[j])*Pv[j]*(xkk[j]-xset[j]);
			V2 = V2 + (xkk2[j]-xset[j])*Pv[j]*(xkk2[j]-xset[j]);
		}
		g[0] = V - V2;
		//System.out.println(g[0]);
		
		/*
		double[] dV = son.evalGrad(xk);
		double[][] g1 = dyn.eval_g1(xk);
		double[][] g2 = dyn.eval_g2(xk);
		double[] Lg1V = new double[5];
		double[] Lg2V = new double[2];
		
		Lg1V[0] = dV[4]*g1[0][4];
		Lg1V[1] = dV[9]*g1[1][9];
		Lg1V[2] = dV[14]*g1[2][14];
		Lg1V[3] = dV[15]*g1[3][19];
		Lg1V[4] = dV[20]*g1[4][24];
		
		Lg2V[0] = dV[6]*g2[0][6] + dV[9]*g2[0][9];
		Lg2V[1] = dV[11]*g2[1][11] + dV[14]*g2[1][14];
		
		g[0] = Lg2V[0]*uk_f[0] + Lg2V[1]*uk_f[1]
		       + Lg1V[0]*uk[0] + Lg1V[1]*uk[1] + Lg1V[2]*uk[2] + Lg1V[3]*uk[3] + Lg1V[4]*uk[4]
		       - (Lg1V[0]*uson[0] + Lg1V[1]*uson[1] + Lg1V[2]*uson[2] + Lg1V[3]*uson[3] + Lg1V[4]*uson[4]);
		*/
		//g[0] = -1;
		return true;
	}

	public boolean setXk(double[] xk){
		this.xk=xk;
		return true;
	}
	public double[] getXk(){
		return this.xk;
	}
	public boolean setXset(double[] xset){
		this.xset=xset;
		return true;
	}
	public double[] getXset(){
		return this.xset;
	}
	public boolean setNc(int nc){
		this.nc = nc;
		return true;
	}
	public int getNc(){
		return this.nc;
	}
}
