// John Model FDI Paramter Estimation
// PEaestimates fault value of Q1,Q2,Q3,Q4,Q5,F4 or F6.  evalPE calculates new steady state input target values.
// ver a, adjust cost to only match endpoint of history[counter].
package core.util;

import org.coinor.Ipopt;

import core.model.Constants;
import core.model.Dynamics;

public class PEa extends Ipopt {
	private int n, m, nele_jac, nele_hess;
	private double[] xk;
	private double[] xset;
	private double[] Qc, Rc, Rc2;
	private double h = Constants.h;
	
	double[][] Farpe = new double[Constants.ODE][]; // averaged record for parameter estimation
	double[][] Frca  = new double[7][]; // + control action, F4,F6,Q1,2,3,4,5
	int Counter;
	int fail;
	double pe = 0; // fault parameter estimate
	
	Dynamics dyn = new Dynamics();

	/* n: dimension of the control inputs; m: number of constraints */
	/* u_L, u_U: bounds on control inputs; g_L, g_U: constraints */
	public PEa(int n, int m, double[] u_L, double[] u_U, double[] g_L,
			double[] g_U) {
		this.n = n; // set the dimension of the control input, horizon*dim
		this.m = m; // set the number of inequality or equality constraints
		nele_jac = m * n;
		nele_hess = n * (n + 1) / 2;
		int index_style = Ipopt.C_STYLE; // start counting of rows and column indices at 0
		create(this.n, u_L, u_U, this.m, g_L, g_U, nele_jac, nele_hess, index_style);
	}

	public PEa() {
	}
	// estimate fault magnitude
	/* modify this function to write the correct objective function */
	protected boolean eval_f(int n, double[] u, boolean new_x, double[] obj_value) {
		
		double[][] xkk = new double[Constants.ODE][Counter+1];
		double[] txkk = new double[Constants.ODE];
		double[] usk = new double[5];
		double[] uskf = new double[2];
		double cost = 0;
		
		for (int i = 0; i < xkk.length; i++) {
			xkk[i][0] = this.Farpe[i][0];
		}			
		
		for (int k = 0; k < Counter-1; k++) {
		
			for (int i = 0; i < 2; i++) {
				uskf[i] = this.Frca[i][k];
			}

			for (int i = 0; i < 5; i++) {
				usk[i] = this.Frca[i+2][k];
			}

			if(fail<5)
				usk[fail]=u[0];
			else if(fail==5) uskf[0]=u[0];
			else uskf[1]=u[0];
			
			txkk = Stuff.Equal(dyn.solve(Stuff.Equal(xkk,k), usk, uskf)); // no noise
			Stuff.Equal(txkk, xkk,k+1);
		}
		
		for (int k = 0; k < Counter; k++)
			for (int i = 0; i < xkk.length; i++) // add up x'Qcx term
				cost += (xkk[i][k] - this.Farpe[i][k]) * (xkk[i][k] - this.Farpe[i][k]);

		obj_value[0] = cost;
		return true;
	}

	/* modify this function to set the inequality or equality constraints */
	protected boolean eval_g(int n, double[] u, boolean new_x, int m, double[] g) {	return true;}
	
	public double[] evalPE(double[] xkt) {
		double[] oPE = new double[7];
		
		try {

			return oPE;
		} catch (Exception e) {
			System.err.print(e.toString());
			return null;
		}
	}
	
	public boolean setH(double h) {
		this.h = h;
		return true;
	}
	public double getH() {
		return this.h;
	}
	public boolean setXk(double[] xk) {
		this.xk = xk;
		return true;
	}
	public double[] getXk() {
		return this.xk;
	}
	public boolean setXset(double[] xset) {
		this.xset = xset;
		return true;
	}
	public double[] getXset() {
		return this.xset;
	}
	/* Qc is supposed to be an array [00 01 02; 10 11 12; 20 21 22] */
	public boolean setQc(double[] Qc) {
		this.Qc = Qc;
		return true;
	}
	public double[] getQc() {
		return this.Qc;
	}
	public boolean setRc(double[] Rc) {
		this.Rc = Rc;
		return true;
	}
	public double[] getRc() {
		return this.Rc;
	}
	public boolean setRc2(double[] Rc2) {
		this.Rc2 = Rc2;
		return true;
	}
	public double[] getRc2() {
		return this.Rc2;
	}
	public boolean setFarpe(double[][] farpe) {
		this.Farpe = farpe;
		return true;
	}
	public double[][] getFarpe() {
		return this.Farpe;
	}
	public boolean setFrca(double[][] frca) {
		this.Frca = frca;
		return true;
	}
	public double[][] getFrca() {
		return this.Frca;
	}
	public boolean setCounter(int counter) {
		this.Counter = counter;
		return true;
	}
	public int getCounter() {
		return this.Counter;
	}

	public boolean setFail(int Fail) {
		this.fail = Fail;
		return true;
	}
	public boolean setPE(double PE) {
		this.pe = PE;
		return true;
	}
	public double getPE() {
		return this.pe;
	}
	}