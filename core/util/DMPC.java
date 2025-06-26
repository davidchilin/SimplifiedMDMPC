package core.util;

import org.coinor.Ipopt;

import core.model.Constants;
import core.model.Dynamics;

public class DMPC extends Ipopt{
	private int n, m, nele_jac, nele_hess;
	private double[] xk, xset;
	private double[] Qc = Constants.Qc;
	private double[] Rc = Constants.Rc;
	private double[] Rc2 = Constants.Rc2;
	private double[] Pv = Constants.Pv; // Note: diagonal elements only
	private int nc;
	Dynamics dyn;

	private int MPC_ID = 0; // MPC_ID = 1, MPC~1; MPC_ID = 2, MPC~2; MPC_ID = 3, MPC~3;
	private double[][] u_prev;
	private double[] u_ref;

	/* n: dimension of the control inputs; m: number of constraints */
	/* u_L, u_U: bounds on control inputs; g_L, g_U: constraints */
	public DMPC(int n, int m, double[] u_L, double[] u_U, double[] g_L, double[] g_U){
		this.n = n; this.m = m;	nele_jac = m*n;	nele_hess = n*(n+1)/2;
		create(this.n, u_L, u_U, this.m, g_L, g_U, nele_jac, nele_hess, Ipopt.C_STYLE);
		dyn = new Dynamics();
	}

	/* modify this function to write the correct objective function */
	protected boolean eval_f(int n, double[] u, boolean new_x, double[] obj_value) {
		double[] xkk = new double[Constants.ODE];
		for(int i=0; i<Constants.ODE; i++){
			xkk[i] = this.xk[i];
		}
		double[] xset = this.xset;
		double[] Qc = this.Qc;
		double[] Rc = this.Rc;
		double[] Rc2 = this.Rc2;
		double cost = 0;

		for(int i=0; i<xkk.length; i++){
			cost = cost + (xkk[i]-xset[i])*Qc[i]*(xkk[i]-xset[i]);
		}
		int u_num = 2;
		if(MPC_ID == 1){
			u_num = 2;
		}else if(MPC_ID == 2){
			u_num = 3;
		}else if(MPC_ID == 3){
			u_num = 2;
		}else{
			System.out.println("Error: MPC_ID has not been set!");
		}
		double[] uk_f = new double[2];
		double[] uk = new double[5];
		for(int k=0; k<n/u_num; k++){
			if(MPC_ID == 1){
				for(int l=0; l<2; l++){
					uk_f[l] = u[k*2+l];
				}
				for(int l=0; l<5; l++){
					uk[l] = u_prev[k][2+l];
				}
			}else if(MPC_ID == 2){
				for(int l=0; l<2; l++){
					uk_f[l] = u_prev[k][l];
				}
				for(int l=0; l<3; l++){
					uk[l] = u[k*3+l];
				}
				for(int l=3; l<5; l++){
					uk[l] = u_prev[k][l+2];
				}
			}else if(MPC_ID == 3){
				for(int l=0; l<2; l++){
					uk_f[l] = u_prev[k][l];
				}
				for(int l=0; l<3; l++){
					uk[l] = u_prev[k][2+l];
				}
				for(int l=0; l<2; l++){
					uk[l+3] = u[k*2+l];
				}
			}
			// Note: only update flow rate T related to reactors to speed up calculation
			xkk = dyn.solve(nc, xkk, uk, uk_f);
			for(int j=0; j<xkk.length; j++){
				cost = cost + (xkk[j]-xset[j])*Qc[j]*(xkk[j]-xset[j]);
			}
			for(int i=0; i<2; i++){
				cost = cost + uk_f[i]*Constants.umax[i]*Rc2[i]*uk_f[i]*Constants.umax[i];
			}
			for(int i=0; i<5; i++){
				cost = cost + uk[i]*Constants.umax[i+2]*Rc[i]*uk[i]*Constants.umax[i+2];
			}
		}
		obj_value[0] = cost;
		return true;
	}

	/* modify this function to set the inequality or equality constraints */
	protected boolean eval_g(int n, double[] u, boolean new_x, int m, double[] g) {

		double[] xkk = new double[Constants.ODE];
		double[] xkk2 = new double[Constants.ODE];
		for(int i=0; i<Constants.ODE; i++){
			xkk[i] = this.xk[i];
			xkk2[i] = this.xk[i];
		}
		double[] uk_f = new double[2];
		double[] uk = new double[5];

		if(MPC_ID == 1){
			for(int l=0; l<2; l++){
				uk_f[l] = u[l];
			}
			for(int l=0; l<5; l++){
				uk[l] = u_prev[0][2+l];
			}
		}else if(MPC_ID == 2){
			for(int l=0; l<2; l++){
				uk_f[l] = u_prev[0][l];
			}
			for(int l=0; l<3; l++){
				uk[l] = u[l];
			}
			for(int l=3; l<5; l++){
				uk[l] = u_prev[0][l+2];
			}
		}else if(MPC_ID == 3){
			for(int l=0; l<2; l++){
				uk_f[l] = u_prev[0][l];
			}
			for(int l=0; l<3; l++){
				uk[l] = u_prev[0][2+l];
			}
			for(int l=0; l<2; l++){
				uk[l+3] = u[l];
			}
		}
		xkk = dyn.solve(nc, xkk, uk, uk_f);
		double[] uk2 = new double[5];
		double[] uk_f2 = new double[2];
		for(int i=0; i<2; i++){
			uk_f2[i] = u_ref[i];
		}
		for(int i=0; i<5; i++){
			uk2[i] = u_ref[i+2];
		}
		xkk2 = dyn.solve(nc, xkk2, uk2, uk_f2);
		double V=0, V2=0;
		for(int j=0; j<xkk.length; j++){
			V = V + (xkk[j]-xset[j])*Pv[j]*(xkk[j]-xset[j]);
			V2 = V2 + (xkk2[j]-xset[j])*Pv[j]*(xkk2[j]-xset[j]);
		}
		g[0] = V - V2;
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
	public boolean setMPCID(int i){
		this.MPC_ID = i;
		return true;
	}
	public int getMPCID(){
		return this.MPC_ID;
	}
	public boolean setUPrev(double[][] u_prev){
		this.u_prev = u_prev;
		return true;
	}
	public double[][] getUPrev(){
		return this.u_prev;
	}
	public boolean setURef(double[] u_ref){
		this.u_ref = u_ref;
		return true;
	}
	public double[] getURef(){
		return this.u_ref;
	}
}
