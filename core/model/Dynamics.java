package core.model;

import core.util.*;

public class Dynamics {

	double R = Constants.R;
	double H_r1 = Constants.H_r1;
	double H_r2 = Constants.H_r2;
	double H_r3 = Constants.H_r3;
	double[] T0 = Constants.T0;
	double[] V = Constants.V;
	double ratio = Constants.ratio;
	double k = Constants.k;
	double F1 = Constants.F1;
	double F2 = Constants.F2, F4 = Constants.F4, F6 = Constants.F6;
	double F10 = Constants.F10;
	double Fr = Constants.Fr;
	double Fr1 = Constants.Fr1, Fr2 = Constants.Fr2;
	double CA0 = Constants.CA0, CB0 = Constants.CB0;
	double CC0 = Constants.CC0, CD0 = Constants.CD0;
	double HA0 = Constants.HA0, HB0 = Constants.HB0;
	double HC0 = Constants.HC0, HD0 = Constants.HD0;
	double Tref = Constants.Tref;
	double CpA = Constants.CpA, CpB = Constants.CpB;
	double CpC = Constants.CpC, CpD = Constants.CpD;
	double HvA = Constants.HvA, HvB = Constants.HvB;
	double HvC = Constants.HvC, HvD = Constants.HvD;
	double[] Q = Constants.Q2;
	double h = Constants.h;
	
	public static void main(String[] args){
		Dynamics dyn = new Dynamics();
		Constants cons = new Constants();
		double[] x0 = cons.getInitPoint();
		
		double[] uk = {0, 0, 0, 0, 0};
		double[] uk_f = {0, 0};
		int cc = 1000;
		double[][] x = new double[cc+1][Constants.ODE];
		double[] xk = new double[Constants.ODE];
		double[] t = new double[cc+1];
		for(int i=0; i<Constants.ODE; i++){
			xk[i] = x0[i];
			x[0][i] = x0[i];
		}
		t[0] = 0;
		double h = 1;
		for(int i=0; i<cc; i++){
			xk = dyn.solve(1, xk, uk, uk_f);
			t[i+1] = h*(i+1);
			for(int k=0; k<Constants.ODE; k++){
				x[i+1][k] = xk[k];
			}
		}
		for(int i=0; i<cc+1; i+=cc){
			System.out.print(i+": ");
			for(int k=0; k<Constants.ODE; k++){
				System.out.print("\t"+x[i][k]);
			}
			System.out.println();
		}
		int[] display = {16,17,18,19,20};
		new NicePlot("CSTR1", x, t, display).plot();
		
	}
		
	/* integrate for one step */
	public double[] solve(double[] xk, double[] uk, double[] uk_f){
		double[] retx = new double[xk.length];
		double[] dx = eval(xk, uk, uk_f);
		for(int i=0; i<xk.length; i++){
			retx[i] = xk[i] + dx[i]*h;
		}
		return retx;
	}
	
	/* integrate for nc step */
	public double[] solve(int nc, double[] xk, double[] uk, double[] uk_f){
		double[] xk2 = new double[xk.length];
		for(int i=0; i<xk.length; i++){
			xk2[i] = xk[i];
		}
		for(int j=0; j<nc; j++){
			xk2 = solve(xk2, uk, uk_f);
		}
		return xk2;
	}

	public double[] eval_f(double[] xk){
		double[] uk = {0, 0, 0, 0, 0};
		double[] uk_f = {0, 0};
		double[] dx = eval(xk, uk, uk_f);
		return dx;
	}

	public double[][] eval_g2(double[] xk){
		double[] uk = {0, 0, 0, 0, 0};
		double[][] g2 = new double[2][Constants.ODE];
		double[] dx = eval_f(xk);
		double[] uk_f = {1, 0};
		double[] dx2 = eval(xk, uk, uk_f);
		for(int i=0; i<dx.length; i++){
			g2[0][i] = dx2[i] - dx[i];
		}
		uk_f[0] = 0; uk_f[1] = 1;
		dx2 = eval(xk, uk, uk_f);
		for(int i=0; i<dx.length; i++){
			g2[1][i] = dx2[i] - dx[i];
		}
		return g2;
	}
	
	public double[][] eval_g1(double[] xk){
		double[] uk_f = {0, 0};
		double[] uk = {0, 0, 0, 0, 0};
		double[][] g1 = new double[5][Constants.ODE];
		double[] dx = eval_f(xk);
		uk[0] = 1;
		double[] dx2 = eval(xk, uk, uk_f);
		for(int i=0; i<dx.length; i++){
			g1[0][i] = dx2[i] - dx[i];
		}
		uk[0] = 0; uk[1] = 1;
		dx2 = eval(xk, uk, uk_f);
		for(int i=0; i<dx.length; i++){
			g1[1][i] = dx2[i] - dx[i];
		}
		uk[1] = 0; uk[2] = 1;
		dx2 = eval(xk, uk, uk_f);
		for(int i=0; i<dx.length; i++){
			g1[2][i] = dx2[i] - dx[i];
		}
		uk[2] = 0; uk[3] = 1;
		dx2 = eval(xk, uk, uk_f);
		for(int i=0; i<dx.length; i++){
			g1[3][i] = dx2[i] - dx[i];
		}
		uk[3] = 0; uk[4] = 1;
		dx2 = eval(xk, uk, uk_f);
		for(int i=0; i<dx.length; i++){
			g1[4][i] = dx2[i] - dx[i];
		}
		return g1;
	}

	
	/* integrate for one step with process noise*/
	public double[] solveWithNoise(double[] xk, double[] uk, double[] uk_f, double[] wk){
		double[] retx = new double[xk.length];
		double[] dx = eval(xk, uk, uk_f);
		for(int i=0; i<xk.length; i++){
			retx[i] = xk[i] + (dx[i]+wk[i])*h;
		}
		return retx;
	}

	public double[] eval(double[] x, double[] uknorm, double[] uk_fnorm){
		double[] dx = new double[Constants.ODE];
		double[] uk = new double[uknorm.length];
		double[] uk_f = new double[uk_fnorm.length];
		for(int i=0; i<uk.length; i++){
			uk[i] = uknorm[i]*Constants.umax[i+2];
		}
		for(int i=0; i<uk_f.length; i++){
			uk_f[i] = uk_fnorm[i]*Constants.umax[i];
		}
		/* Flow mass balance */
		double F3 = F1 + F2 + Fr2;
		double F5 = F3 + F4; 
		double F7 = F5 + F6;
		double F9 = Fr1 + F10;
		double F8 = F9 + F7 - Fr;
		/* Separator */
		double[] alpha = {0.0449*x[19]+10, 0.0260*x[19]+10,
				0.0065*x[19]+0.5, 0.0058*x[19]+0.25};
		double TF2FD = alpha[0]*(F7*x[10] + F9*x[20])
					 + alpha[1]*(F7*x[11] + F9*x[21])
					 + alpha[2]*(F7*x[12] + F9*x[22])
					 + alpha[3]*(F7*x[13] + F9*x[23]);
		double temp = F7*x[10] + F7*x[11] + F7*x[12] + F7*x[13] 
		            + F9*x[20] + F9*x[21] + F9*x[22] + F9*x[23];
		double FAr = k*temp*(F7*x[10] + F9*x[20])*alpha[0]/TF2FD;
		double FBr = k*temp*(F7*x[11] + F9*x[21])*alpha[1]/TF2FD;
		double FCr = k*temp*(F7*x[12] + F9*x[22])*alpha[2]/TF2FD;
		double FDr = k*temp*(F7*x[13] + F9*x[23])*alpha[3]/TF2FD;
		double TotV = FAr/CA0 + FBr/CB0 + FCr/CC0 + FDr/CD0;
		double CAr = FAr/TotV, CBr = FBr/TotV, CCr = FCr/TotV, CDr = FDr/TotV;
		/* Reactions */
		double[] kEB = new double[4];
		double[] r1 = new double[3];
		double[] r2 = new double[4];
		kEB[0] = 1.5202e-1*Math.exp(-3.933e3/R/x[4]);
		r1[0] = 0.084*Math.exp(-9502/R/x[4])*Math.pow(x[0],0.32)*Math.pow(x[1],1.5);
		r2[0] = 0.0850*Math.exp(-20643/R/x[4])*Math.pow(x[1],2.5)*Math.pow(x[2],0.5)/(1+kEB[0]*x[3]);
		kEB[1] = 1.5202e-1*Math.exp(-3.933e3/R/x[9]);
		r1[1] = 0.084*Math.exp(-9502/R/x[9])*Math.pow(x[5],0.32)*Math.pow(x[6],1.5);
		r2[1] = 0.085*Math.exp(-20643/R/x[9])*Math.pow(x[6],2.5)*Math.pow(x[7],0.5)/(1+kEB[1]*x[8]);
		kEB[2] = 1.5202e-1*Math.exp(-3.933e3/R/x[14]);
		r1[2] = 0.084*Math.exp(-9502/R/x[14])*Math.pow(x[10],0.32)*Math.pow(x[11],1.5);
		r2[2] = 0.085*Math.exp(-20643/R/x[14])*Math.pow(x[11],2.5)*Math.pow(x[12],0.5)/(1+kEB[2]*x[13]);
		kEB[3] = 1.5202e-1*Math.exp(-3.933e3/R/x[24]);
		double r3 = 2.378e2*Math.exp(-6.128e4/R/x[24])*Math.pow(x[20],1.0218)*x[23]/(1+0.4901*Math.exp(-5.087e4/8.314/x[24])*x[20])*1000/3600;
		r2[3] = 0.085*Math.exp(-20643/R/x[24])*Math.pow(x[21],2.5)*Math.pow(x[22],0.5)/(1+kEB[3]*x[23]);
		// reactor (1)
		dx[0] = (F1*CA0 + Fr2*CAr - F3*x[0])/V[0] - r1[0];
		dx[1] = (F2*CB0 + Fr2*CBr - F3*x[1])/V[0] - r1[0] -r2[0];
		dx[2] = (Fr2*CCr - F3*x[2])/V[0] + r1[0] - r2[0];
		dx[3] = (Fr2*CDr - F3*x[3])/V[0] + r2[0];
		dx[4] = (Q[0] + uk[0] 
		        + F1*CA0*(HA0 + CpA*(T0[0]-Tref)) + F2*CB0*(HB0 + CpB*(T0[1]-Tref)) 
				+ Fr2*CAr*(HA0 + CpA*(x[19]-Tref)) + Fr2*CBr*(HB0 + CpB*(x[19]-Tref)) 
				+ Fr2*CCr*(HC0 + CpC*(x[19]-Tref)) + Fr2*CDr*(HD0 + CpD*(x[19]-Tref))
		        + (-H_r1*r1[0]*V[0]) + (-H_r2*r2[0]*V[0])
		        - F3*x[0]*(HA0 + CpA*(x[4]-Tref)) - F3*x[1]*(HB0 + CpB*(x[4]-Tref))
		        - F3*x[2]*(HC0 + CpC*(x[4]-Tref)) - F3*x[3]*(HD0 + CpD*(x[4]-Tref)))
		        /((x[0]*CpA + x[1]*CpB + x[2]*CpC + x[3]*CpD)*V[0]);
		// reactor (2)
		dx[5] = (F3*x[0] - F5*x[5])/V[1] - r1[1];
		dx[6] = ((F4 + uk_f[0])*CB0 + F3*x[1] - F5*x[6])/V[1] - r1[1] -r2[1];
		dx[7] = (F3*x[2] - F5*x[7])/V[1] + r1[1] - r2[1];
		dx[8] = (F3*x[3] - F5*x[8])/V[1] + r2[1];
		dx[9] = (Q[1] + uk[1] 
		        + (F4+uk_f[0])*CB0*(HB0 + CpB*(T0[1]-Tref)) 
   		        + F3*x[0]*(HA0 + CpA*(x[4]-Tref)) + F3*x[1]*(HB0 + CpB*(x[4]-Tref))
   		        + F3*x[2]*(HC0 + CpC*(x[4]-Tref)) + F3*x[3]*(HD0 + CpD*(x[4]-Tref))
		        + (-H_r1*r1[1]*V[1]) + (-H_r2*r2[1]*V[1])
		        - F5*x[5]*(HA0 + CpA*(x[9]-Tref)) - F5*x[6]*(HB0 + CpB*(x[9]-Tref))
		        - F5*x[7]*(HC0 + CpC*(x[9]-Tref)) - F5*x[8]*(HD0 + CpD*(x[9]-Tref)))
		        /((x[5]*CpA + x[6]*CpB + x[7]*CpC + x[8]*CpD)*V[1]);
		// reactor(3)
		dx[10] = (F5*x[5] - F7*x[10])/V[2] - r1[2];
		dx[11] = ((F6 + uk_f[1])*CB0 + F5*x[6] - F7*x[11])/V[2] - r1[2] -r2[2];
		dx[12] = (F5*x[7] - F7*x[12])/V[2] + r1[2] - r2[2];
		dx[13] = (F5*x[8] - F7*x[13])/V[2] + r2[2];
		dx[14] = (Q[2] + uk[2] 
		        + (F6+uk_f[1])*CB0*(HB0 + CpB*(T0[1]-Tref)) 
   		        + F5*x[5]*(HA0 + CpA*(x[9]-Tref)) + F5*x[6]*(HB0 + CpB*(x[9]-Tref))
   		        + F5*x[7]*(HC0 + CpC*(x[9]-Tref)) + F5*x[8]*(HD0 + CpD*(x[9]-Tref))
		        + (-H_r1*r1[2]*V[2]) + (-H_r2*r2[2]*V[2])
		        - F7*x[10]*(HA0 + CpA*(x[14]-Tref)) - F7*x[11]*(HB0 + CpB*(x[14]-Tref))
		        - F7*x[12]*(HC0 + CpC*(x[14]-Tref)) - F7*x[13]*(HD0 + CpD*(x[14]-Tref)))
		        /((x[10]*CpA + x[11]*CpB + x[12]*CpC + x[13]*CpD)*V[2]);
		// flash separator
		dx[15] = (F7*x[10] + F9*x[20] - Fr*CAr - F8*x[15])/V[3];
		dx[16] = (F7*x[11] + F9*x[21] - Fr*CBr - F8*x[16])/V[3];
		dx[17] = (F7*x[12] + F9*x[22] - Fr*CCr - F8*x[17])/V[3];
		dx[18] = (F7*x[13] + F9*x[23] - Fr*CDr - F8*x[18])/V[3];
		dx[19] = (Q[3] + uk[3] 
		        + F7*x[10]*(HA0 + CpA*(x[14]-Tref)) + F7*x[11]*(HB0 + CpB*(x[14]-Tref))
		        + F7*x[12]*(HC0 + CpC*(x[14]-Tref)) + F7*x[13]*(HD0 + CpD*(x[14]-Tref))
		        + F9*x[20]*(HA0 + CpA*(x[24]-Tref)) + F9*x[21]*(HB0 + CpB*(x[24]-Tref))
		        + F9*x[22]*(HC0 + CpC*(x[24]-Tref)) + F9*x[23]*(HD0 + CpD*(x[24]-Tref))
		        - F8*x[15]*(HA0 + CpA*(x[19]-Tref)) - F8*x[16]*(HB0 + CpB*(x[19]-Tref))
		        - F8*x[17]*(HC0 + CpC*(x[19]-Tref)) - F8*x[18]*(HD0 + CpD*(x[19]-Tref))
		        - FAr*(HA0 + CpA*(x[19]-Tref)) - FBr*(HB0 + CpB*(x[19]-Tref))
		        - FCr*(HC0 + CpC*(x[19]-Tref)) - FDr*(HD0 + CpD*(x[19]-Tref))
		        - FAr*HvA - FBr*HvB - FCr*HvC - FDr*HvD)
		        /((x[15]*CpA + x[16]*CpB + x[17]*CpC + x[18]*CpD)*V[3]); 
		// Transalylation
		dx[20] = (Fr1*CAr - F9*x[20])/V[4] - r3;
		dx[21] = (Fr1*CBr - F9*x[21])/V[4] - r2[3];
		dx[22] = (Fr1*CCr - F9*x[22])/V[4] + 2*r3 - r2[3];
		dx[23] = (Fr1*CDr + F10*CD0 - F9*x[23])/V[4] -r3 + r2[3];
		dx[24] = (Q[4] + uk[4]
		        + F10*CD0*(HD0 + CpD*(T0[2]-Tref))
		        + Fr1*CAr*(HA0 + CpA*(x[19]-Tref)) + Fr1*CBr*(HB0 + CpB*(x[19]-Tref))
		        + Fr1*CCr*(HC0 + CpC*(x[19]-Tref)) + Fr1*CDr*(HD0 + CpD*(x[19]-Tref))
		        + (-H_r3*r3*V[4]) + (-H_r2*r2[3]*V[4])
		        - F9*x[20]*(HA0 + CpA*(x[24]-Tref)) - F9*x[21]*(HB0 + CpB*(x[24]-Tref))
		        - F9*x[22]*(HC0 + CpC*(x[24]-Tref)) - F9*x[23]*(HD0 + CpD*(x[24]-Tref)))
		        /((x[20]*CpA + x[21]*CpB + x[22]*CpC + x[23]*CpD)*V[4]); 
		return dx;
	}
	
	/* input x_setpoint, u_setpoint, and fault value, returns u with new ss setpoint */
	public double[] inveval(double[] x, double[] u, double[] fail){
		double[] dx = new double[Constants.ODE];
		double[] uk = new double[u.length];
		double[] uk_f = new double[u.length];
		for(int i=0; i<uk.length; i++){
			uk[i] = u[i]*Constants.umax[i+2];
		}
		for(int i=0; i<uk_f.length; i++){
			uk_f[i] = u[i]*Constants.umax[i];
		}
		/* Flow mass balance */
		double F3 = F1 + F2 + Fr2;
		double F5 = F3 + F4; 
		double F7 = F5 + F6;
		double F9 = Fr1 + F10;
		double F8 = F9 + F7 - Fr;
		/* Separator */
		double[] alpha = {0.0449*x[19]+10, 0.0260*x[19]+10,
				0.0065*x[19]+0.5, 0.0058*x[19]+0.25};
		double TF2FD = alpha[0]*(F7*x[10] + F9*x[20])
					 + alpha[1]*(F7*x[11] + F9*x[21])
					 + alpha[2]*(F7*x[12] + F9*x[22])
					 + alpha[3]*(F7*x[13] + F9*x[23]);
		double temp = F7*x[10] + F7*x[11] + F7*x[12] + F7*x[13] 
		            + F9*x[20] + F9*x[21] + F9*x[22] + F9*x[23];
		double FAr = k*temp*(F7*x[10] + F9*x[20])*alpha[0]/TF2FD;
		double FBr = k*temp*(F7*x[11] + F9*x[21])*alpha[1]/TF2FD;
		double FCr = k*temp*(F7*x[12] + F9*x[22])*alpha[2]/TF2FD;
		double FDr = k*temp*(F7*x[13] + F9*x[23])*alpha[3]/TF2FD;
		double TotV = FAr/CA0 + FBr/CB0 + FCr/CC0 + FDr/CD0;
		double CAr = FAr/TotV, CBr = FBr/TotV, CCr = FCr/TotV, CDr = FDr/TotV;
		/* Reactions */
		double[] kEB = new double[4];
		double[] r1 = new double[3];
		double[] r2 = new double[4];
		kEB[0] = 1.5202e-1*Math.exp(-3.933e3/R/x[4]);
		r1[0] = 0.084*Math.exp(-9502/R/x[4])*Math.pow(x[0],0.32)*Math.pow(x[1],1.5);
		r2[0] = 0.0850*Math.exp(-20643/R/x[4])*Math.pow(x[1],2.5)*Math.pow(x[2],0.5)/(1+kEB[0]*x[3]);
		kEB[1] = 1.5202e-1*Math.exp(-3.933e3/R/x[9]);
		r1[1] = 0.084*Math.exp(-9502/R/x[9])*Math.pow(x[5],0.32)*Math.pow(x[6],1.5);
		r2[1] = 0.085*Math.exp(-20643/R/x[9])*Math.pow(x[6],2.5)*Math.pow(x[7],0.5)/(1+kEB[1]*x[8]);
		kEB[2] = 1.5202e-1*Math.exp(-3.933e3/R/x[14]);
		r1[2] = 0.084*Math.exp(-9502/R/x[14])*Math.pow(x[10],0.32)*Math.pow(x[11],1.5);
		r2[2] = 0.085*Math.exp(-20643/R/x[14])*Math.pow(x[11],2.5)*Math.pow(x[12],0.5)/(1+kEB[2]*x[13]);
		kEB[3] = 1.5202e-1*Math.exp(-3.933e3/R/x[24]);
		double r3 = 2.378e2*Math.exp(-6.128e4/R/x[24])*Math.pow(x[20],1.0218)*x[23]/(1+0.4901*Math.exp(-5.087e4/8.314/x[24])*x[20])*1000/3600;
		r2[3] = 0.085*Math.exp(-20643/R/x[24])*Math.pow(x[21],2.5)*Math.pow(x[22],0.5)/(1+kEB[3]*x[23]);
		// reactor (1)
		dx[0] = (F1*CA0 + Fr2*CAr - F3*x[0])/V[0] - r1[0];
		dx[1] = (F2*CB0 + Fr2*CBr - F3*x[1])/V[0] - r1[0] -r2[0];
		dx[2] = (Fr2*CCr - F3*x[2])/V[0] + r1[0] - r2[0];
		dx[3] = (Fr2*CDr - F3*x[3])/V[0] + r2[0];
		dx[4] = (Q[0] + uk[0] 
		        + F1*CA0*(HA0 + CpA*(T0[0]-Tref)) + F2*CB0*(HB0 + CpB*(T0[1]-Tref)) 
				+ Fr2*CAr*(HA0 + CpA*(x[19]-Tref)) + Fr2*CBr*(HB0 + CpB*(x[19]-Tref)) 
				+ Fr2*CCr*(HC0 + CpC*(x[19]-Tref)) + Fr2*CDr*(HD0 + CpD*(x[19]-Tref))
		        + (-H_r1*r1[0]*V[0]) + (-H_r2*r2[0]*V[0])
		        - F3*x[0]*(HA0 + CpA*(x[4]-Tref)) - F3*x[1]*(HB0 + CpB*(x[4]-Tref))
		        - F3*x[2]*(HC0 + CpC*(x[4]-Tref)) - F3*x[3]*(HD0 + CpD*(x[4]-Tref)))
		        /((x[0]*CpA + x[1]*CpB + x[2]*CpC + x[3]*CpD)*V[0]);
		// reactor (2)
		dx[5] = (F3*x[0] - F5*x[5])/V[1] - r1[1];
		dx[6] = ((F4 + uk_f[0])*CB0 + F3*x[1] - F5*x[6])/V[1] - r1[1] -r2[1];
		dx[7] = (F3*x[2] - F5*x[7])/V[1] + r1[1] - r2[1];
		dx[8] = (F3*x[3] - F5*x[8])/V[1] + r2[1];
		dx[9] = (Q[1] + uk[1] 
		        + (F4+uk_f[0])*CB0*(HB0 + CpB*(T0[1]-Tref)) 
   		        + F3*x[0]*(HA0 + CpA*(x[4]-Tref)) + F3*x[1]*(HB0 + CpB*(x[4]-Tref))
   		        + F3*x[2]*(HC0 + CpC*(x[4]-Tref)) + F3*x[3]*(HD0 + CpD*(x[4]-Tref))
		        + (-H_r1*r1[1]*V[1]) + (-H_r2*r2[1]*V[1])
		        - F5*x[5]*(HA0 + CpA*(x[9]-Tref)) - F5*x[6]*(HB0 + CpB*(x[9]-Tref))
		        - F5*x[7]*(HC0 + CpC*(x[9]-Tref)) - F5*x[8]*(HD0 + CpD*(x[9]-Tref)))
		        /((x[5]*CpA + x[6]*CpB + x[7]*CpC + x[8]*CpD)*V[1]);
		// reactor(3)
		dx[10] = (F5*x[5] - F7*x[10])/V[2] - r1[2];
		dx[11] = ((F6 + uk_f[1])*CB0 + F5*x[6] - F7*x[11])/V[2] - r1[2] -r2[2];
		dx[12] = (F5*x[7] - F7*x[12])/V[2] + r1[2] - r2[2];
		dx[13] = (F5*x[8] - F7*x[13])/V[2] + r2[2];
		dx[14] = (Q[2] + uk[2] 
		        + (F6+uk_f[1])*CB0*(HB0 + CpB*(T0[1]-Tref)) 
   		        + F5*x[5]*(HA0 + CpA*(x[9]-Tref)) + F5*x[6]*(HB0 + CpB*(x[9]-Tref))
   		        + F5*x[7]*(HC0 + CpC*(x[9]-Tref)) + F5*x[8]*(HD0 + CpD*(x[9]-Tref))
		        + (-H_r1*r1[2]*V[2]) + (-H_r2*r2[2]*V[2])
		        - F7*x[10]*(HA0 + CpA*(x[14]-Tref)) - F7*x[11]*(HB0 + CpB*(x[14]-Tref))
		        - F7*x[12]*(HC0 + CpC*(x[14]-Tref)) - F7*x[13]*(HD0 + CpD*(x[14]-Tref)))
		        /((x[10]*CpA + x[11]*CpB + x[12]*CpC + x[13]*CpD)*V[2]);
		// flash separator
		dx[15] = (F7*x[10] + F9*x[20] - Fr*CAr - F8*x[15])/V[3];
		dx[16] = (F7*x[11] + F9*x[21] - Fr*CBr - F8*x[16])/V[3];
		dx[17] = (F7*x[12] + F9*x[22] - Fr*CCr - F8*x[17])/V[3];
		dx[18] = (F7*x[13] + F9*x[23] - Fr*CDr - F8*x[18])/V[3];
		dx[19] = (Q[3] + uk[3] 
		        + F7*x[10]*(HA0 + CpA*(x[14]-Tref)) + F7*x[11]*(HB0 + CpB*(x[14]-Tref))
		        + F7*x[12]*(HC0 + CpC*(x[14]-Tref)) + F7*x[13]*(HD0 + CpD*(x[14]-Tref))
		        + F9*x[20]*(HA0 + CpA*(x[24]-Tref)) + F9*x[21]*(HB0 + CpB*(x[24]-Tref))
		        + F9*x[22]*(HC0 + CpC*(x[24]-Tref)) + F9*x[23]*(HD0 + CpD*(x[24]-Tref))
		        - F8*x[15]*(HA0 + CpA*(x[19]-Tref)) - F8*x[16]*(HB0 + CpB*(x[19]-Tref))
		        - F8*x[17]*(HC0 + CpC*(x[19]-Tref)) - F8*x[18]*(HD0 + CpD*(x[19]-Tref))
		        - FAr*(HA0 + CpA*(x[19]-Tref)) - FBr*(HB0 + CpB*(x[19]-Tref))
		        - FCr*(HC0 + CpC*(x[19]-Tref)) - FDr*(HD0 + CpD*(x[19]-Tref))
		        - FAr*HvA - FBr*HvB - FCr*HvC - FDr*HvD)
		        /((x[15]*CpA + x[16]*CpB + x[17]*CpC + x[18]*CpD)*V[3]); 
		// Transalylation
		dx[20] = (Fr1*CAr - F9*x[20])/V[4] - r3;
		dx[21] = (Fr1*CBr - F9*x[21])/V[4] - r2[3];
		dx[22] = (Fr1*CCr - F9*x[22])/V[4] + 2*r3 - r2[3];
		dx[23] = (Fr1*CDr + F10*CD0 - F9*x[23])/V[4] -r3 + r2[3];
		dx[24] = (Q[4] + uk[4]
		        + F10*CD0*(HD0 + CpD*(T0[2]-Tref))
		        + Fr1*CAr*(HA0 + CpA*(x[19]-Tref)) + Fr1*CBr*(HB0 + CpB*(x[19]-Tref))
		        + Fr1*CCr*(HC0 + CpC*(x[19]-Tref)) + Fr1*CDr*(HD0 + CpD*(x[19]-Tref))
		        + (-H_r3*r3*V[4]) + (-H_r2*r2[3]*V[4])
		        - F9*x[20]*(HA0 + CpA*(x[24]-Tref)) - F9*x[21]*(HB0 + CpB*(x[24]-Tref))
		        - F9*x[22]*(HC0 + CpC*(x[24]-Tref)) - F9*x[23]*(HD0 + CpD*(x[24]-Tref)))
		        /((x[20]*CpA + x[21]*CpB + x[22]*CpC + x[23]*CpD)*V[4]); 
		return dx;
	}
	
	public boolean setQ(double[] Qnew) {
		this.Q = Qnew;
		return true;
	}
	public double[] getQ() {
		return this.Q;
	}
	public boolean setF4(double F4new) {
		this.F4 = F4new;
		return true;
	}
	public double getF4() {
		return this.F4;
	}
	public boolean setF6(double F6new) {
		this.F6 = F6new;
		return true;
	}
	public double getF6() {
		return this.F6;
	}
}
