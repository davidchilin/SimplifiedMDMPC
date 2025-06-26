// SeqMPCControl_FDI2 - after fault detection, reconfigure mpc to work with persistent fault

package core;

import java.io.BufferedWriter;
import java.util.Date;

import core.model.Constants;
import core.model.Dynamics;
import core.util.DMPC;
import core.util.FileIO;
import core.util.GenerateBoundedNoise;
import core.util.PlotIt6;
import core.util.Sontag;
import core.util.Stuff;

public class SeqMPCControl_FDI3 {
	public static void main(String[] args) {
		/* Set up initial state and final state */
		Constants cons = new Constants();
		double[] xset = new double[Constants.ODE];
		double[] xk = new double[Constants.ODE];
		xk = cons.getInitPoint(); // initial point
		xset = cons.getSetPoint(); // set point

		/* Set up simulation environment */
		int cc = 1900; // simulation time
		int lcc = 1900; // cost counter time
		double[] uk = new double[5];
		double[] uk_f = new double[2];
		double[] t = new double[cc + 1];
		int dilute=10; // reduce recorded data by dilute
		double[] tt = new double[(cc+1)/dilute];
		double[][] et = new double[50][3]; // timing
		int cet = 0;
		double[][] x = new double[cc + 1][Constants.ODE];
		double[][] xx = new double[(cc + 1)/dilute][Constants.ODE];
		double[][] u = new double[cc + 1][2 + 5];
		double[][] uu = new double[(cc + 1)/dilute][2 + 5];
		GenerateBoundedNoise gen = new GenerateBoundedNoise();
		gen.setSeed(13);
		double[][] pnoise = gen.generateNoise(cc);
		Dynamics dyn = new Dynamics();
		Sontag son = new Sontag(xset);

		/* Set up MPC */
		int nc = 10, N = 3;
		int coc = 30; // cost increment
		double[] Qc = Constants.Qc, Rc = Constants.Rc, Rc2 = Constants.Rc2;
		double[][] u_prev = new double[N][7];
		
		/* Set up Ipopt */
		int nn1 = 2, nn2 = 3, nn3 = 2, n1 = nn1 * N, n2 = nn2 * N, n3 = nn3 * N, m = 1;
		double[] u1_L = new double[n1], u1_U = new double[n1], u2_L = new double[n2], u2_U = new double[n2], u3_L = new double[n3], u3_U = new double[n3];
		double[] g_L = new double[m], g_U = new double[m], u1_opt = new double[n1], u2_opt = new double[n2], u3_opt = new double[n3];
		for (int i = 0; i < N; i++) {
			u1_L[i * nn1 + 0] = -1;
			u1_U[i * nn1 + 0] = 1;
			u1_L[i * nn1 + 1] = -1;
			u1_U[i * nn1 + 1] = 1;
		}
		for (int i = 0; i < N; i++) {
			u2_L[i * nn2 + 0] = -1;
			u2_U[i * nn2 + 0] = 1;
			u2_L[i * nn2 + 1] = -1;
			u2_U[i * nn2 + 1] = 1;
			u2_L[i * nn2 + 2] = -1;
			u2_U[i * nn2 + 2] = 1;
		}
		for (int i = 0; i < N; i++) {
			u3_L[i * nn3 + 0] = -1;
			u3_U[i * nn3 + 0] = 1;
			u3_L[i * nn3 + 1] = -1;
			u3_U[i * nn3 + 1] = 1;
		}
		for (int i = 0; i < m; i++) {
			g_L[i] = -1e11;
			g_U[i] = 1e-3;
		}
		double step = 1e-4;
		DMPC mpc1 = new DMPC(n1, m, u1_L, u1_U, g_L, g_U); // create IPOPT
		DMPC mpc2 = new DMPC(n2, m, u2_L, u2_U, g_L, g_U); // create IPOPT
		DMPC mpc3 = new DMPC(n3, m, u3_L, u3_U, g_L, g_U); // create IPOPT
		mpc1.setMPCID(1);
		mpc2.setMPCID(2);
		mpc3.setMPCID(3);
		mpc1.setStep(step);
		mpc2.setStep(step);
		mpc3.setStep(step);
		mpc1.addIntOption("max_iter", 400);
		mpc1.addIntOption("print_level", -2);
		mpc1.addStrOption("hessian_approximation", "limited-memory");
		mpc1.addNumOption("tol", step);
		mpc1.addNumOption("dual_inf_tol", step);
		mpc1.addNumOption("acceptable_tol", step);
		mpc2.addIntOption("max_iter", 800);
		mpc2.addIntOption("print_level", -2);
		mpc2.addStrOption("hessian_approximation", "limited-memory");
		mpc2.addNumOption("tol", step);
		mpc2.addNumOption("dual_inf_tol", step);
		mpc2.addNumOption("acceptable_tol", step);
		mpc3.addIntOption("max_iter", 400);
		mpc3.addIntOption("print_level", -2);
		mpc3.addStrOption("hessian_approximation", "limited-memory");
		mpc3.addNumOption("tol", step);
		mpc3.addNumOption("dual_inf_tol", step);
		mpc3.addNumOption("acceptable_tol", step);

		mpc1.setNc(nc);
		mpc1.setXset(xset);
		mpc2.setNc(nc);
		mpc2.setXset(xset);
		mpc3.setNc(nc);
		mpc3.setXset(xset);

		for (int i = 0; i < Constants.ODE; i++) {
			x[0][i] = xk[i];
		}

		// fdi initiate ***************************************
		String[] act_name = { "Q1", "Q2", "Q3", "Q4", "Q5", "F4", "F6", "Error" };
		int[] rdisplay = {4,9,14,19,24,6,11}; // only 7 residuals: Temp(5) + CB2 + CB3
//		int[] rdisplay = new int[Constants.ODE]; // all states residuals
//		for (int i = 0; i < rdisplay.length; i++)
//			rdisplay[i] = i;

		double[][] r = new double[cc + 1][rdisplay.length];
		double[][] rr = new double[(cc+1)/dilute][rdisplay.length];
		int iiii=0;
		double[][] pfu = new double[7][rdisplay.length]; // previous f1u
		double[][] fxk = new double[Constants.ODE][rdisplay.length]; // current filter state
		double[][][] fu_prev = new double[N][7][rdisplay.length];
		double[][] fu1_opt = new double[n1][rdisplay.length], fu2_opt = new double[n2][rdisplay.length], fu3_opt = new double[n3][rdisplay.length];
		double[][] fuk = new double[5][rdisplay.length];
		double[][] fuk_f = new double[2][rdisplay.length];

		for (int z = 0; z < rdisplay.length; z++)
			Stuff.Equal(xk, fxk, z); // initialize fxk
		
		String xfile = "./data/x.data";
		String ufile = "./data/u.data";
		String rfile = "./data/r"+rdisplay.length+".data";

		DMPC[] fmpc1 = new DMPC[rdisplay.length];
		DMPC[] fmpc2 = new DMPC[rdisplay.length];
		DMPC[] fmpc3 = new DMPC[rdisplay.length];

		for (int z = 0; z < rdisplay.length; z++) // fdi
		{
			fmpc1[z] = new DMPC(n1, m, u1_L, u1_U, g_L, g_U);
			fmpc2[z] = new DMPC(n2, m, u2_L, u2_U, g_L, g_U);
			fmpc3[z] = new DMPC(n3, m, u3_L, u3_U, g_L, g_U);

			fmpc1[z].setMPCID(1);
			fmpc2[z].setMPCID(2);
			fmpc3[z].setMPCID(3);

			fmpc1[z].setStep(step);
			fmpc2[z].setStep(step);
			fmpc3[z].setStep(step);
			fmpc1[z].addIntOption("max_iter", 400);
			fmpc1[z].addIntOption("print_level", -2);
			fmpc1[z].addStrOption("hessian_approximation", "limited-memory");
			fmpc1[z].addNumOption("tol", step);
			fmpc1[z].addNumOption("dual_inf_tol", step);
			fmpc1[z].addNumOption("acceptable_tol", step);
			fmpc2[z].addIntOption("max_iter", 800);
			fmpc2[z].addIntOption("print_level", -2);
			fmpc2[z].addStrOption("hessian_approximation", "limited-memory");
			fmpc2[z].addNumOption("tol", step);
			fmpc2[z].addNumOption("dual_inf_tol", step);
			fmpc2[z].addNumOption("acceptable_tol", step);
			fmpc3[z].addIntOption("max_iter", 400);
			fmpc3[z].addIntOption("print_level", -2);
			fmpc3[z].addStrOption("hessian_approximation", "limited-memory");
			fmpc3[z].addNumOption("tol", step);
			fmpc3[z].addNumOption("dual_inf_tol", step);
			fmpc3[z].addNumOption("acceptable_tol", step);

			fmpc1[z].setNc(nc);
			fmpc1[z].setXset(xset);
			fmpc2[z].setNc(nc);
			fmpc2[z].setXset(xset);
			fmpc3[z].setNc(nc);
			fmpc3[z].setXset(xset);
		}

		double[] rlimits = new double[rdisplay.length];

		int ii=0;
		for (int i : rdisplay) {
			rlimits[ii++] = Constants.average[i] + 3 * Constants.stdev[i];
		}

		int[][] isolate = { { 0, 0 }, { 0, 0 } }; // first index r to exceed
													// threshold, second is
													// counter
		int isolated = 10; // minimum time to confirm a problem
		int detected = -1; // fault detection status

		// end fdi initiate ***************************************
		t[0] = 0;

		try {
			BufferedWriter out = new FileIO().getFileBufferedWriter(xfile);
			BufferedWriter uout = new FileIO().getFileBufferedWriter(ufile);
			BufferedWriter rout = new FileIO().getFileBufferedWriter(rfile);

			String firstline = t[0] + "";
			for (int j = 0; j < xk.length; j++) {
				firstline = firstline + "\t" + xk[j];
			}
			out.write(firstline);
			out.newLine();
			out.flush();

			double[][] uk_prev;
			double[][][] fuk_prev;
			double[] u10 = new double[nn1 * N];
			double[][] fu10 = new double[nn1 * N][rdisplay.length]; // fdi
			double[] u20 = new double[nn2 * N];
			double[][] fu20 = new double[nn2 * N][rdisplay.length]; // fdi
			double[] u30 = new double[nn3 * N];
			double[][] fu30 = new double[nn3 * N][rdisplay.length]; // fdi
			double temp = 0;
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			for (int i = 0; i < cc; i++) {
				// fault detection
				if (detected == -1) {// if not detected - perform FDI
					ii=0;
					for (int j : rdisplay) {
						if (r[i][ii] > rlimits[ii++]) { // residual exceed threshold
							System.out.print("Residual: " + j
									+ " exceeds threshold at time " + i + "\n");

							if (isolate[0][1] == 0) // if isolate[0][1] not set, update
							{
								isolate[0][0] = j;
								isolate[0][1]++;
							} else if (isolate[1][1] == 0) // if isolate[1][1] not set, update
							{
								isolate[1][0] = j;
								isolate[1][1]++;
							} else if (isolate[0][0] == j) // if already set check if = previous r and update counter
								isolate[0][1]++;
							else if (isolate[1][0] == j)
								isolate[1][1]++;
						} else // if residual dips below thresholds, reset if r in isolate.
						{
							if (isolate[0][0] == j) {
								isolate[0][0] = isolate[1][0];
								isolate[0][1] = isolate[1][1];
								isolate[1][0] = 0;
								isolate[1][1] = 0;
							} else if (isolate[1][0] == j) {
								isolate[1][0] = 0;
								isolate[1][1] = 0;
							}
						}

						if (detected == -1
								&& (isolate[0][1] > isolated || isolate[1][1] > isolated)) {
							System.out.print("Fault detected in residual: " + j
									+ "\n");
							detected = j; // set detected equal to actuator with fault &&&
						}
					} // for(int j=0;j<Constants.ODE;j++)
				}  // end fault detection
				
				if (i % nc == 0) {

					uk_prev = son.predict(xk, N, nc);
					fuk_prev = new double[N][5][rdisplay.length]; // hold only Q1-Q5

					for (int z = 0; z < rdisplay.length; z++) { // fdi
						Stuff.Equal(son.predict(Stuff.Equal(fxk, z), N, nc), fuk_prev, z); // fdi
					}

					for (int l = 0; l < N; l++) {
						for (int k = 2; k < 7; k++) {
							u_prev[l][k] = uk_prev[l][k - 2];
							fu_prev[l][k] = fuk_prev[l][k - 2]; // fdi
						}
					}
					mpc1.setUPrev(u_prev);
					mpc1.setXk(xk);
					mpc1.setURef(u_prev[0]);
					for (int z = 0; z < rdisplay.length; z++) {// fdi
						fmpc1[z].setUPrev(Stuff.Equal(fu_prev, z)); // fdi
						fmpc1[z].setXk(Stuff.Equal(fxk, z)); // fdi
						fmpc1[z].setURef(Stuff.Equal(fu_prev[0], z)); // fdi
					}
					for (int l = 0; l < N; l++) {
						u10[l * nn1 + 0] = u_prev[l][0];
						u10[l * nn1 + 1] = u_prev[l][1];
						fu10[l * nn1 + 0] = fu_prev[l][0]; // fdi
						fu10[l * nn1 + 1] = fu_prev[l][1]; // fdi
					}
					System.out.print(i + " : status MPC1: ");
					long d1 = new Date().getTime();
					u1_opt = mpc1.solve(u10);
					for (int z = 0; z < rdisplay.length; z++)
						Stuff.Equal(fmpc1[z].solve(Stuff.Equal(fu10, z)), fu1_opt, z); // fdi
					long d2 = new Date().getTime();
					System.out.print("" + mpc1.getStatus() + ", ");
					if (cet < 50) { // ?
						et[cet][0] = d2 - d1;
					}

					for (int l = 0; l < N; l++) {
						u_prev[l][0] = u1_opt[l * nn1 + 0];
						u_prev[l][1] = u1_opt[l * nn1 + 1];
						fu_prev[l][0] = fu1_opt[l * nn1 + 0]; // fdi
						fu_prev[l][1] = fu1_opt[l * nn1 + 1]; // fdi
					}
					mpc2.setUPrev(u_prev);
					mpc2.setXk(xk);
					mpc2.setURef(u_prev[0]);
					for (int z = 0; z < rdisplay.length; z++) {// fdi
						fmpc2[z].setUPrev(Stuff.Equal(fu_prev, z)); // fdi
						fmpc2[z].setXk(Stuff.Equal(fxk, z)); // fdi
						fmpc2[z].setURef(Stuff.Equal(fu_prev[0], z)); // fdi
					}
					for (int s = 0; s < N; s++) {
						u20[s * nn2 + 0] = u_prev[s][2];
						u20[s * nn2 + 1] = u_prev[s][3];
						u20[s * nn2 + 2] = u_prev[s][4];
						fu20[s * nn2 + 0] = fu_prev[s][2]; // fdi
						fu20[s * nn2 + 1] = fu_prev[s][3]; // fdi
						fu20[s * nn2 + 2] = fu_prev[s][4]; // fdi
					}
					System.out.print("MPC2: ");
					long d3 = new Date().getTime();
					u2_opt = mpc2.solve(u20);
					for (int z = 0; z < rdisplay.length; z++) // fdi
						Stuff.Equal(fmpc2[z].solve(Stuff.Equal(fu20, z)), fu2_opt, z); // fdi
					long d4 = new Date().getTime();
					System.out.print("" + mpc2.getStatus() + ", ");
					if (cet < 50) {
						et[cet][1] = d4 - d3;
					}

					for (int l = 0; l < N; l++) {
						u_prev[l][2] = u2_opt[l * nn2 + 0];
						u_prev[l][3] = u2_opt[l * nn2 + 1];
						u_prev[l][4] = u2_opt[l * nn2 + 2];
						fu_prev[l][2] = fu2_opt[l * nn2 + 0]; // fdi
						fu_prev[l][3] = fu2_opt[l * nn2 + 1]; // fdi
						fu_prev[l][4] = fu2_opt[l * nn2 + 2]; // fdi
					}
					mpc3.setUPrev(u_prev);
					mpc3.setXk(xk);
					mpc3.setURef(u_prev[0]);
					for (int z = 0; z < rdisplay.length; z++) {// fdi
						fmpc3[z].setUPrev(Stuff.Equal(fu_prev, z)); // fdi
						fmpc3[z].setXk(Stuff.Equal(fxk, z)); // fdi
						fmpc3[z].setURef(Stuff.Equal(fu_prev[0], z)); // fdi
					}
					for (int l = 0; l < N; l++) {
						u30[l * nn3 + 0] = u_prev[l][5];
						u30[l * nn3 + 1] = u_prev[l][6];
						fu30[l * nn3 + 0] = fu_prev[l][5]; // fdi
						fu30[l * nn3 + 1] = fu_prev[l][6]; // fdi
					}
					System.out.print("MPC3: ");
					long d5 = new Date().getTime();
					u3_opt = mpc3.solve(u30);
					for (int z = 0; z < rdisplay.length; z++) // fdi
						Stuff.Equal(fmpc3[z].solve(Stuff.Equal(fu30, z)), fu3_opt, z); // fdi
					long d6 = new Date().getTime();
					System.out.println("" + mpc3.getStatus());
					if (cet < 50) {
						et[cet][2] = d6 - d5;
					}
					cet = cet + 1;

					for (int l = 0; l < N; l++) {
						u_prev[l][5] = u3_opt[0 + nn3 * l];
						u_prev[l][6] = u3_opt[1 + nn3 * l];
						fu_prev[l][5] = fu3_opt[0 + nn3 * l]; // fdi
						fu_prev[l][6] = fu3_opt[1 + nn3 * l]; // fdi
					}
					for (int l = 0; l < nn1; l++) {
						uk_f[l] = u1_opt[l];
						fuk_f[l] = fu1_opt[l]; // fdi
					}
					for (int l = 0; l < nn2; l++) {
						uk[l] = u2_opt[l];
						fuk[l] = fu2_opt[l]; // fdi
					}
					for (int l = nn2; l < 5; l++) {
						uk[l] = u3_opt[l - nn2];
						fuk[l] = fu3_opt[l - nn2]; // fdi
					}
				} else {
					for (int k = 0; k < 2; k++) {
						uk_f[k] = u[i - 1][k];
						fuk_f[k] = pfu[k]; // fdi
					}
					for (int k = 0; k < 5; k++) {
						uk[k] = u[i - 1][k + 2];
						fuk[k] = pfu[k + 2]; // fdi
					}
				}

				if (detected == -1) { // if fault is not isolated, continue with 
					if (i > cc / 2) { // initiate actuator fault here ^^^
						uk_f[1] = 0.5;
						//uk[1]=0.5;
					}
				}
				// initiate reconfigration here
				else if (detected != -2) {// if detected is set to faulty actuator, reset fault
					switch (detected) {
					// mpc2
					case 4:
						detected = 0;
						for (int iii = 0; iii < N; iii++) {
							u2_L[iii * nn2 + 0] = uk[0];
							u2_U[iii * nn2 + 0] = uk[0];
						}
						mpc2 = new DMPC(n2, m, u2_L, u2_U, g_L, g_U); // Q1/Q2/Q3 uk[0,1,2]
						break;
					case 9:
						detected = 1;
						for (int iii = 0; iii < N; iii++) {
							u2_L[iii * nn2 + 1] = uk[1];
							u2_U[iii * nn2 + 1] = uk[1];
						}
						mpc2 = new DMPC(n2, m, u2_L, u2_U, g_L, g_U); // Q1/Q2/Q3 uk[0,1,2]
						break;
					case 14:
						detected = 2;
						for (int iii = 0; iii < N; iii++) {
							u2_L[iii * nn2 + 2] = uk[2];
							u2_U[iii * nn2 + 2] = uk[2];
						}
						mpc2 = new DMPC(n2, m, u2_L, u2_U, g_L, g_U); // Q1/Q2/Q3 uk[0,1,2]
						break;
					// mpc3
					case 19:
						detected = 3;
						for (int iii = 0; i < N; iii++) {
							u3_L[iii * nn3 + 0] = uk[3];
							u3_U[iii * nn3 + 0] = uk[3];
						}
						mpc3 = new DMPC(n3, m, u3_L, u3_U, g_L, g_U); // Q4/Q5 uk[3,4]
						break;
					case 24:
						detected = 4;
						for (int iii = 0; i < N; iii++) {
							u3_L[iii * nn3 + 1] = uk[4];
							u3_U[iii * nn3 + 1] = uk[4];
						}
						mpc3 = new DMPC(n3, m, u3_L, u3_U, g_L, g_U); // Q4/Q5 uk[3,4]
						break;
					// mpc1
					case 6:
						detected = 5;
						for (int iii = 0; iii < N; iii++) {
							u1_L[iii * nn1 + 0] = uk_f[0];
							u1_U[iii * nn1 + 0] = uk_f[0];
						}
						mpc1 = new DMPC(n1, m, u1_L, u1_U, g_L, g_U); // F4/F6 uk_f[0,1]
						break;
					case 11:
						detected = 6;
						for (int iii = 0; iii < N; iii++) {
							u1_L[iii * nn1 + 1] = uk_f[1];
							u1_U[iii * nn1 + 1] = uk_f[1];
						}
						mpc1 = new DMPC(n1, m, u1_L, u1_U, g_L, g_U); // F4/F6 uk_f[0,1]
						break;
					default:
						detected = 7;
					}
					System.out.print("Fault isolated/reconfigured: " + act_name[detected]
							+ "\n");
					detected = -2;
					
					mpc1.setMPCID(1);
					mpc2.setMPCID(2);
					mpc3.setMPCID(3);
					mpc1.setStep(step);
					mpc2.setStep(step);
					mpc3.setStep(step);
					mpc1.addIntOption("max_iter", 400);
					mpc1.addIntOption("print_level", -2);
					mpc1.addStrOption("hessian_approximation", "limited-memory");
					mpc1.addNumOption("tol", step);
					mpc1.addNumOption("dual_inf_tol", step);
					mpc1.addNumOption("acceptable_tol", step);
					mpc2.addIntOption("max_iter", 800);
					mpc2.addIntOption("print_level", -2);
					mpc2.addStrOption("hessian_approximation", "limited-memory");
					mpc2.addNumOption("tol", step);
					mpc2.addNumOption("dual_inf_tol", step);
					mpc2.addNumOption("acceptable_tol", step);
					mpc3.addIntOption("max_iter", 400);
					mpc3.addIntOption("print_level", -2);
					mpc3.addStrOption("hessian_approximation", "limited-memory");
					mpc3.addNumOption("tol", step);
					mpc3.addNumOption("dual_inf_tol", step);
					mpc3.addNumOption("acceptable_tol", step);

					mpc1.setNc(nc);
					mpc1.setXset(xset);
					mpc2.setNc(nc);
					mpc2.setXset(xset);
					mpc3.setNc(nc);
					mpc3.setXset(xset);

				}

				for (int k = 0; k < 2; k++) {
					u[i][k] = uk_f[k];
					pfu[k] = fuk_f[k]; // fdi
				}
				for (int k = 0; k < 5; k++) {
					u[i][k + 2] = uk[k];
					pfu[k + 2] = fuk[k]; // fdi
				}
				
				if(i%dilute==0) {
				String uline = t[i] + "";
				for (int k = 0; k < 2; k++) {
					uline = uline + "\t" + uk_f[k];
				}
				for (int k = 0; k < 5; k++) {
					uline = uline + "\t" + uk[k];
				}
				for (int k=0;k<7;k++)
					uu[iiii][k]=u[i][k];
				uout.write(uline);
				uout.newLine();
				uout.flush();
				}
				
				//xk = dyn.solve(xk, uk, uk_f); // no noise
				xk = dyn.solveWithNoise(xk, uk, uk_f, pnoise[i]);
				for (int z = 0; z < rdisplay.length; z++) 	// fdi
					Stuff.Equal(dyn.solve(Stuff.Equal(fxk, z), Stuff.Equal(fuk, z), Stuff.Equal(fuk_f, z)), fxk, z); // fdi
				for (int k = 0; k < Constants.ODE; k++) { // record keeping
					x[i + 1][k] = xk[k];
				}

				for(int z=0; z<rdisplay.length; z++){ // record keeping
				 r[i+1][z]= xk[rdisplay[z]]-fxk[rdisplay[z]][z]; // fdi
				 if(r[i+1][z]<0) r[i+1][z]=-r[i+1][z];
				}
				
				if(i%dilute==0) {
				String rline = t[i] + "";
				for (int k = 0; k < r[i+1].length; k++) {
					rr[iiii][k]=r[i+1][k];
					rline = rline + "\t" + r[i+1][k];
				}
				rout.write(rline);
				rout.newLine();
				rout.flush();
				}
				
				for(int z=0; z<rdisplay.length; z++) { // fdi
				 temp=fxk[rdisplay[z]][z];
				 Stuff.Equal(xk,fxk,z);
				 fxk[rdisplay[z]][z]=temp;
				 }

				t[i + 1] = (i + 1) * Constants.h;
				
				if(i%dilute==0) {
				String line = t[i + 1] + "";
				for (int j = 0; j < xk.length; j++) {
					line = line + "\t" + xk[j];
					xx[iiii][j]=xk[j];
					tt[iiii]=t[i];
				}
				out.write(line);
				out.newLine();
				out.flush();
				iiii++;
				}
			} // for(int i=0; i<cc; i++){
//	~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			out.close();
			uout.close();
			rout.close();
			
			for (int i = 0; i < 2 + 5; i++) {
				u[cc][i] = u[cc - 1][i];
			}
			
			// x state index
			int[] tdisplay = { 4, 9, 14, 19, 24 }; // Temp
			// int[] adisplay = {0,5,10,15,20}; // A
			int[] bdisplay = {1,6,11,16,21}; // B
			// int[] cdisplay = {2,7,12,17,22}; // C
			// int[] ddisplay = {3,8,13,18,23}; // D
			int[] udisplay = { 0, 1, 2, 3, 4, 5, 6 }; // All Control
			int[] alldisplay = new int[Constants.ODE];

			for (int i = 0; i < Constants.ODE; i++)
				alldisplay[i] = i;

			for (int z = 0; z < rdisplay.length; z++) { // print residual stats
				System.out.println(rdisplay[z] + "- avg: "
						+ Stuff.avg(Stuff.Equal(r, z)) + ", std dev: "
						+ Stuff.sdKnuth(Stuff.Equal(r, z)));
			}

			int[] abc =  {1,1,1,1,1,1,1};
			
			double[] threshold = new double[rdisplay.length]; // residual threshold
			
			for (int z = 0; z < rdisplay.length; z++) { // print residual stats
				threshold[z]= rlimits[z];
			}
			
			new PlotIt6("Temperatures", Stuff.Reverse(xx), tt, xset, tdisplay,1).plot();
			new PlotIt6("Control Inputs", Stuff.Reverse(uu), tt, udisplay,1).plot();
			// new PlotIt6("fTemperatures", f1x, t, display,1).plot(); // fdi
			// new PlotIt6("fControl Inputs", f1u, t, udisplay,1).plot(); // fdi
			new PlotIt6("Residuals",Stuff.Reverse(rr),tt,threshold,abc).plot();
			// new PlotIt6("aResiduals", r, t, adisplay,1).plot(); // fdi
			new PlotIt6("b", Stuff.Reverse(xx), tt, xset, bdisplay,1).plot(); // fdi
			// new PlotIt6("cResiduals", r, t, cdisplay,1).plot(); // fdi
			// new PlotIt6("dResiduals", r, t, ddisplay,1).plot(); // fdi
			//new PlotIt6("tResiduals", r, t, tdisplay,1).plot(); // fdi

			double cost = 0;
			for (int i = 0; i < lcc; i += coc) {
				for (int j = 0; j < Constants.ODE; j++) {
					cost = cost + (x[i][j] - xset[j]) * Qc[j]
							* (x[i][j] - xset[j]);
				}
				for (int j = 0; j < 2; j++) {
					cost = cost + u[i][j] * Rc2[j] * u[i][j]
							* Constants.umax[j] * Constants.umax[j];
				}
				for (int j = 0; j < 5; j++) {
					cost = cost + u[i][j + 2] * Rc[j] * u[i][j + 2]
							* Constants.umax[j + 2] * Constants.umax[j + 2];
				}
			}
			System.out.println("Total cost is : " + cost);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}