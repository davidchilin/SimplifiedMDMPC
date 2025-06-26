// SeqMPCControl_FDI - after fault detection, reset fault to zero.
// all faults stabilize to stable steady state

package core;

import java.io.BufferedWriter;
import java.util.Date;

import core.model.Constants;
import core.model.Dynamics;
import core.util.DMPC;
import core.util.FileIO;
import core.util.GenerateBoundedNoise;
import core.util.NicePlot;
import core.util.Sontag;
import core.util.Stuff;

//import core.util.PlotIt5;

public class SeqMPCControl_FDI {
	public static void main(String[] args) {
		/* Set up initial state and final state */
		Constants cons = new Constants();
		double[] xset = new double[Constants.ODE];
		double[] xk = new double[Constants.ODE];
		xk = cons.getInitPoint(); // initial point
		xset = cons.getSetPoint(); // set point

		/* Set up simulation environment */
		int cc = 500; // simulation time
		int lcc = 500; // cost counter time
		double[] uk = new double[5];
		double[] uk_f = new double[2];
		double[] t = new double[cc + 1];
		double[][] et = new double[50][3]; // timing
		int cet = 0;
		double[][] x = new double[cc + 1][Constants.ODE];
		double[][] u = new double[cc + 1][2 + 5];
		GenerateBoundedNoise gen = new GenerateBoundedNoise();
		gen.setSeed(13);
		double[][] pnoise = gen.generateNoise(cc);
		Dynamics dyn = new Dynamics();
		Sontag son = new Sontag(xset);

		/* Set up MPC */
		int nc = 10, N = 2;
		int coc = 30; // cost increment
		double[] Qc = Constants.Qc, Rc = Constants.Rc, Rc2 = Constants.Rc2;
		double[][] u_prev = new double[N][7];
		String xfile = "./data/newsim2/seq_x_n" + N + "_nc" + nc + ".data";
		String ufile = "./data/newsim2/seq_u_n" + N + "_nc" + nc + ".data";

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
		// int[] rdisplay = {4,9,14,19,24,6,11}; // only 7 residuals: Temp + CB2 + CB3
		int[] rdisplay = new int[Constants.ODE]; // all states residuals
		for (int i = 0; i < rdisplay.length; i++)
			rdisplay[i] = i;

		double[][] r = new double[cc + 1][rdisplay.length];
		double[][] pfu = new double[7][rdisplay.length]; // previous f1u
		double[][] fxk = new double[Constants.ODE][rdisplay.length]; // current filter state
		double[][][] fu_prev = new double[N][7][rdisplay.length];
		double[][] fu1_opt = new double[n1][rdisplay.length], fu2_opt = new double[n2][rdisplay.length], fu3_opt = new double[n3][rdisplay.length];
		double[][] fuk = new double[5][rdisplay.length];
		double[][] fuk_f = new double[2][rdisplay.length];

		for (int z = 0; z < rdisplay.length; z++)
			Stuff.Equal(xk, fxk, z); // initialize fxk

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

		double[] average = { 0.09723058782177801, 3.723033120629459E-5,
				0.05758480768570673, 0.04703631096146602, -0.22627025012018767,
				0.1260114803158344, -2.190761581523269E-6, -0.2610399493940081,
				0.046355032412964826, 0.31257401889738057, 0.17326306718232526,
				1.0730513157131594E-5, -0.11126016853880062,
				0.011731060692115134, -0.04304395316038593,
				-0.7996301427717076, 0.008731975589383358, 0.43640451406565794,
				0.14015988311593872, 0.5624473553397791, 0.37786388465388193,
				4.997253828246685E-4, -0.30333303010199536,
				0.006036139679915797, 0.010336042885870183 };

		double[] stdev = { 0.6884122999285412, 0.005942076103779302,
				0.3041884665393345, 0.1434590842301248, 0.9081626266172459,
				0.7137035128835183, 0.005785426696972679, 0.36422836404192305,
				0.1436521811845303, 0.9067107195743265, 0.8546310108066848,
				0.006008739148119745, 0.3147175515448892, 0.13847039574119177,
				0.823049129222075, 1.6730965773445865, 0.03884048363808831,
				0.39897424540634907, 0.380068371132413, 0.6701334356071306,
				0.8597705914824046, 0.015304840586291604, 0.33949914034258133,
				0.06947871936520975, 0.31332886577316826 };

		double[][] rlimits = new double[rdisplay.length][2];

		for (int i : rdisplay) {
			rlimits[i][0] = average[i] - 3 * stdev[i];
			rlimits[i][1] = average[i] + 3 * stdev[i];
		}

		int[][] isolate = { { 0, 0 }, { 0, 0 } }; // first index r to exceed
													// threshold, second is
													// counter
		int isolated = 10; // minimum time to confirm a problem
		int detected = -1; // fault detection status

// end fdi  initiate ***************************************
		t[0] = 0;

		try {
			BufferedWriter out = new FileIO().getFileBufferedWriter(xfile);
			BufferedWriter uout = new FileIO().getFileBufferedWriter(ufile);

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

			for (int i = 0; i < cc; i++) {
				// fault detection
				if (detected == -1) // if not detected - perform FDI
					for (int j : rdisplay) {
						if (r[i][j] < rlimits[j][0] || r[i][j] > rlimits[j][1]) { // residual exceed threshold
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
							detected = j; // set detected equal to actuator with fault
						}

					} // for(int j=0;j<Constants.ODE;j++)
// end fault detection
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
					for (int z=0; z<rdisplay.length;z++) {// fdi
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
					for (int z = 0; z < rdisplay.length; z++)	// fdi
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
					if (i > cc / 2) { // initiate fault here %%%%%%%%%%%%%%%%%%%%%
						uk_f[0] = 1;
						// uk[2]=-1;
					}
				}
				// initiate reconfigration here
				else if (detected != -2) { // if detected is set to faulty actuator, reset fault
					switch (detected) {
					// mpc 1
					case 4:
						detected = 0;
						uk[0] = 0;
						break;
					case 9:
						detected = 1;
						uk[1] = 0;
						break;
					case 14:
						detected = 2;
						uk[2] = 0;
						break;
					// mpc 2
					case 19:
						detected = 3;
						uk[3] = 0;
						break;
					case 24:
						detected = 4;
						uk[4] = 0;
						break;
					// mpc 3
					case 6:
						detected = 5;
						uk_f[0] = 0;
						break;
					case 11:
						detected = 6;
						uk_f[1] = 0;
						break;
					default:
						detected = 7;
					}
					System.out.print("Fault isolated to: " + act_name[detected]
							+ "\n");
					detected = -2;
				}

				for (int k = 0; k < 2; k++) {
					u[i][k] = uk_f[k];
					pfu[k] = fuk_f[k]; // fdi
				}
				for (int k = 0; k < 5; k++) {
					u[i][k + 2] = uk[k];
					pfu[k + 2] = fuk[k]; // fdi
				}
				String uline = t[i] + "";
				for (int k = 0; k < 2; k++) {
					uline = uline + "\t" + uk_f[k];
				}
				for (int k = 0; k < 5; k++) {
					uline = uline + "\t" + uk[k];
				}
				uout.write(uline);
				uout.newLine();
				uout.flush();

				// xk = dyn.solve(xk, uk, uk_f); // no noise
				xk = dyn.solveWithNoise(xk, uk, uk_f, pnoise[i]);
				for (int z = 0; z < rdisplay.length; z++) 	// fdi
					Stuff.Equal(dyn.solve(Stuff.Equal(fxk, z), Stuff.Equal(fuk, z), Stuff.Equal(fuk_f, z)), fxk, z); // fdi
					
				for (int k = 0; k < Constants.ODE; k++) { // record keeping
					x[i + 1][k] = xk[k];
				}

				 for(int z=0; z<rdisplay.length; z++){ // record keeping
				 r[i+1][z]= xk[rdisplay[z]]-fxk[rdisplay[z]][z]; // fdi
				 }

				 for(int z=0; z<rdisplay.length; z++) { // fdi
				 temp=fxk[rdisplay[z]][z];
				 Stuff.Equal(xk,fxk,z);
				 fxk[rdisplay[z]][z]=temp;
				 }

				t[i + 1] = (i + 1) * Constants.h;
				String line = t[i + 1] + "";
				for (int j = 0; j < xk.length; j++) {
					line = line + "\t" + xk[j];
				}
				out.write(line);
				out.newLine();
				out.flush();
			} // for(int i=0; i<cc; i++){

			out.close();
			uout.close();
			for (int i = 0; i < 2 + 5; i++) {
				u[cc][i] = u[cc - 1][i];
			}
			if (cet > 50) {
				double tsum1 = 0, tsum2 = 0, tsum3 = 0;
				for (int i = 0; i < 50; i++) {
					tsum1 = tsum1 + et[i][0];
					tsum2 = tsum2 + et[i][1];
					tsum3 = tsum3 + et[i][2];
				}
				System.out.println("Avg eval time 50 runs is 1: " + tsum1 / 50
						+ " ms 2: " + tsum2 / 50 + " ms 3: " + tsum3 / 50
						+ " ms");
			} else if (cet <= 50) {
				double tsum1 = 0, tsum2 = 0, tsum3 = 0;
				for (int i = 0; i < cet; i++) {
					tsum1 = tsum1 + et[i][0];
					tsum2 = tsum2 + et[i][1];
					tsum3 = tsum3 + et[i][2];
				}
				System.out.println("Avg eval time MPC1 " + cet + " runs: "
						+ tsum1 / 50 + " ms");
				System.out.println("Avg eval time MPC2 " + cet + " runs: "
						+ tsum2 / 50 + " ms");
				System.out.println("Avg eval time MPC3 " + cet + " runs: "
						+ tsum3 / 50 + " ms");
			}
			// x state index
			int[] tdisplay = { 4, 9, 14, 19, 24 }; // Temp
			// int[] adisplay = {0,5,10,15,20}; // A
			// int[] bdisplay = {1,6,11,16,21}; // B
			// int[] cdisplay = {2,7,12,17,22}; // C
			// int[] ddisplay = {3,8,13,18,23}; // D
			int[] udisplay = { 0, 1, 2, 3, 4, 5, 6 }; // All Control
			int[] alldisplay = new int[Constants.ODE];
			int[] rrdisplay = { 4, 9, 14, 19, 24, 6, 11 }; // Temp + CB2 + CB3

			for (int i = 0; i < Constants.ODE; i++)
				alldisplay[i] = i;

			for (int z = 0; z < rdisplay.length; z++) { // print residual stats
				System.out.println(rdisplay[z] + "- avg: "
						+ Stuff.avg(Stuff.Equal(r, z)) + ", std dev: "
						+ Stuff.sdKnuth(Stuff.Equal(r, z)));
			}

			new NicePlot("Temperatures", x, t, tdisplay).plot();
			new NicePlot("Control Inputs", u, t, udisplay).plot();
			// new NicePlot("fTemperatures", f1x, t, display).plot(); // fdi
			// new NicePlot("fControl Inputs", f1u, t, udisplay).plot(); // fdi
			new NicePlot("Residuals", r, t, rrdisplay).plot(); // fdi
			// new NicePlot("aResiduals", r, t, adisplay).plot(); // fdi
			// new NicePlot("bResiduals", r, t, bdisplay).plot(); // fdi
			// new NicePlot("cResiduals", r, t, cdisplay).plot(); // fdi
			// new NicePlot("dResiduals", r, t, ddisplay).plot(); // fdi
			// new NicePlot("tResiduals", r, t, tdisplay).plot(); // fdi

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