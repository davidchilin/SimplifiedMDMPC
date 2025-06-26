package core.util;

import java.io.BufferedReader;
import java.util.StringTokenizer;
import core.model.Constants;

public class PlotSavedData {
	
	public static void main(String[] args){

		int[] tdisplay = { 4, 9, 14, 19, 24 }; // Temp
		int[] bdisplay = {1,6,11,16,21}; // B
		int[] udisplay = { 0, 1, 2, 3, 4, 5, 6 }; // All Control
		int[] alldisplay = new int[Constants.ODE];
		int[] rdisplay = {4,9,14,19,24,6,11}; // only 7 residuals: Temp(5) + CB2 + CB3
		
		for (int i = 0; i < Constants.ODE; i++)
			alldisplay[i] = i;
		
		int[] abc =  {1,1,1,1,1,1,1};
		
		double[][] rlimits = new double[rdisplay.length][2];

		int ii=0;
		for (int i : rdisplay) {
			rlimits[ii][0] = Constants.average[i] - 3 * Constants.stdev[i];
			rlimits[ii++][1] = Constants.average[i] + 3 * Constants.stdev[i];
		}
	
		double[] threshold = new double[rdisplay.length]; // residual threshold
		
		for (int z = 0; z < rdisplay.length; z++) { // print residual stats
			threshold[z]= rlimits[z][1];
		}
		
		//double[][] ranges = {{485, 495},{500, 505}, 
		//		{490, 520}, {450, 470}, {420, 490}};
		String xfile = "./data/x.data";
		String ufile = "./data/u.data";
		String rfile = "./data/r7.data";

		Constants cons = new Constants();
		double[] xset = new double[Constants.ODE];
		xset = cons.getSetPoint(); // set point
		
		try {
			BufferedReader in = new FileIO().getFileBufferedReader(xfile);
			String line = "";
			int cc = 0;
			
			while((line = in.readLine()) != null){
				cc = cc + 1;
			}
			in.close();
			in = new FileIO().getFileBufferedReader(xfile);

			double[] t = new double[cc];
			double[][] x = new double[cc][Constants.ODE];
			
			int i = 0;
			
			while((line = in.readLine()) != null){
				StringTokenizer tokenizer=new StringTokenizer(line,"\t");
				t[i] = Double.parseDouble(tokenizer.nextToken());
				int j = 0;
				while(tokenizer.hasMoreTokens())
				{
					String val = tokenizer.nextToken();
					x[i][j] = Double.parseDouble(val);
					j = j + 1;
				}
				i = i + 1;
			}
			in.close();
			
			new PlotIt6("Temperatures", Stuff.Reverse(x), t, xset, tdisplay,1).plot();
			new PlotIt6("Component B", Stuff.Reverse(x), t, xset, bdisplay,1).plot(); // fdi
			
			in = new FileIO().getFileBufferedReader(ufile);
			line = "";
			cc = 0;
			
			while((line = in.readLine()) != null){
				cc = cc + 1;
			}
			in.close();
			in = new FileIO().getFileBufferedReader(ufile);

			t = new double[cc];
			double[][] u = new double[cc][udisplay.length];
			
			i = 0;
			
			while((line = in.readLine()) != null){
				StringTokenizer tokenizer=new StringTokenizer(line,"\t");
				t[i] = Double.parseDouble(tokenizer.nextToken());
				int j = 0;
				while(tokenizer.hasMoreTokens())
				{
					String val = tokenizer.nextToken();
					u[i][j] = Double.parseDouble(val);
					j = j + 1;
				}
				i = i + 1;
			}
			in.close();
			
			new PlotIt6("Control Inputs", Stuff.Reverse(u), t, udisplay,1).plot();

			in = new FileIO().getFileBufferedReader(rfile);
			line = "";
			cc = 0;
			
			while((line = in.readLine()) != null){
				cc = cc + 1;
			}
			in.close();
			in = new FileIO().getFileBufferedReader(rfile);

			t = new double[cc];
			double[][] r = new double[cc][rdisplay.length];
			
			i = 0;
			
			while((line = in.readLine()) != null){
				StringTokenizer tokenizer=new StringTokenizer(line,"\t");
				t[i] = Double.parseDouble(tokenizer.nextToken());
				int j = 0;
				while(tokenizer.hasMoreTokens())
				{
					String val = tokenizer.nextToken();
					r[i][j] = Double.parseDouble(val);
					j = j + 1;
				}
				i = i + 1;
			}
			in.close();
	
			new PlotIt6("Residuals",Stuff.Reverse(r),t,threshold,abc).plot();
	} catch (Exception e) {	e.printStackTrace();}
	}
}