package core.util;

import java.io.BufferedReader;
import java.util.StringTokenizer;

import core.model.Constants;

public class CalculateCost {

	public static void main(String[] arg){
		String xfile = "./data/newsim2/c_x_n1_nc30.data";
		String ufile = "./data/newsim2/c_u_n1_nc30.data";
		xfile = "./data/newsim2/para_x_ite1_n1_nc30.data";
		ufile = "./data/newsim2/para_u_ite1_n1_nc30.data";
		//xfile = "./data/newsim2/seq_x_n1_nc30.data";
		//ufile = "./data/newsim2/seq_u_n1_nc30.data";
		//xfile = "./data/newsim2/son_x.data";
		//ufile = "./data/newsim2/son_u.data";
		try{
			int lcc = 900;
			int nc = 30;
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

			BufferedReader uin = new FileIO().getFileBufferedReader(ufile);
			line = uin.readLine();
			StringTokenizer tokenizer=new StringTokenizer(line,"\t");
			int j = 0;
			while(tokenizer.hasMoreTokens())
			{
				tokenizer.nextToken();
				j = j + 1;
			}
			
			double[][] u = new double[cc][j-1];
			uin = new FileIO().getFileBufferedReader(ufile);
			i = 0;
			while((line = uin.readLine()) != null){
				StringTokenizer tokenizer1=new StringTokenizer(line,"\t");
				Double.parseDouble(tokenizer1.nextToken());
				int k = 0;
				while(tokenizer1.hasMoreTokens())
				{
					String val = tokenizer1.nextToken();
					u[i][k] = Double.parseDouble(val);
					k = k + 1;
				}
				i = i + 1;
			}
			uin.close();
			double cost = 0;
			double[] xcost = new double[Constants.ODE];
			double[] ucost = new double[j-1];
			Constants cons = new Constants();
			double[] xset = cons.getSetPoint();
			for(i=0; i<lcc; i+=nc){
				for(j=0; j<Constants.ODE; j++){
					cost = cost + (x[i][j] - xset[j])*Constants.Qc[j]*(x[i][j] - xset[j]);
					xcost[j] = xcost[j] + (x[i][j] - xset[j])*Constants.Qc[j]*(x[i][j] - xset[j]);
				}
				if(u[0].length == 7){
					for(j=0; j<2; j++){
						cost =  cost + u[i][j]*Constants.Rc2[j]*u[i][j]*Constants.umax[j]*Constants.umax[j];
						ucost[j] = ucost[j] + u[i][j]*Constants.Rc2[j]*u[i][j]*Constants.umax[j]*Constants.umax[j]; 						
					}
					for(j=0; j<5; j++){
						cost =  cost + (u[i][j+2])*Constants.Rc[j]*(u[i][j+2])*Constants.umax[j+2]*Constants.umax[j+2];
						ucost[j+2] = ucost[j+2] + u[i][j+2]*Constants.Rc[j]*u[i][j+2]*Constants.umax[j+2]*Constants.umax[j+2]; 
					}
				}
				else if(u[0].length == 5){
					for(j=0; j<5; j++){
						cost =  cost + (u[i][j])*Constants.Rc[j]*(u[i][j])*Constants.umax[j+2]*Constants.umax[j+2];
						ucost[j] = ucost[j] + u[i][j]*Constants.Rc[j]*u[i][j]*Constants.umax[j+2]*Constants.umax[j+2]; 						
					}
				}
			}
			System.out.println("Total cost is : "+cost);
			
		}catch(Exception e){
			e.printStackTrace();
		}
	}
}
