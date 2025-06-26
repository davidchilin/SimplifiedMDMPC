// PlotIt("Box String Title Here", x, t, s, u, t2).plot(); for plotting RRS model in 3 plots (Temperatures with setpoints, Concentrations with setpoints, Control); where x is ? by t, s(etpoint for each x) is ? by 1, u is ?? by t2
// PlotIt("Box String Title Here", x, t).plot(); single plot where x is ? by t, and t is 1 x t.
// PlotIt("Box String Title Here", x, t, s).plot(); single plot where x is ? by t, and t is 1 x t, s is some setpoint for each ? in x
// 4b: Add Batik SVG generator (debug, error when title is long or many symbols, unable to save file)
// 5: PlotIt("Box String Title Here", x, t, c, #).plot(); where x is ? by t, and t is 1 x t, c.length is number of plots, and c[0] is
//			number of data from x to include in 1st plot, if #=0 then c[1] is number of data from x to include in 2nd plot, etc.
//			if # !=0 then c[0],c[1],c[...] is index in x to plot
// 6: add setpoints to 5. PlotIt("Box String Title Here", x, t, s, c, #) and dashed lines

package core.util;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.Font;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;

import java.awt.event.ActionEvent;
import javax.swing.*;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.AxisLocation;
import org.jfree.chart.axis.NumberAxis;
//import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.plot.CombinedDomainXYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
//import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RectangleInsets;
//import org.jfree.ui.RefineryUtilities;

import org.apache.batik.dom.GenericDOMImplementation;
import org.apache.batik.svggen.SVGGraphics2D;
import org.w3c.dom.DOMImplementation;
import org.w3c.dom.Document;

import java.awt.BasicStroke;

public class PlotIt6 extends ApplicationFrame {
	static final long serialVersionUID=1;
	private JFreeChart chart;
	private Action exportFigure;
	static private int lfont = 14; // font size axis labels
	static private int afont = 14; // font size axis labels
	static private int ht = 450; // 
	static private int wd = 450; // 

	static BasicStroke dashedStroke = new BasicStroke(
            1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
            1.0f, new float[] {3.0f, 6.0f}, 0.0f);
	
// 	Start Constructors
	
	public PlotIt6(String title, double[] x, double[] t) {
		super(title);
		XYDataset dataset = createDataset(x, t);
		chart = createChart(title, dataset);
		chart.setBackgroundPaint(Color.white);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(wd, ht));
		setContentPane(chartPanel);
	}

	public PlotIt6(String title, double[] x, double[] t, double s) {
		super(title);
		XYDataset dataset = createDataset(x, t, s);
		chart = createChart(title, dataset);
		chart.setBackgroundPaint(Color.white);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(wd, ht));
		setContentPane(chartPanel);
	}
	
	public PlotIt6(String title, double[][] x, double[] t, double[] s) {
		super(title);
		XYDataset dataset = createDataset(x, t, s);
		chart = createChart(title, dataset);
		chart.setBackgroundPaint(Color.white);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(wd, ht));
		setContentPane(chartPanel);
	}
	
	public PlotIt6(String title, double[][] x, double[] t) {
		super(title);
		XYDataset dataset = createDataset(x, t);
		chart = createChart(title, dataset);
		chart.setBackgroundPaint(Color.white);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(wd, ht));
		setContentPane(chartPanel);
	}
	
	public PlotIt6(String title, double[][] x, double[] t, int[] c) {
		super(title);
		chart = createChart(title, x, t, c, 0);
		chart.setBackgroundPaint(Color.white);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(wd, ht/5*c.length));
		setContentPane(chartPanel);
	}
	
	public PlotIt6(String title, double[][] x, double[] t, double[] s, int[] c) {
		super(title);
		chart = createChart(title, x, t, s, c, 0);
		chart.setBackgroundPaint(Color.white);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(wd, ht/5*c.length));
		setContentPane(chartPanel);
	}
	
	public PlotIt6(String title, double[][] x, double[] t, int[] c, int ii) {
		super(title);
		chart = createChart(title, x, t, c, ii);
		chart.setBackgroundPaint(Color.white);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(wd, ht/5*c.length));
		setContentPane(chartPanel);
	}
	
	public PlotIt6(String title, double[][] x, double[] t, double[] s, int[] c, int ii) {
		super(title);
		chart = createChart(title, x, t, s, c, ii);
		chart.setBackgroundPaint(Color.white);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(wd, ht/5*c.length));
		setContentPane(chartPanel);
	}
	
	public PlotIt6(String title, double[][] x, double[] t, double[] s, double[][] u, double[] t2) {
		super(title);
		chart  = createChart(x, t, s, u, t2);
		chart.setBackgroundPaint(Color.white);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(wd, ht));
		setContentPane(chartPanel);
	}
	
// End Constructors //
	
	private static XYDataset createDataset(double[] x, double[] t) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		
		XYSeries series = new XYSeries("");
		
			for (int i = 0; i < t.length; i++) 
			{	series.add(t[i], x[i]);		}

			dataset.addSeries(series);
		
		return dataset;
	}

	private static XYDataset createDataset(double[] x, double[] t, double s) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		
		XYSeries series = new XYSeries("");
		
			for (int i = 0; i < t.length; i++) 
			{	series.add(t[i], x[i]); }		

			dataset.addSeries(series);
			
			series = new XYSeries(s+"");
			
			series.add(0, s);
			series.add(t[t.length-1], s);
			
			dataset.addSeries(series);
		
		return dataset;
	}	
	
	private static XYSeriesCollection createDataset(double[][] x, double[] t) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		
		XYSeries series[] = new XYSeries[1];
		
		for(int z = 0; z < x.length; z++) {
			series[0] = new XYSeries(""+z);
			for (int i = 0; i < t.length; i++) 
			{	series[0].add(t[i], x[z][i]); }		

			dataset.addSeries(series[0]);
		}
		return dataset;
	}
	
	private static XYSeriesCollection createDataset(double[][] x, double[] t, double s) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		
		XYSeries series[] = new XYSeries[1];
		
		for(int z = 0; z < x.length; z++) {
			series[0] = new XYSeries(""+z);
			for (int i = 0; i < t.length; i++) 
			{	series[0].add(t[i], x[z][i]); }		

			dataset.addSeries(series[0]);
		}
		
		series[0]= new XYSeries("");
		series[0].add(0, s);
		series[0].add(t[t.length-1], s);
		
		dataset.addSeries(series[0]);
		
		return dataset;
	}

	
	private static XYDataset createDataset(double[][] x, double[] t, double[] s) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		
		XYSeries series[] = new XYSeries[2];
		
		for(int z = 0; z < x.length; z++) {
			series[0] = new XYSeries(""+z);
			for (int i = 0; i < t.length; i++) {
				series[0].add(t[i], x[z][i]);		
			}
			
			series[1]= new XYSeries("");
			series[1].add(0, s[z]);
			series[1].add(t[t.length-1], s[z]);

			dataset.addSeries(series[0]);
			dataset.addSeries(series[1]);
		}
		return dataset;
	}
	
	private static JFreeChart createChart(String title, XYDataset dataset) {
		// create the chart...
		JFreeChart chart = ChartFactory.createXYLineChart(
				title, // chart title
				"time (min)", // x axis label
				"Units", // y axis label
				dataset, // data
				PlotOrientation.VERTICAL, false, // include legend
				true, // tooltips
				false // urls
				);

		// get a reference to the plot for further customisation...
		XYPlot plot = (XYPlot) chart.getPlot();
		plot.setBackgroundPaint(Color.white);
		plot.setAxisOffset(new RectangleInsets(5.0, 5.0, 5.0, 5.0));
		plot.setDomainGridlinesVisible(false);
		plot.setRangeGridlinesVisible(false);
		//plot.setDomainGridlinePaint(Color.white);
		//plot.setRangeGridlinePaint(Color.white);
		NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
		rangeAxis.setLabelFont(new Font("SansSerif", Font.PLAIN, lfont));
		rangeAxis.setTickLabelFont(new Font("SansSerif", Font.PLAIN, afont));
		rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
		        
//		plot.getRenderer().setSeriesStroke(6,dashedStroke);
		plot.getRenderer().setSeriesPaint(0, Color.BLACK);
		plot.getRenderer().setSeriesPaint(1, Color.BLACK);
		return chart;
	}
// **********************************
	private static JFreeChart createChart(String title, double[][] x, double[] t, int[] c, int ii)
	{
		// setup parent plot
		NumberAxis domainAxis = new NumberAxis("Time (min)");
		domainAxis.setLabelFont(new Font("SansSerif", Font.PLAIN, lfont));
        domainAxis.setTickLabelFont(new Font("SansSerif", Font.PLAIN, afont));
		final CombinedDomainXYPlot plot = new CombinedDomainXYPlot(domainAxis);
        plot.setGap(10.0);
        // first create plots from each dataset        
        XYItemRenderer renderer;
        NumberAxis rangeAxis;
        XYPlot subplot;
		XYSeriesCollection dataset;// = new XYSeriesCollection();
		
		// setup dataset for each plot
		int w = 0;  // 0  to x.length
		for (int i = 0; i < c.length; i++) {
			
			if(ii!=0) {w=c[i];c[i]=1;}
			double[][] y = new double[c[i]][];
				for (int z = 0; z < y.length;z++)
				{
					y[z]=x[w];
					w++;
				}

		dataset = createDataset( y, t);
		// setup subplots
		renderer = new StandardXYItemRenderer();
        rangeAxis = new NumberAxis(""+i/* "X-axis title" */);
        rangeAxis.setLabelFont(new Font("SansSerif", Font.PLAIN, lfont));
        rangeAxis.setTickLabelFont(new Font("SansSerif", Font.PLAIN, afont));
        rangeAxis.setAutoRangeIncludesZero(false);
        subplot = new XYPlot(dataset, null, rangeAxis, renderer);
        subplot.setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
        subplot.setDomainGridlinesVisible(false);
		subplot.setRangeGridlinesVisible(false);
		subplot.getRenderer().setSeriesPaint(0, Color.BLACK);
		subplot.getRenderer().setSeriesPaint(1, Color.BLACK);
		plot.add(subplot, 1);
		}
		
		plot.setOrientation(PlotOrientation.VERTICAL);
		return new JFreeChart(title, JFreeChart.DEFAULT_TITLE_FONT, plot, /* legend */ false);
	}
	
	private static JFreeChart createChart(String title, double[][] x, double[] t, double[] s, int[] c, int ii)
	{
		// setup parent plot
		NumberAxis domainAxis = new NumberAxis("Time (min)");
		domainAxis.setLabelFont(new Font("SansSerif", Font.PLAIN, lfont));
        domainAxis.setTickLabelFont(new Font("SansSerif", Font.PLAIN, afont));
		final CombinedDomainXYPlot plot = new CombinedDomainXYPlot(domainAxis);
        plot.setGap(10.0);
        // first create plots from each dataset        
        XYItemRenderer renderer;
        NumberAxis rangeAxis;
        XYPlot subplot;
		XYSeriesCollection dataset;// = new XYSeriesCollection();
		
		// setup dataset for each plot
		int w = 0;  // 0  to x.length
		for (int i = 0; i < c.length; i++) { // c.length is number of plots
			if(ii!=0) {w=c[i];c[i]=1;} // if ii=0, multiple states per subplot, else only 1
			double[][] y = new double[c[i]][];
				for (int z = 0; z < y.length;z++) // y.length is number of states per lot
				{
					y[z]=x[w];
					w++; // loop through all states, starting at state c[i]
				}
				
		if(ii==0) dataset = createDataset( y, t, s[i]);
		else dataset = createDataset( y, t, s[w-1]); // error here.
				
		// setup subplots
		renderer = new StandardXYItemRenderer();
        rangeAxis = new NumberAxis(""+i/* "X-axis title" */);
        rangeAxis.setLabelFont(new Font("SansSerif", Font.PLAIN, lfont));
        rangeAxis.setTickLabelFont(new Font("SansSerif", Font.PLAIN, afont));
        rangeAxis.setAutoRangeIncludesZero(false);
        subplot = new XYPlot(dataset, null, rangeAxis, renderer);
        subplot.setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
        subplot.setDomainGridlinesVisible(false);
		subplot.setRangeGridlinesVisible(false);
		subplot.getRenderer().setSeriesStroke(1,dashedStroke);
		subplot.getRenderer().setSeriesPaint(0, Color.BLACK);
		subplot.getRenderer().setSeriesPaint(1, Color.BLACK);
		plot.add(subplot, 1);
		} // for (int i = 0; i < c.length; i++) {
		
		plot.setOrientation(PlotOrientation.VERTICAL);
		return new JFreeChart(title, JFreeChart.DEFAULT_TITLE_FONT, plot, /* legend */ false);
	}
	
// **********************************		
	private static JFreeChart createChart(double[][] x, double[] t,	double[] xset, double[][] u, double[] t2) {
		// for plotting states/control of RRS system
		XYSeriesCollection dataT = new XYSeriesCollection();
		XYSeriesCollection dataC = new XYSeriesCollection();
		XYSeriesCollection dataU = new XYSeriesCollection();
		XYSeries series[] = new XYSeries[2];
		
		char ves[] = {'A','B','C'};

		for (int z = 0, j = 0; z < xset.length; z++) {
			if(j==0) {
				series[0] = new XYSeries("T" + (z/4 + 1));
				series[1] = new XYSeries("T" + (z/4 + 1) + "s");
				for (int i = 0; i < t.length; i++) {
					series[0].add(t[i], x[z][i]);
				}
				series[1].add(0, xset[z]);
				series[1].add(t[t.length-1], xset[z]);
				dataT.addSeries(series[0]);
				dataT.addSeries(series[1]);	
			}
			else {
				series[0] = new XYSeries("C" + ves[j-1] + (z/4+1));
				series[1] = new XYSeries("C" + ves[j-1] + (z/4+1) + "s");
				for (int i = 0; i < t.length; i++) {
					series[0].add(t[i], x[z][i]);
				}
				series[1].add(0, xset[z]);
				series[1].add(t[t.length-1], xset[z]);
				dataC.addSeries(series[0]);
				dataC.addSeries(series[1]);
			}
			j++;
			if(j==4) j=0;
		}
		
		XYSeries seriesU = new XYSeries("");
		
		for(int z = 0; z < u.length; z++) {
			seriesU = new XYSeries("U"+z);
			for (int i = 0; i < t2.length; i++) {
				seriesU.add(t2[i], u[z][i]);
			}
			dataU.addSeries(seriesU);
		}
		
        // create subplot T...
        final XYItemRenderer renderer1 = new StandardXYItemRenderer();
        final NumberAxis rangeAxis1 = new NumberAxis("Temperature (K)");
        rangeAxis1.setAutoRangeIncludesZero(false);
        final XYPlot subplot1 = new XYPlot(dataT, null, rangeAxis1, renderer1);
//      ValueAxis axis1 = subplot1.getRangeAxis();
//      axis1.setRange(370, 390);
        subplot1.setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
        // create subplot C...
        final XYItemRenderer renderer2 = new StandardXYItemRenderer();
        final NumberAxis rangeAxis2 = new NumberAxis("Concentration (kmol/m3)");
        rangeAxis2.setAutoRangeIncludesZero(false);
        final XYPlot subplot2 = new XYPlot(dataC, null, rangeAxis2, renderer2);
        subplot2.setRangeAxisLocation(AxisLocation.TOP_OR_LEFT);
        // create subplot U...
        final XYItemRenderer renderer3 = new StandardXYItemRenderer();
        final NumberAxis rangeAxis3 = new NumberAxis("Control Action (10e5 KJ/hr)");
        rangeAxis3.setAutoRangeIncludesZero(false);
        final XYPlot subplot3 = new XYPlot(dataU, null, rangeAxis3, renderer3);
        subplot3.setRangeAxisLocation(AxisLocation.TOP_OR_LEFT);
        
        // parent plot...
        final CombinedDomainXYPlot plot = new CombinedDomainXYPlot(new NumberAxis("Time (min)"));
        plot.setGap(10.0);
        
        // add the subplots...
        plot.add(subplot1, 1);
        plot.add(subplot2, 1);
        plot.add(subplot3, 1);
        plot.setOrientation(PlotOrientation.VERTICAL);
        plot.setBackgroundPaint(Color.white);

		return new JFreeChart("System State and Input Trajectories",
                JFreeChart.DEFAULT_TITLE_FONT, plot, true);
	}

    public void exportFigure() {
       	File svgFile = new File("./data/"+chart.getTitle().getText() + ".svg");
    	
    	// write it to file
    	try {
			exportChartAsSVG(chart, 
				getContentPane().getBounds(), svgFile);

	    	System.out.println("Figured saved as " + svgFile.getAbsolutePath());
    	
    	} catch (IOException e) {
			System.err.println("Error saving file:\n" + e.getMessage());
		}
    }

    private JMenuBar createMenuBar() {
		JMenuBar result = new JMenuBar();
		JMenu fileMenu = new JMenu("File");
		fileMenu.add(exportFigure);
		result.add(fileMenu);
		return result;
	}
	
	/**
	 * Exports a JFreeChart to a SVG file.
	 * 
	 * @param chart JFreeChart to export
	 * @param bounds the dimensions of the viewport
	 * @param svgFile the output file.
	 * @throws IOException if writing the svgFile fails.
	 */
	void exportChartAsSVG(JFreeChart chart, Rectangle bounds, File svgFile) throws IOException {
        // Get a DOMImplementation and create an XML document
        DOMImplementation domImpl =
            GenericDOMImplementation.getDOMImplementation();
        Document document = domImpl.createDocument(null, "svg", null);

        // Create an instance of the SVG Generator
        SVGGraphics2D svgGenerator = new SVGGraphics2D(document);

        // draw the chart in the SVG generator
        chart.draw(svgGenerator, bounds);

        // Write svg file
        OutputStream outputStream = new FileOutputStream(svgFile);
        Writer out = new OutputStreamWriter(outputStream, "UTF-8");
        svgGenerator.stream(out, true /* use css */);						
        outputStream.flush();
        outputStream.close();
	}
		
	public void plot() {
		this.pack();
		//RefineryUtilities.centerFrameOnScreen(this);
		this.setVisible(true);
		
        exportFigure = new AbstractAction("Export figure") 
        {      	public void actionPerformed(ActionEvent arg0) 
        		{ 	exportFigure(); 		}  };
        
        this.setJMenuBar(createMenuBar());
	}

	public static void main(String[] args) {

		double[][] x = { { 1.0, 2.0, 3.0, 4.0, 5.0 }, {2.0, 7.0, 8.0, 9.0, 10.0 },
					{ 3.0, 0.25, 0.0, -0.25, -0.5 }, { 4.0, 2.0, 3.0, 4.0, 5.0 },
					{ 5.0, 7.0, 8.0, 9.0, 10.0 }, { 6.0, 1.25, 1.0, -1.25, -1.5 },
					{ 7.0, -2.0, -3.0, -4.0, -5.0 }, { 8.0, 7.0, 8.0, 9.0, 10.0 },
					{ 9.0, 0.25, 0.0, -0.25, -0.5 }, { 10.0, 2.5, 3.5, 4.5, 5.5 },
					{ 11.0, -7.0, -8.0, -9.0, -10.0 }, { 12.0, 2.25, 2.0, -2.25, -2.5 }};
		double[] xss1 = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0};
		double[] t = { 0.0, 0.1, 0.2, 0.3, 0.4 };
		double[][] u = { { 10.0, 50.0, 30.0, 0}, { 60.0, 90.0, 80.0, 0.0 },
						 { 5.05, 5.125, 5.00, 6.00 }, 	{ 10.0, 60.0, 30.0, 0.0 }};
		double[] tt = { 0.0, 0.3, 0.45, 0.6 };

		//new PlotIt6("Testing4", x, t, xss1, u, tt).plot();

		int[] g= {1,2,3,6};
		new PlotIt6("Testing3", x, t, g).plot();
		new PlotIt6("Testing2", x, t, g,1).plot();
		new PlotIt6("Testing1", x, t, xss1, g).plot();
		new PlotIt6("Testing0", x, t, xss1, g,1).plot();
		
		double[][] xx = { { 1.0, 2.0, 3.0, 4.0, 5.0 },
				{ 6.0, 7.0, 8.0, 9.0, 10.0 }, { 0.5, 0.25, 0.0, -0.25, -0.5 } };
		double[] ttt = { 0.0, 0.1, 0.2, 0.3, 0.4 };
		double[] s = {5.0, 10.0, -0.5};

		new PlotIt6("Testing5", xx, ttt, s).plot();
	}
}