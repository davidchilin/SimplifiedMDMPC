package core.util;

import java.io.*;

public class FileIO {
  public BufferedReader getFileBufferedReader(String filename) {
    FileInputStream fin;
    File titleFile;
    BufferedInputStream bin;
    Reader rin;
    BufferedReader in;
    titleFile = new File(filename);
    try {
      fin = new FileInputStream(titleFile);
      bin = new BufferedInputStream(fin);
      rin = new InputStreamReader(bin);
      in = new BufferedReader(rin);
      return in;
    }
    catch (IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  public BufferedReader getFileBufferedReader(File titleFile) {
    FileInputStream fin;
    BufferedInputStream bin;
    Reader rin;
    BufferedReader in;
    try {
      fin = new FileInputStream(titleFile);
      bin = new BufferedInputStream(fin);
      rin = new InputStreamReader(bin);
      in = new BufferedReader(rin);
      return in;
    }
    catch (IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  /* function to get file bufferedWriter
   */
  public BufferedWriter getFileBufferedWriter(String filename) {
    FileOutputStream fout;
    File titleFile;
    BufferedOutputStream bout;
    Writer rout;
    BufferedWriter out;
    titleFile = new File(filename);
    try {
      fout = new FileOutputStream(titleFile);
      bout = new BufferedOutputStream(fout);
      rout = new OutputStreamWriter(bout);
      out = new BufferedWriter(rout);
      return out;
    }
    catch (IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  public BufferedWriter getFileBufferedWriter(File titleFile) {
    FileOutputStream fout;
    BufferedOutputStream bout;
    Writer rout;
    BufferedWriter out;
    try {
      fout = new FileOutputStream(titleFile);
      bout = new BufferedOutputStream(fout);
      rout = new OutputStreamWriter(bout);
      out = new BufferedWriter(rout);
      return out;
    }
    catch (IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  //append to file
  public BufferedWriter getFileBufferedWriter(String filename, boolean mode) {
    FileOutputStream fout;
    File titleFile;
    BufferedOutputStream bout;
    Writer rout;
    BufferedWriter out;
    titleFile = new File(filename);
    try {
      fout = new FileOutputStream(titleFile, mode);
      bout = new BufferedOutputStream(fout);
      rout = new OutputStreamWriter(bout);
      out = new BufferedWriter(rout);
      return out;
    }
    catch (IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  public BufferedWriter getFileBufferedWriter(File titleFile, boolean mode) {
    FileOutputStream fout;
    BufferedOutputStream bout;
    Writer rout;
    BufferedWriter out;
    try {
      fout = new FileOutputStream(titleFile, mode);
      bout = new BufferedOutputStream(fout);
      rout = new OutputStreamWriter(bout);
      out = new BufferedWriter(rout);
      return out;
    }
    catch (IOException e) {
      e.printStackTrace();
    }
    return null;
  }

}
