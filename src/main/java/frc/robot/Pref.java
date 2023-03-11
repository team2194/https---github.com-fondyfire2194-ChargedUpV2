/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Collection;
/**
 * Add your docs here.
 */
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;

import edu.wpi.first.wpilibj.Preferences;

public class Pref {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Collection<String> v;
  private static Enumeration<String> e;
  private static String tempString;
  private static double tempDouble;

  public static HashMap<String, Double> prefDict = new HashMap<>();

  static {

    prefDict.put("SwerveTune", 0.);

    prefDict.put("SwerveTurnPoskP", .004);
    prefDict.put("SwerveTurnPoskI", 0.);
    prefDict.put("SwerveTurnPoskD", 0.);
    prefDict.put("SwerveTurnPoskIz", 0.);

    prefDict.put("SwerveVelPoskP", .1);
    prefDict.put("SwerveVelPoskI", 0.);
    prefDict.put("SwerveVElPoskD", 0.);
    prefDict.put("SwerveVelPoskIz", 0.);

    prefDict.put("StrafeTune", 0.);

    prefDict.put("PPStrafekP", .1);
    prefDict.put("PPStrafekI", 0.);
    prefDict.put("PPStrafekD", 0.);

    prefDict.put("YTune", 0.);
    prefDict.put("XTune", 0.);

    prefDict.put("PPYkP", .1);
    prefDict.put("PPYkI", 0.);
    prefDict.put("PPYkD", 0.);

    prefDict.put("PPXkP", .1);
    prefDict.put("PPXkI", 0.);
    prefDict.put("PPXkD", 0.);

    prefDict.put("ThetaTune", 0.);

    prefDict.put("PPThetakP", .1);
    prefDict.put("PPThetakI", 0.);
    prefDict.put("PPThetakD", 0.);

    prefDict.put("RotateTune", 0.);

    prefDict.put("PPRotatekP", .1);
    prefDict.put("PPRotatekI", 0.);
    prefDict.put("PPRotatekD", 0.);

    // drive to load constants

    prefDict.put("loadkPx", .35);
    prefDict.put("loadkPr", .35);
    prefDict.put("loadYTol", .2);
    prefDict.put("loadStopDist", 1.);
    prefDict.put("loadYOffset", .2);

    // drive to vision tape constants

    prefDict.put("tapekPx", .35);
    prefDict.put("tapekPr", .12);
    prefDict.put("tapeYTol", .2);
    prefDict.put("tapeStopTy", 1.);

    // lift characterization
    prefDict.put("liftKs", 0.);
    prefDict.put("liftKg", 0.);
    prefDict.put("liftKv", .0);// 12V/max in per sec 12/21 starting estimate
    prefDict.put("liftKa", .0);

    // ext characterization
    prefDict.put("extKs", .45);
    prefDict.put("extKv", .07);// 12/180
    prefDict.put("extKa", .0);

    // wrist characterization
    prefDict.put("wristKs", .1);
    prefDict.put("wristKg", .98);
    prefDict.put("wristKv", .016);// 1/60
    prefDict.put("wristKa", .0);
  }

  public static void ensureRioPrefs() {
    // init();
    deleteUnused();
    addMissing();
  }

  public static void deleteUnused() {
    v = new Vector<String>();
    v = Preferences.getKeys();
    // v = (Vector<String>) RobotContainer.prefs.getKeys();
    String[] myArray = v.toArray(new String[v.size()]);

    for (int i = 0; i < v.size(); i++) {
      boolean doNotDelete = myArray[i].equals(".type");

      if (!doNotDelete && !prefDict.containsKey(myArray[i]) && Preferences.containsKey(myArray[i])) {
        Preferences.remove(myArray[i]);
      }
    }

  }

  public static void addMissing() {

    Iterator<Map.Entry<String, Double>> it = prefDict.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<String, Double> pair = it.next();
      tempString = pair.getKey();
      tempDouble = pair.getValue();
      if (!Preferences.containsKey((tempString)))
        Preferences.setDouble(tempString, tempDouble);
    }
  }

  public static double getPref(String key) {
    if (prefDict.containsKey(key))
      return Preferences.getDouble(key, prefDict.get(key));
    else
      return 0;
  }

  public static void deleteAllPrefs(Preferences Preferences) {
    edu.wpi.first.wpilibj.Preferences.removeAll();
  }

}
