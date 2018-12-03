///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       CRPI Performance Evaluation
//  Workfile:        crpi_eval.cpp
//  Revision:        14 March, 2016
//  Author:          J. Marvel
//
//  Description
//  ===========
//  
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include "crpi_robot.h"
#include "crpi_kuka_lwr.h"
#include "crpi_universal.h"
#include "crpi_robotiq.h"
#include "crpi_abb.h"
#include "ulapi.h"
#include "NumericalMath.h" 

#pragma warning (disable: 4996)

using namespace crpi_robot;
using namespace std;

int main ()			// FMP
{
  int i = 0;
  double perf1, perf2, perf3, perf4;
  crpi_timer timer;
  double poseadd1 = 5.0f, poseadd2 = 10.0f, jointadd1 = 0.25f, jointadd2 = 0.5f, jointadd3 = 1.0f;

  //CrpiRobot<CrpiUniversal> arm("universal_ur5.xml");
  //CrpiRobot<CrpiAbb> arm("abb_irb14000_left.xml");
  CrpiRobot<CrpiAbb> arm("abb_irb14000_right.xml");
  //CrpiRobot<CrpiUniversal> arm("universal_ur10_right.xml");
  //CrpiRobot<CrpiKukaLWR> arm("kuka_lwr.xml");

  arm.SetAngleUnits("degree");
  arm.SetLengthUnits("mm");
  arm.Couple("Yumi_Parallel");
  robotIO curIO, tarIO;
  robotPose curPose, origPose, tarPose1, tarPose2;
  robotAxes curAxes, origAxes, tarAxes1, tarAxes2, tarAxes3, tarAxes4;

  curPose.x = curPose.y = curPose.z = curPose.xrot = curPose.yrot = curPose.zrot = 0.0f;
  arm.GetRobotPose(&curPose);
  arm.GetRobotAxes(&curAxes);

  cout << "Robot Pose (" << curPose.x << ", " << curPose.y << ", " << curPose.z << ", " 
       << curPose.xrot << ", " << curPose.yrot << ", " << curPose.zrot << ")" << endl;
  cout << "Robot Joints (" << curAxes.axis.at(0) << ", " << curAxes.axis.at(1) << ", "
    << curAxes.axis.at(2) << ", " << curAxes.axis.at(3) << ", " << curAxes.axis.at(4) << ", "
    << curAxes.axis.at(5) << ", " << curAxes.axis.at(6) << ")" << endl;
  cout << "Enter 1 to run tests (robot will begin moving), 0 to quit: ";
  cin >> i;

  if (i < 1)
  {
    return 1;			// FMP
  }

  origPose = curPose;
  tarPose1 = curPose;
  tarPose2 = curPose;
  tarPose1.z += 10.0f;
  tarPose2.z -= 10.0f;

  origAxes = curAxes;
  tarAxes1 = curAxes;
  tarAxes2 = curAxes;
  tarAxes3 = curAxes;
  tarAxes4 = curAxes;
  tarAxes1.axis.at(0) += 5;
  tarAxes2.axis.at(0) -= 5;
  tarAxes3.axis.at(0) += 10;
  tarAxes4.axis.at(0) -= 10;

  cout << "Running performance tests: " << endl;
  cout << "  Base logic time:" << endl;
  timer.start();
  perf1 = 0.0f;
  perf4 = 0.0f;
  for (i = 0; i < 100; ++i)
  {
    if (poseadd1 < poseadd2)
    {
      perf4 *= perf1;
    }
    else
    {
      perf4 += perf1;
    }
  }
  perf1 = timer.stop();
  cout << "    Total time: " << perf1 << endl;
  perf1 = perf1 / 100.0f;
  perf3 = 1000.0f / perf1;
  cout << "    Logic sample: Avg = " << perf1 << "ms, " << perf3 << "Hz" << endl;

  cout << "  Feedback test:" << endl;
  timer.start();
  perf4 = 0.0f;
  for (i = 0; i < 100; ++i)
  {
    if (arm.GetRobotPose(&curPose) != CANON_SUCCESS)
    {
      perf4++;
    }
  }
  perf1 = timer.stop();
  cout << "    Total time: " << perf1 << endl;
  perf1 = perf1 / 100.0f;
  perf3 = 1000.0f / perf1;
  cout << "    Pose feedback: Avg = " << perf1 << "ms, " << perf3 << "Hz, " << perf4 << " misses" << endl;

  perf4 = 0.0f;
  timer.restart();
  for (i = 0; i < 100; ++i)
  {
    if (arm.GetRobotAxes(&curAxes) != CANON_SUCCESS)
    {
      perf4++;
    }
  }
  perf1 = timer.stop();
  cout << "    Total time: " << perf1 << endl;

  perf1 = perf1 / 100.0f;
  perf3 = 1000.0f / perf1;
  cout << "    Joint feedback: Avg = " << perf1 << "ms, " << perf3 << "Hz, " << perf4 << " misses" << endl;

  timer.restart();
  perf4 = 0.0f;
  for (i = 0; i < 100; ++i)
  {
    if (arm.GetRobotIO(&curIO) != CANON_SUCCESS)
    {
      perf4++;
    }
  }
  perf1 = timer.stop();
  cout << "    Total time:  " << perf1 << endl;
  perf1 = perf1 / 100.0f;
  perf3 = 1000.0f / perf1;
  cout << "    IO feedback: Avg = " << perf1 << "ms, " << perf3 << "Hz, " << perf4 << " misses" << endl;


  cout << "  Motion test:" << endl;
  cout << "    Zero motion:" << endl;
  timer.restart();
  perf1 = 0.0f;
  perf2 = 0.0f;
  for (i = 0; i < 100; ++i)
  {
    if (arm.MoveStraightTo(origPose) != CANON_SUCCESS)
    {
      ++perf2;
    }
    else
    {
      arm.GetRobotPose(&curPose);
      perf1 += curPose.distance(origPose);
    }
  }
  perf4 = timer.stop();
  cout << "      Total Time:  " << perf4 << endl;
  perf1 = perf1 / 100.0f;
  perf4 = perf4 / 100.0f;
  perf3 = 1000.0f / perf4;
  cout << "      LIN:  Avg Error = " << perf1 << "mm, " << "Avg = " << perf4 << "ms, " << perf3 << "Hz, " << perf2 << " misses " << endl;

  timer.restart();
  perf1 = 0.0f;
  perf2 = 0.0f;
  for (i = 0; i < 100; ++i)
  {
    if (arm.MoveTo(origPose) != CANON_SUCCESS)
    {
      ++perf2;
    }
    else
    {
      arm.GetRobotPose(&curPose);
      perf1 += curPose.distance(origPose);
    }
  }
  perf4 = timer.stop();
  cout << "      Total Time:  " << perf4 << endl;
  perf1 = perf1 / 100.0f;
  perf4 = perf4 / 100.0f;
  perf3 = 1000.0f / perf4;
  cout << "      PTP:  Avg Error = " << perf1 << "mm, " << "Avg = " << perf4 << "ms, " << perf3 << "Hz, " << perf2 << " misses " << endl;

  timer.restart();
  perf1 = 0.0f;
  perf2 = 0.0f;
  for (i = 0; i < 100; ++i)
  {
    if (arm.MoveToAxisTarget(origAxes) != CANON_SUCCESS)
    {
      ++perf2;
    }
    else
    {
      arm.GetRobotAxes(&curAxes);
      //cout << "------" << endl;
      //curAxes.print();
      //origAxes.print();
      perf1 += curAxes.error(origAxes);
      //cout << perf1 << endl;
      //cout << "------" << endl;
    }
  }
  perf4 = timer.stop();
  cout << "      Total Time:  " << perf4 << endl;
  perf1 = perf1 / 100.0f;
  perf4 = perf4 / 100.0f;
  perf3 = 1000.0f / perf4;
  cout << "      Joint:  Avg Error = " << perf1 << "deg, " << "Avg = " << perf4 << "ms, " << perf3 << "Hz, " << perf2 << " misses " << endl;

  cout << "    Oscillation:" << endl;

  arm.MoveStraightTo(origPose);

  tarPose1 = origPose;
  tarPose2 = origPose;
  tarPose1.z += poseadd1;
  tarPose2.z -= poseadd1;
  cout << "       LIN:  Orig +/- " << poseadd1 << "mm" << endl;

  timer.restart();
  perf1 = 0.0f;
  perf2 = 0.0f;
  perf3 = 0.0f;
  for (i = 0; i < 50; ++i)
  {
    if (arm.MoveStraightTo(tarPose1) == CANON_SUCCESS)
    {
      arm.GetRobotPose(&curPose);
      perf1 += tarPose1.distance(curPose);
    }
    else
    {
      ++perf3;
    }

    if (arm.MoveStraightTo(tarPose2) == CANON_SUCCESS)
    {
      arm.GetRobotPose(&curPose);
      perf2 += tarPose2.distance(curPose);
    }
    else
    {
      ++perf3;
    }
  }
  perf4 = timer.stop();
  cout << "         Total Time:  " << perf4 << endl;
  perf1 = perf1 / 50.0f;
  perf2 = perf2 / 50.0f;
  perf4 = perf4 / 100.0f;
  cout << "         LIN:  Avg Err = (" << perf1 << ", " << perf2 << ")mm, " << perf4 << "ms, ";
  perf4 = 1000.0f / perf4;
  cout << perf4 << "Hz, " << perf3 << " misses" << endl;


  arm.MoveStraightTo(origPose);

  tarPose1 = origPose;
  tarPose2 = origPose;
  tarPose1.z += poseadd2;
  tarPose2.z -= poseadd2;
  cout << "       LIN:  Orig +/- " << poseadd2 << "mm" << endl;

  timer.restart();
  perf1 = 0.0f;
  perf2 = 0.0f;
  perf3 = 0.0f;
  for (i = 0; i < 50; ++i)
  {
    if (arm.MoveStraightTo(tarPose1) == CANON_SUCCESS)
    {
      arm.GetRobotPose(&curPose);
      perf1 += tarPose1.distance(curPose);
    }
    else
    {
      ++perf3;
    }

    if (arm.MoveStraightTo(tarPose2) == CANON_SUCCESS)
    {
      arm.GetRobotPose(&curPose);
      perf2 += tarPose2.distance(curPose);
    }
    else
    {
      ++perf3;
    }
  }
  perf4 = timer.stop();
  cout << "         Total Time:  " << perf4 << endl;
  perf1 = perf1 / 50.0f;
  perf2 = perf2 / 50.0f;
  perf4 = perf4 / 100.0f;
  cout << "         LIN:  Avg Err = (" << perf1 << ", " << perf2 << ")mm, " << perf4 << "ms, ";
  perf4 = 1000.0f / perf4;
  cout << perf4 << "Hz, " << perf3 << " misses" << endl;



  arm.MoveToAxisTarget(origAxes);

  tarAxes1 = origAxes;
  tarAxes2 = origAxes;
  tarAxes1.axis.at(0) += jointadd1;
  tarAxes2.axis.at(0) -= jointadd1;
  cout << "       Joint:  Orig +/- " << jointadd1 << "deg" << endl;

  timer.restart();
  perf1 = 0.0f;
  perf2 = 0.0f;
  perf3 = 0.0f;
  for (i = 0; i < 50; ++i)
  {
    if (arm.MoveToAxisTarget(tarAxes1) == CANON_SUCCESS)
    {
      arm.GetRobotAxes(&curAxes);
      perf1 += tarAxes1.error(curAxes);
    }
    else
    {
      ++perf3;
    }
      
    if (arm.MoveToAxisTarget(tarAxes2) == CANON_SUCCESS)
    {
      arm.GetRobotAxes(&curAxes);
      perf2 += tarAxes2.error(curAxes);
    }
    else
    {
      ++perf3;
    }
  }
  perf4 = timer.stop();
  cout << "         Total Time:  " << perf4 << endl;
  perf1 = perf1 / 50.0f;
  perf2 = perf2 / 50.0f;
  perf4 = perf4 / 100.0f;
  cout << "         Joint:  Avg Err = (" << perf1 << ", " << perf2 << ")deg, " << perf4 << "ms, ";
  perf4 = 1000.0f / perf4;
  cout << perf4 << "Hz, " << perf3 << " misses" << endl;


  arm.MoveToAxisTarget(origAxes);

  tarAxes1 = origAxes;
  tarAxes2 = origAxes;
  tarAxes1.axis.at(0) += jointadd2;
  tarAxes2.axis.at(0) -= jointadd2;
  cout << "       Joint:  Orig +/- " << jointadd2 << "deg" << endl;

  timer.restart();
  perf1 = 0.0f;
  perf2 = 0.0f;
  perf3 = 0.0f;
  for (i = 0; i < 50; ++i)
  {
    if (arm.MoveToAxisTarget(tarAxes1) == CANON_SUCCESS)
    {
      arm.GetRobotAxes(&curAxes);
      perf1 += tarAxes1.error(curAxes);
    }
    else
    {
      ++perf3;
    }

    if (arm.MoveToAxisTarget(tarAxes2) == CANON_SUCCESS)
    {
      arm.GetRobotAxes(&curAxes);
      perf2 += tarAxes2.error(curAxes);
    }
    else
    {
      ++perf3;
    }
  }
  perf4 = timer.stop();
  cout << "         Total Time:  " << perf4 << endl;
  perf1 = perf1 / 50.0f;
  perf2 = perf2 / 50.0f;
  perf4 = perf4 / 100.0f;
  cout << "         Joint:  Avg Err = (" << perf1 << ", " << perf2 << ")deg, " << perf4 << "ms, ";
  perf4 = 1000.0f / perf4;
  cout << perf4 << "Hz, " << perf3 << " misses" << endl;


  arm.MoveToAxisTarget(origAxes);

  tarAxes1 = origAxes;
  tarAxes2 = origAxes;
  tarAxes1.axis.at(0) += jointadd3;
  tarAxes2.axis.at(0) -= jointadd3;
  cout << "       Joint:  Orig +/- " << jointadd3 << "deg" << endl;

  timer.restart();
  perf1 = 0.0f;
  perf2 = 0.0f;
  perf3 = 0.0f;
  for (i = 0; i < 50; ++i)
  {
    if (arm.MoveToAxisTarget(tarAxes1) == CANON_SUCCESS)
    {
      arm.GetRobotAxes(&curAxes);
      perf1 += tarAxes1.error(curAxes);
    }
    else
    {
      ++perf3;
    }

    if (arm.MoveToAxisTarget(tarAxes2) == CANON_SUCCESS)
    {
      arm.GetRobotAxes(&curAxes);
      perf2 += tarAxes2.error(curAxes);
    }
    else
    {
      ++perf3;
    }
  }
  perf4 = timer.stop();
  cout << "         Total Time:  " << perf4 << endl;
  perf1 = perf1 / 50.0f;
  perf2 = perf2 / 50.0f;
  perf4 = perf4 / 100.0f;
  cout << "         Joint:  Avg Err = (" << perf1 << ", " << perf2 << ")deg, " << perf4 << "ms, ";
  perf4 = 1000.0f / perf4;
  cout << perf4 << "Hz, " << perf3 << " misses" << endl;

  arm.MoveToAxisTarget(origAxes);

  cout << "All done" << endl;

  return 0;			// FMP
}


