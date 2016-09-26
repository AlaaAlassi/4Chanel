//  (C) 2001-2014 Force Dimension
//  All Rights Reserved.
//
//  Version 3.5.1



#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include "drdc.h"

#define DEFAULT_K    1000 // 1000
#define DEFAULT_B     10// 10
#define MIN_SCALE      0.2
#define MAX_SCALE      5.0

#define MIN(a,b) ((a)<(b))?(a):(b)
#define MAX(a,b) ((a)>(b))?(a):(b)



int
main (int  argc,
      char **argv)
{
  double mx, my, mz;
  double sx, sy, sz;
  double fx, fy, fz;
  double fcsx, fcsy, fcsz;
  double fsx, fsy, fsz;
  double fmx, fmy, fmz;
  double vhx, vhy, vhz;
  double vex, vey, vez;
  double c1 = 0.5, c2 =0, c3 = 0, c4 = 0.4, c5 = 0 , c6 = 0; //c2 =0.5, c3 = 0.5, c4 = 0, c5 = -0.5 , c6 = -0.5;
  double time;
  double refTime = dhdGetTime ();
  double Km       = 1000;
  double Bm       = 10;
  double Ks       = 1000;
  double Bs       = 10;
  double scale   = 1.0;
  int    done    = 0;
  int    master, slave;
   // c2 = 1+c6;
   // c3 = 1+c5;

  // message
  int major, minor, release, revision;
  dhdGetSDKVersion (&major, &minor, &release, &revision);
  printf ("Force Dimension - Master Slave Example %d.%d.%d.%d\n", major, minor, release, revision);
  printf ("(C) 2014 Force Dimension\n");
  printf ("All Rights Reserved.\n\n");

  // open and initialize 2 devices
  for (int dev=0; dev<2; dev++) {

    // open device
    if (drdOpenID (dev) < 0) {
      printf ("error: not enough devices found\n");
      dhdSleep (2.0);
      for (int j=0; j<=dev; j++) drdClose (j);
      return -1;
    }

    // check that device is supported
    if (!drdIsSupported()) {
      printf ("error: unsupported device (%s)\n", dhdGetSystemName(dev));
      dhdSleep (2.0);
      for (int j=0; j<=dev; j++) drdClose (j);
      return -1;
    }

    // initialize Falcon by hand if necessary
    if (!drdIsInitialized() && dhdGetSystemType() == DHD_DEVICE_FALCON) {
      printf ("please initialize Falcon device...\r"); fflush(stdout);
      while (!drdIsInitialized()) dhdSetForce (0.0, 0.0, 0.0);
      printf ("                                  \r");
      dhdSleep (0.5);
    }

    // initialize if necessary
    if (!drdIsInitialized (dev) && (drdAutoInit (dev) < 0)) {
      printf ("error: initialization failed (%s)\n", dhdErrorGetLastStr ());
      dhdSleep (2.0);
      for (int j=0; j<=dev; j++) drdClose (j);
      return -1;
    }

    // start robot control loop
    if (drdStart (dev) < 0) {
      printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
      dhdSleep (2.0);
      for (int j=0; j<=dev; j++) drdClose (j);
      return -1;
    }
  }

  // default role assignment
  master = 0;
  slave  = 1;

  // prefer Falcon as master 
  if (dhdGetSystemType (0) != DHD_DEVICE_FALCON && dhdGetSystemType (1) == DHD_DEVICE_FALCON) {
    master = 1;
    slave  = 0;
  }

  // give preference to omega.3 as slave
  if (dhdGetSystemType (0) == DHD_DEVICE_OMEGA3 && dhdGetSystemType (1) != DHD_DEVICE_OMEGA3) {
    master = 1;
    slave  = 0;
  }

  // if a device is virtual, make it the master
  if (dhdGetComMode (1) == DHD_COM_MODE_VIRTUAL) {
    master = 1;
    slave  = 0;
  }

  ushort mastersn, slavesn;
  dhdGetSerialNumber (&mastersn, master);
  dhdGetSerialNumber (&slavesn, slave);
  printf ("%s haptic device [sn: %04d] as master\n", dhdGetSystemName(master), mastersn);
  printf ("%s haptic device [sn: %04d] as slave\n", dhdGetSystemName(slave), slavesn);

  // display instructions
  printf ("\n");
  printf ("press 'd' to decrease scaling factor\n");
  printf ("      'u' to increase scaling factor\n");
  printf ("      ',' to decrease virtual stiffness\n");
  printf ("      '.' to increase virtual stiffness\n");
  printf ("      'q' to quit\n\n");

  // center both devices
  drdMoveToPos (0.0, 0.0, 0.0, true, master);
  drdMoveToPos (0.0, 0.0, 0.0, true,  slave);
  while (drdIsMoving (master) || drdIsMoving (slave)) drdWaitForTick (master);

  // stop regulation on master, stop motion filters on slave
  drdStop (true, master);
  drdStop (true, slave);
  dhdSetForce (0.0, 0.0, 0.0, master);
  drdEnableFilter (false, slave);
  
	std::cout << c2 << "  " << c3 << std::endl;
 while (!done) {

    // send the slave to master pos
    dhdGetPosition (&mx, &my, &mz, master);
	dhdGetForce(&fsx, &fsy , &fsz, slave);
	dhdGetForce(&fmx, &fmy , &fmz, master);
	drdGetPos (&sx, &sy, &sz, slave);
	dhdGetLinearVelocity	(&vhx, &vhy, &vhz, master);
	dhdGetLinearVelocity	(&vex, &vey, &vez, slave);
	
    //drdTrackPos (scale*mx, scale*my, scale*mz, slave);
	drdSetEncIGain	(0,slave);
	drdSetEncPGain	(0,slave);
	drdSetEncDGain	(0,slave);
	drdSetEncIGain	(0,master);
	drdSetEncPGain	(0,master);
	drdSetEncDGain	(0,master);
    // compute force and render on master

	

    //fx = -K * (mx-sx/scale)- B*(vhx-vex) + c6*fmx - c2*fsx; fy = -K * (my-sy/scale)-B*(vhy-vey)+ c6*fmy - c2*fsy; fz = -K * (mz-sz/scale)-B*(vhz-vez)+ c6*fmz - c2*fsz;
	fx = -Km*mx + c4*Ks*sx - Bm*vhx + c4*Bs*vex - c2*fsx + c6*fmx  ; fy = -Km*my + c4*Ks*sy - Bm*vhy + c4*Bs*vey - c2*fsy + c6*fmy ;fz = -Km*mz + c4*Ks*sz - Bm*vhz + c4*Bs*vez - c2*fsz + c6*fmz ;
	//- c2*fsx + c6*fmx

	//fcsx =  K * (mx-sx/scale) + B*(vhx-vex) +  c3*fmx - c5*fsx; fcsy = +K * (my-sy/scale)+B*(vhy-vey)+ c3*fmy - c5*fsy; fcsz = +K * (mz-sz/scale)+B*(vhz-vez)+ c3*fmz - c5*fsz;
	fcsx =  c1*Km*mx - Ks*sx + c1*Bm*vhx - Bs*vex  +c3*fmx - c5 *fsx ; fcsy = c1*Km*my - Ks*sy + c1*Bm*vhy - Bs*vey +c3*fmy - c5 *fsy  ;fcsz = c1*Km*mz - Ks*sz + c1*Bm*vhz - Bs*vez +c3*fmz - c5 *fsz  ;
	//+c3*fmx - c5 *fsx 

	//std::cout << fcsx << "  " << fx << std::endl;
    dhdSetForce (fx, fy, fz, master);
	dhdSetForce (fcsx, fcsy, fcsz, slave);


    // print stats and check for exit condition
    time = dhdGetTime ();
    if (time-refTime > 0.04) {
      printf ("scale = %0.03f | Ks = %04d | master %0.02f kHz | slave %0.02f kHz            \r", scale, (int)Ks, dhdGetComFreq (master), drdGetCtrlFreq (slave));
      refTime = time;
      if (dhdKbHit ()){
        switch (dhdKbGet ()) {
        case 'q': done = 1;   break;
        case ',': Ks -= 0.1*Ks; break;
        case '.': Ks += 0.1*Ks; break;
        case 'd': scale = MIN(MAX_SCALE, scale-0.005*scale); break;
        case 'u': scale = MAX(MIN_SCALE, scale+0.005*scale); break;
        }
      }
    }
  }

  // report exit cause
  printf ("                                                                           \r");
  if (done == -1) printf ("\nregulation finished abnormally on slave device\n");
  else            printf ("\nexiting on user request\n");

  // close the connection
  drdClose (slave);
  drdClose (master);

  // exit
  printf ("\ndone.\n");
  return 0;
}

