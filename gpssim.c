#define _CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sched.h>

#include <time.h>
#include <omp.h>
#ifdef _WIN32
#include "getopt.h"
#include <conio.h>
#else
#include <unistd.h>
// For _kbhit() and _getch() on Linux
#include <termios.h>
#ifdef __APPLE__
#include <stdio.h>
#else
#include <stdio_ext.h> // for __fpurge()
#endif
#endif

#include "gpssim.h"
#include "bladegps.h"
#include "../algorithms/libs/environment/frame.h"

int sinTable512[] = {
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250,
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2
};

int cosTable512[] = {
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2,
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250
};

// Receiver antenna attenuation in dB for boresight angle = 0:5:180 [deg]
double ant_pat_db[37] = {
	 0.00,  0.00,  0.22,  0.44,  0.67,  1.11,  1.56,  2.00,  2.44,  2.89,  3.56,  4.22,
	 4.89,  5.56,  6.22,  6.89,  7.56,  8.22,  8.89,  9.78, 10.67, 11.56, 12.44, 13.33,
	14.44, 15.56, 16.67, 17.78, 18.89, 20.00, 21.33, 22.67, 24.00, 25.56, 27.33, 29.33,
	31.56
};

	// Environment
static TimeSystem *time_system;
static earth *earth_;
static moon *moon_;
static Frame *frame;
static gps_satellite gps_sats[MAX_SAT];

/*! \brief Subtract two vectors of double
 *  \param[out] y Result of subtraction
 *  \param[in] x1 Minuend of subtracion
 *  \param[in] x2 Subtrahend of subtracion
 */
void subVect(double *y, const double *x1, const double *x2)
{
	y[0] = x1[0]-x2[0];
	y[1] = x1[1]-x2[1];
	y[2] = x1[2]-x2[2];

	return;
}

/*! \brief Compute Norm of Vector
 *  \param[in] x Input vector
 *  \returns Length (Norm) of the input vector
 */
double normVect(const double *x)
{
	return(sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]));
}

/*! \brief Compute dot-product of two vectors
 *  \param[in] x1 First multiplicand
 *  \param[in] x2 Second multiplicand
 *  \returns Dot-product of both multiplicands
 */
double dotProd(const double *x1, const double *x2)
{
	return(x1[0]*x2[0]+x1[1]*x2[1]+x1[2]*x2[2]);
}

/* !\brief generate the C/A code sequence for a given Satellite Vehicle PRN
 *  \param[in] prn PRN nuber of the Satellite Vehicle
 *  \param[out] ca Caller-allocated integer array of 1023 bytes
 */
void codegen(int *ca, int prn)
{
	int delay[] = {
		  5,   6,   7,   8,  17,  18, 139, 140, 141, 251,
		252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
		473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
		861, 862};

	int g1[CA_SEQ_LEN], g2[CA_SEQ_LEN];
	int r1[N_DWRD_SBF], r2[N_DWRD_SBF];
	int c1, c2;
	int i,j;

	if (prn<1 || prn>32)
		return;

	for (i=0; i<N_DWRD_SBF; i++)
		r1[i] = r2[i] = -1;

	for (i=0; i<CA_SEQ_LEN; i++)
	{
		g1[i] = r1[9];
		g2[i] = r2[9];
		c1 = r1[2]*r1[9];
		c2 = r2[1]*r2[2]*r2[5]*r2[7]*r2[8]*r2[9];

		for (j=9; j>0; j--)
		{
			r1[j] = r1[j-1];
			r2[j] = r2[j-1];
		}
		r1[0] = c1;
		r2[0] = c2;
	}

	for (i=0,j=CA_SEQ_LEN-delay[prn-1]; i<CA_SEQ_LEN; i++,j++)
		ca[i] = (1-g1[i]*g2[j%CA_SEQ_LEN])/2;

	return;
}

/*! \brief Convert a UTC date into a GPS date
 *  \param[in] t input date in UTC form
 *  \param[out] g output date in GPS form
 */
void date2gps(const datetime_t *t, gpstime_t *g)
{
	int doy[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
	int ye;
	int de;
	int lpdays;

	ye = t->y - 1980;

	// Compute the number of leap days since Jan 5/Jan 6, 1980.
	lpdays = ye/4 + 1;
	if ((ye%4)==0 && t->m<=2)
		lpdays--;

	// Compute the number of days elapsed since Jan 5/Jan 6, 1980.
	de = ye*365 + doy[t->m-1] + t->d + lpdays - 6;

	// Convert time to GPS weeks and seconds.
	g->week = de / 7;
	g->sec = (double)(de%7)*SECONDS_IN_DAY + t->hh*SECONDS_IN_HOUR
		+ t->mm*SECONDS_IN_MINUTE + t->sec + SECONDS_UTC_TO_GPS;

	return;
}

void gps2date(const gpstime_t *g, datetime_t *t)
{
	long c,d,e,f;
	double gsec = round(g->sec - SECONDS_UTC_TO_GPS);

	// Convert Julian day number to calendar date
	c = (long)(7.0*(double)g->week + floor(gsec/86400.0)+2444245.0) + 1537;
	d = (long)(((double)c-122.1)/365.25);
	e = 365*d + d/4;
	f = (long)((double)(c-e)/30.6001 );

	t->d = c - e - (int)(30.6001*f);
	t->m = f - 1 - 12*(f/14);
	t->y  = d - 4715 - ((7 + t->m)/10);

	t->hh = ((int)(gsec/3600.0))%24;
	t->mm = ((int)(gsec/60.0))%60;
	t->sec = gsec - 60.0*floor(gsec/60.0);

	return;
}

/*! \brief Convert Earth-centered Earth-fixed (ECEF) into Lat/Long/Heighth
 *  \param[in] xyz Input Array of X, Y and Z ECEF coordinates
 *  \param[out] llh Output Array of Latitude, Longitude and Height
 */
void xyz2llh(const double *xyz, double *llh)
{
	double a,eps,e,e2;
	double x,y,z;
	double rho2,dz,zdz,nh,slat,n,dz_new;

	a = WGS84_RADIUS;
	e = WGS84_ECCENTRICITY;

	eps = 1.0e-3;
	e2 = e*e;

	if (normVect(xyz)<eps)
	{
		// Invalid ECEF vector
		llh[0] = 0.0;
		llh[1] = 0.0;
		llh[2] = -a;

		return;
	}

	x = xyz[0];
	y = xyz[1];
	z = xyz[2];

	rho2 = x*x + y*y;
	dz = e2*z;

	while (1)
	{
		zdz = z + dz;
		nh = sqrt(rho2 + zdz*zdz);
		slat = zdz / nh;
		n = a / sqrt(1.0-e2*slat*slat);
		dz_new = n*e2*slat;

		if (fabs(dz-dz_new) < eps)
			break;

		dz = dz_new;
	}

	llh[0] = atan2(zdz, sqrt(rho2));
	llh[1] = atan2(y, x);
	llh[2] = nh - n;

	return;
}

/*! \brief Convert Lat/Long/Height into Earth-centered Earth-fixed (ECEF)
 *  \param[in] llh Input Array of Latitude, Longitude and Height
 *  \param[out] xyz Output Array of X, Y and Z ECEF coordinates
 */
void llh2xyz(const double *llh, double *xyz)
{
	double n;
	double a;
	double e;
	double e2;
	double clat;
	double slat;
	double clon;
	double slon;
	double d,nph;
	double tmp;

	a = WGS84_RADIUS;
	e = WGS84_ECCENTRICITY;
	e2 = e*e;

	clat = cos(llh[0]);
	slat = sin(llh[0]);
	clon = cos(llh[1]);
	slon = sin(llh[1]);
	d = e*slat;

	n = a/sqrt(1.0-d*d);
	nph = n + llh[2];

	tmp = nph*clat;
	xyz[0] = tmp*clon;
	xyz[1] = tmp*slon;
	xyz[2] = ((1.0-e2)*n + llh[2])*slat;

	return;
}

/*! \brief Compute the intermediate matrix for LLH to ECEF
 *  \param[in] llh Input position in Latitude-Longitude-Height format
 *  \param[out] t Three-by-Three output matrix
 */
void ltcmat(const double *llh, double t[3][3])
{
	double slat, clat;
	double slon, clon;

	slat = sin(llh[0]);
	clat = cos(llh[0]);
	slon = sin(llh[1]);
	clon = cos(llh[1]);

	t[0][0] = -slat*clon;
	t[0][1] = -slat*slon;
	t[0][2] = clat;
	t[1][0] = -slon;
	t[1][1] = clon;
	t[1][2] = 0.0;
	t[2][0] = clat*clon;
	t[2][1] = clat*slon;
	t[2][2] = slat;

	return;
}

/*! \brief Convert Earth-centered Earth-Fixed to North-East-Up
 *  \param[in] xyz Input position as vector in ECEF format
 *  \param[in] t Intermediate matrix computed by \ref ltcmat
 *  \param[out] neu Output position as North-East-Up format
 */
void ecef2neu(const double *xyz, double t[3][3], double *neu)
{
	neu[0] = t[0][0]*xyz[0] + t[0][1]*xyz[1] + t[0][2]*xyz[2];
	neu[1] = t[1][0]*xyz[0] + t[1][1]*xyz[1] + t[1][2]*xyz[2];
	neu[2] = t[2][0]*xyz[0] + t[2][1]*xyz[1] + t[2][2]*xyz[2];

	return;
}

/*! \brief Convert North-East-Up to Antenna-fixed North-East-Up
 *  \param[in] neu Input vector in North-East-Up format
 *  \param[in] ant_dir Input antenna direction in Azimuth and Elevation
 *  \param[out] neu_ant Output vector in Antenna-fixed NEU format
 */
void neu2ant(const double *neu, const double *ant_dir, double *neu_ant)
{
	double cos_az = cos(ant_dir[0]);
	double sin_az = sin(ant_dir[0]);
	double cos_el = cos(ant_dir[1]);
	double sin_el = sin(ant_dir[1]);

	neu_ant[0] = sin_el*cos_az*neu[0] -sin_az*neu[1] + cos_el*cos_az*neu[2];
	neu_ant[1] = sin_el*sin_az*neu[0] + cos_az*neu[1] + cos_el*sin_az*neu[2];
	neu_ant[2] = -cos_el*neu[0] + sin_el*neu[2];

	return;
}

/*! \brief Convert North-Eeast-Up to Azimuth + Elevation
 *  \param[in] neu Input position in North-East-Up format
 *  \param[out] azel Output array of azimuth + elevation as double
 */
void neu2azel(double *azel, const double *neu)
{
	double ne;

	azel[0] = atan2(neu[1],neu[0]);
	if (azel[0]<0.0)
		azel[0] += (2.0*PI);

	ne = sqrt(neu[0]*neu[0] + neu[1]*neu[1]);
	azel[1] = atan(neu[2] / ne);

	return;
}

/*! \brief Compute Satellite position, velocity and clock at given time
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at which position is to be computed
 *  \param[out] pos Computed position (vector)
 *  \param[out] vel Computed velociy (vector)
 *  \param[out] clk Computed clock
 */
void satpos(const ephem_t eph, const gpstime_t g, double *pos, double *vel, double *clk)
{
	// Computing Satellite Velocity using the Broadcast Ephemeris
	// http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

	double tk;
	double mk;
	double ek;
	double ekold;
	double ekdot;
	double cek,sek;
	double pk;
	double pkdot;
	double c2pk,s2pk;
	double uk;
	double ukdot;
	double cuk,suk;
	double ok;
	double sok,cok;
	double ik;
	double ikdot;
	double sik,cik;
	double rk;
	double rkdot;
	double xpk,ypk;
	double xpkdot,ypkdot;

	double relativistic, OneMinusecosE, tmp;

	tk = g.sec - eph.toe.sec;

	if(tk>SECONDS_IN_HALF_WEEK)
		tk -= SECONDS_IN_WEEK;
	else if(tk<-SECONDS_IN_HALF_WEEK)
		tk += SECONDS_IN_WEEK;

	mk = eph.m0 + eph.n*tk;
	ek = mk;
	ekold = ek + 1.0;

	OneMinusecosE = 0; // Suppress the uninitialized warning.
	while(fabs(ek-ekold)>1.0E-14)
	{
		ekold = ek;
		OneMinusecosE = 1.0-eph.ecc*cos(ekold);
		ek = ek + (mk-ekold+eph.ecc*sin(ekold))/OneMinusecosE;
	}

	sek = sin(ek);
	cek = cos(ek);

	ekdot = eph.n/OneMinusecosE;

	relativistic = -4.442807633E-10*eph.ecc*eph.sqrta*sek;

	pk = atan2(eph.sq1e2*sek,cek-eph.ecc) + eph.aop;
	pkdot = eph.sq1e2*ekdot/OneMinusecosE;

	s2pk = sin(2.0*pk);
	c2pk = cos(2.0*pk);

	uk = pk + eph.cus*s2pk + eph.cuc*c2pk;
	suk = sin(uk);
	cuk = cos(uk);
	ukdot = pkdot*(1.0 + 2.0*(eph.cus*c2pk - eph.cuc*s2pk));

	rk = eph.A*OneMinusecosE + eph.crc*c2pk + eph.crs*s2pk;
	rkdot = eph.A*eph.ecc*sek*ekdot + 2.0*pkdot*(eph.crs*c2pk - eph.crc*s2pk);

	ik = eph.inc0 + eph.idot*tk + eph.cic*c2pk + eph.cis*s2pk;
	sik = sin(ik);
	cik = cos(ik);
	ikdot = eph.idot + 2.0*pkdot*(eph.cis*c2pk - eph.cic*s2pk);

	xpk = rk*cuk;
	ypk = rk*suk;
	xpkdot = rkdot*cuk - ypk*ukdot;
	ypkdot = rkdot*suk + xpk*ukdot;

	ok = eph.omg0 + tk*eph.omgkdot - OMEGA_EARTH*eph.toe.sec;
	sok = sin(ok);
	cok = cos(ok);

	pos[0] = xpk*cok - ypk*cik*sok;
	pos[1] = xpk*sok + ypk*cik*cok;
	pos[2] = ypk*sik;

	tmp = ypkdot*cik - ypk*sik*ikdot;

	vel[0] = -eph.omgkdot*pos[1] + xpkdot*cok - tmp*sok;
	vel[1] = eph.omgkdot*pos[0] + xpkdot*sok + tmp*cok;
	vel[2] = ypk*cik*ikdot + ypkdot*sik;

	// Satellite clock correction
	tk = g.sec - eph.toc.sec;

	if(tk>SECONDS_IN_HALF_WEEK)
		tk -= SECONDS_IN_WEEK;
	else if(tk<-SECONDS_IN_HALF_WEEK)
		tk += SECONDS_IN_WEEK;

	clk[0] = eph.af0 + tk*(eph.af1 + tk*eph.af2) + relativistic - eph.tgd;
	clk[1] = eph.af1 + 2.0*tk*eph.af2;

	return;
}

/*! \brief Compute Subframe from Ephemeris
 *  \param[in] eph Ephemeris of given SV
 *  \param[in] ionoutc ionosphere information
 *  \param[in] alm almanac information
 *  \param[out] sbf Array of five sub-frames, 10 long words each
 */

#define EMPTY_WORD 0xaaaaaaaaUL

void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, const almanac_t *alm, unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF])
{
	unsigned long wn;
	unsigned long toe;
	unsigned long toc;
	unsigned long iode;
	unsigned long iodc;
	long deltan;
	long cuc;
	long cus;
	long cic;
	long cis;
	long crc;
	long crs;
	unsigned long ecc;
	unsigned long sqrta;
	long m0;
	long omg0;
	long inc0;
	long aop;
	long omgdot;
	long idot;
	long af0;
	long af1;
	long af2;
	long tgd;

	unsigned long ura = 0UL;
	unsigned long dataId = 1UL;

	unsigned long wna;
	unsigned long toa;

	signed long alpha0,alpha1,alpha2,alpha3;
	signed long beta0,beta1,beta2,beta3;
	signed long A0,A1;
	signed long dtls,dtlsf;
	unsigned long tot,wnt,wnlsf,dn;

	int sv,i;
	unsigned long svId;
	signed long deli; // Relative to i0 = 0.30 semicircles

	unsigned long sbf4_svId[25] = {
		57UL, 0UL, 0UL, 0UL, 0UL, 57UL, 0UL, 0UL, 0UL, 0UL,
		57UL, 62UL, 52UL, 53UL, 54UL, 57UL, 55UL, 56UL, 58UL,
		59UL, 57UL, 60UL, 61UL, 62UL, 63UL };
	unsigned long sbf5_svId[25] = {
		0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
		0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
		0UL, 0UL, 0UL, 0UL, 51UL };

	// FIXED: This has to be the "transmission" week number, not for the ephemeris reference time
	//wn = (unsigned long)(eph.toe.week%1024);
	wn = 0UL;
	toe = (unsigned long)(eph.toe.sec/16.0);
	toc = (unsigned long)(eph.toc.sec/16.0);
	iode = (unsigned long)(eph.iode);
	iodc = (unsigned long)(eph.iodc);
	deltan = (long)(eph.deltan/POW2_M43/PI);
	cuc = (long)(eph.cuc/POW2_M29);
	cus = (long)(eph.cus/POW2_M29);
	cic = (long)(eph.cic/POW2_M29);
	cis = (long)(eph.cis/POW2_M29);
	crc = (long)(eph.crc/POW2_M5);
	crs = (long)(eph.crs/POW2_M5);
	ecc = (unsigned long)(eph.ecc/POW2_M33);
	sqrta = (unsigned long)(eph.sqrta/POW2_M19);
	m0 = (long)(eph.m0/POW2_M31/PI);
	omg0 = (long)(eph.omg0/POW2_M31/PI);
	inc0 = (long)(eph.inc0/POW2_M31/PI);
	aop = (long)(eph.aop/POW2_M31/PI);
	omgdot = (long)(eph.omgdot/POW2_M43/PI);
	idot = (long)(eph.idot/POW2_M43/PI);
	af0 = (long)(eph.af0/POW2_M31);
	af1 = (long)(eph.af1/POW2_M43);
	af2 = (long)(eph.af2/POW2_M55);
	tgd = (long)(eph.tgd/POW2_M31);

	alpha0 = (signed long)round(ionoutc.alpha0/POW2_M30);
	alpha1 = (signed long)round(ionoutc.alpha1/POW2_M27);
	alpha2 = (signed long)round(ionoutc.alpha2/POW2_M24);
	alpha3 = (signed long)round(ionoutc.alpha3/POW2_M24);
	beta0 = (signed long)round(ionoutc.beta0/2048.0);
	beta1 = (signed long)round(ionoutc.beta1/16384.0);
	beta2 = (signed long)round(ionoutc.beta2/65536.0);
	beta3 = (signed long)round(ionoutc.beta3/65536.0);
	A0 = (signed long)round(ionoutc.A0/POW2_M30);
	A1 = (signed long)round(ionoutc.A1/POW2_M50);
	dtls = (signed long)(ionoutc.dtls);
	tot = (unsigned long)(ionoutc.tot/4096);
	wnt = (unsigned long)(ionoutc.wnt%256);
	// TO DO: Specify scheduled leap seconds in command options
	// 2016/12/31 (Sat) -> WNlsf = 1929, DN = 7 (http://navigationservices.agi.com/GNSSWeb/)
	// Days are counted from 1 to 7 (Sunday is 1).
	wnlsf = 1929%256;
	dn = 7;
	dtlsf = 18;

	// Subframe 1
	sbf[0][0] = 0x8B0000UL<<6;
	sbf[0][1] = 0x1UL<<8;
	sbf[0][2] = ((wn&0x3FFUL)<<20) | (ura<<14) | (((iodc>>8)&0x3UL)<<6);
	sbf[0][3] = 0UL;
	sbf[0][4] = 0UL;
	sbf[0][5] = 0UL;
	sbf[0][6] = (tgd&0xFFUL)<<6;
	sbf[0][7] = ((iodc&0xFFUL)<<22) | ((toc&0xFFFFUL)<<6);
	sbf[0][8] = ((af2&0xFFUL)<<22) | ((af1&0xFFFFUL)<<6);
	sbf[0][9] = (af0&0x3FFFFFUL)<<8;

	// Subframe 2
	sbf[1][0] = 0x8B0000UL<<6;
	sbf[1][1] = 0x2UL<<8;
	sbf[1][2] = ((iode&0xFFUL)<<22) | ((crs&0xFFFFUL)<<6);
	sbf[1][3] = ((deltan&0xFFFFUL)<<14) | (((m0>>24)&0xFFUL)<<6);
	sbf[1][4] = (m0&0xFFFFFFUL)<<6;
	sbf[1][5] = ((cuc&0xFFFFUL)<<14) | (((ecc>>24)&0xFFUL)<<6);
	sbf[1][6] = (ecc&0xFFFFFFUL)<<6;
	sbf[1][7] = ((cus&0xFFFFUL)<<14) | (((sqrta>>24)&0xFFUL)<<6);
	sbf[1][8] = (sqrta&0xFFFFFFUL)<<6;
	sbf[1][9] = (toe&0xFFFFUL)<<14;

	// Subframe 3
	sbf[2][0] = 0x8B0000UL<<6;
	sbf[2][1] = 0x3UL<<8;
	sbf[2][2] = ((cic&0xFFFFUL)<<14) | (((omg0>>24)&0xFFUL)<<6);
	sbf[2][3] = (omg0&0xFFFFFFUL)<<6;
	sbf[2][4] = ((cis&0xFFFFUL)<<14) | (((inc0>>24)&0xFFUL)<<6);
	sbf[2][5] = (inc0&0xFFFFFFUL)<<6;
	sbf[2][6] = ((crc&0xFFFFUL)<<14) | (((aop>>24)&0xFFUL)<<6);
	sbf[2][7] = (aop&0xFFFFFFUL)<<6;
	sbf[2][8] = (omgdot&0xFFFFFFUL)<<6;
	sbf[2][9] = ((iode&0xFFUL)<<22) | ((idot&0x3FFFUL)<<8);

	// Empty all the pages of subframes 4 and 5
	for (i = 0; i < 25; i++)
	{
		svId = 0UL; // Dummy SV
		//svId = sbf4_svId[i];

		sbf[3 + i * 2][0] = 0x8B0000UL << 6; // Preamble for TLM
		sbf[3 + i * 2][1] = 0x4UL << 8; // Subframe ID for HOW
		sbf[3 + i * 2][2] = (dataId << 28) | (svId << 22) | ((EMPTY_WORD & 0xFFFFUL) << 6);
		sbf[3 + i * 2][3] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[3 + i * 2][4] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[3 + i * 2][5] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[3 + i * 2][6] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[3 + i * 2][7] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[3 + i * 2][8] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[3 + i * 2][9] = (EMPTY_WORD & 0x3FFFFFUL) << 8;

		//svId = sbf5_svId[i];

		sbf[4 + i * 2][0] = 0x8B0000UL << 6; // Preamble for TLM
		sbf[4 + i * 2][1] = 0x5UL << 8; // Subframe ID for HOW
		sbf[4 + i * 2][2] = (dataId << 28) | (svId << 22) | ((EMPTY_WORD & 0xFFFFUL) << 6);
		sbf[4 + i * 2][3] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[4 + i * 2][4] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[4 + i * 2][5] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[4 + i * 2][6] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[4 + i * 2][7] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[4 + i * 2][8] = (EMPTY_WORD & 0xFFFFFFUL) << 6;
		sbf[4 + i * 2][9] = (EMPTY_WORD & 0x3FFFFFUL) << 8;
	}

	// Subframe 4, pages 2-5 and 7-10: almanac data for PRN 25 through 32
	for (sv = 24; sv < MAX_SAT; sv++)
	{
		if (sv >= 24 && sv <= 27) // PRN 25-28
			i = sv - 23; // Pages 2-5 (i = 1-4)
		else if (sv >= 28 && sv < MAX_SAT) // PRN 29-32
			i = sv - 22; // Page 7-10 (i = 6-9)

		if (alm[sv].id != 0)
		{
			svId = (unsigned long)(sv + 1);
			ecc = (unsigned long)(alm[sv].ecc / POW2_M21);
			toa = (unsigned long)(alm[sv].toa.sec / 4096.0);
			deli = (signed long)((alm[sv].inc0 / PI - 0.3) / POW2_M19);
			omgdot = (signed long)(alm[sv].omgdot / PI / POW2_M38);
			sqrta = (unsigned long)(alm[sv].sqrta / POW2_M11);
			omg0 = (signed long)(alm[sv].omg0 / PI / POW2_M23);
			aop = (signed long)(alm[sv].aop / PI / POW2_M23);
			m0 = (signed long)(alm[sv].m0 / PI / POW2_M23);
			af0 = (signed long)(alm[sv].af0 / POW2_M20);
			af1 = (signed long)(alm[sv].af1 / POW2_M38);

			sbf[3 + i * 2][0] = 0x8B0000UL << 6; // Preamble for TLM
			sbf[3 + i * 2][1] = 0x4UL << 8; // Subframe ID for HOW
			sbf[3 + i * 2][2] = (dataId << 28) | (svId << 22) | ((ecc & 0xFFFFUL) << 6);
			sbf[3 + i * 2][3] = ((toa & 0xFFUL) << 22) | ((deli & 0xFFFFUL) << 6);
			sbf[3 + i * 2][4] = ((omgdot & 0xFFFFUL) << 14); // SV HEALTH = 000 (ALL DATA OK)
			sbf[3 + i * 2][5] = ((sqrta & 0xFFFFFFUL) << 6);
			sbf[3 + i * 2][6] = ((omg0 & 0xFFFFFFUL) << 6);
			sbf[3 + i * 2][7] = ((aop & 0xFFFFFFUL) << 6);
			sbf[3 + i * 2][8] = ((m0 & 0xFFFFFFUL) << 6);
			sbf[3 + i * 2][9] = ((af0 & 0x7F8UL) << 19) | ((af1 & 0x7FFUL) << 11) | ((af0 & 0x7UL) << 8);
		}
	}

	// Subframe 4, page 18: ionospheric and UTC data
	if (ionoutc.vflg==TRUE)
	{
		sbf[3 + 17 * 2][0] = 0x8B0000UL<<6;
		sbf[3 + 17 * 2][1] = 0x4UL<<8;
		sbf[3 + 17 * 2][2] = (dataId<<28) | (sbf4_svId[17]<<22) | ((alpha0&0xFFUL)<<14) | ((alpha1&0xFFUL)<<6);
		sbf[3 + 17 * 2][3] = ((alpha2&0xFFUL)<<22) | ((alpha3&0xFFUL)<<14) | ((beta0&0xFFUL)<<6);
		sbf[3 + 17 * 2][4] = ((beta1&0xFFUL)<<22) | ((beta2&0xFFUL)<<14) | ((beta3&0xFFUL)<<6);
		sbf[3 + 17 * 2][5] = (A1&0xFFFFFFUL)<<6;
		sbf[3 + 17 * 2][6] = ((A0>>8)&0xFFFFFFUL)<<6;
		sbf[3 + 17 * 2][7] = ((A0&0xFFUL)<<22) | ((tot&0xFFUL)<<14) | ((wnt&0xFFUL)<<6);
		sbf[3 + 17 * 2][8] = ((dtls&0xFFUL)<<22) | ((wnlsf&0xFFUL)<<14) | ((dn&0xFFUL)<<6);
		sbf[3 + 17 * 2][9] = (dtlsf&0xFFUL)<<22;

	}

	// Subframe 4, page 25: SV health data for PRN 25 through 32
	sbf[3 + 24 * 2][0] = 0x8B0000UL<<6;
	sbf[3 + 24 * 2][1] = 0x4UL<<8;
	sbf[3 + 24 * 2][2] = (dataId<<28) | (sbf4_svId[24]<<22);
	sbf[3 + 24 * 2][3] = 0UL;
	sbf[3 + 24 * 2][4] = 0UL;
	sbf[3 + 24 * 2][5] = 0UL;
	sbf[3 + 24 * 2][6] = 0UL;
	sbf[3 + 24 * 2][7] = 0UL;
	sbf[3 + 24 * 2][8] = 0UL;
	sbf[3 + 24 * 2][9] = 0UL;

	// Subframe 5, page 1-24: almanac data for PRN 1 through 24
	for (sv = 0; sv < 24; sv++)
	{
		i = sv;

		if (alm[sv].id != 0)
		{
			svId = (unsigned long)(sv + 1);
			ecc = (unsigned long)(alm[sv].ecc / POW2_M21);
			toa = (unsigned long)(alm[sv].toa.sec / 4096.0);
			deli = (signed long)((alm[sv].inc0 / PI - 0.3) / POW2_M19);
			omgdot = (signed long)(alm[sv].omgdot / PI / POW2_M38);
			sqrta = (unsigned long)(alm[sv].sqrta / POW2_M11);
			omg0 = (signed long)(alm[sv].omg0 / PI / POW2_M23);
			aop = (signed long)(alm[sv].aop / PI / POW2_M23);
			m0 = (signed long)(alm[sv].m0 / PI / POW2_M23);
			af0 = (signed long)(alm[sv].af0 / POW2_M20);
			af1 = (signed long)(alm[sv].af1 / POW2_M38);

			sbf[4 + i * 2][0] = 0x8B0000UL << 6; // Preamble
			sbf[4 + i * 2][1] = 0x5UL << 8; // Subframe ID
			sbf[4 + i * 2][2] = (dataId << 28) | (svId << 22) | ((ecc & 0xFFFFUL) << 6);
			sbf[4 + i * 2][3] = ((toa & 0xFFUL) << 22) | ((deli & 0xFFFFUL) << 6);
			sbf[4 + i * 2][4] = ((omgdot & 0xFFFFUL) << 14); // SV HEALTH = 000 (ALL DATA OK)
			sbf[4 + i * 2][5] = ((sqrta & 0xFFFFFFUL) << 6);
			sbf[4 + i * 2][6] = ((omg0 & 0xFFFFFFUL) << 6);
			sbf[4 + i * 2][7] = ((aop & 0xFFFFFFUL) << 6);
			sbf[4 + i * 2][8] = ((m0 & 0xFFFFFFUL) << 6);
			sbf[4 + i * 2][9] = ((af0 & 0x7F8UL) << 19) | ((af1 & 0x7FFUL) << 11) | ((af0 & 0x7UL) << 8);
		}
	}

	// Subframe 5, page 25: SV health data for PRN 1 through 24
	wna = (unsigned long)(eph.toe.week % 256);
	toa = (unsigned long)(eph.toe.sec / 4096.0);

	for (sv = 0; sv < MAX_SAT; sv++)
	{
		if (alm[sv].id != 0) // Valid almanac is availabe
		{
			wna = (unsigned long)(alm[sv].toa.week % 256);
			toa = (unsigned long)(alm[sv].toa.sec / 4096.0);
			break;
		}
	}

	sbf[4 + 24 * 2][0] = 0x8B0000UL<<6;
	sbf[4 + 24 * 2][1] = 0x5UL<<8;
	sbf[4 + 24 * 2][2] = (dataId<<28) | (sbf5_svId[24]<<22) | ((toa&0xFFUL)<<14) | ((wna&0xFFUL)<<6);
	sbf[4 + 24 * 2][3] = 0UL;
	sbf[4 + 24 * 2][4] = 0UL;
	sbf[4 + 24 * 2][5] = 0UL;
	sbf[4 + 24 * 2][6] = 0UL;
	sbf[4 + 24 * 2][7] = 0UL;
	sbf[4 + 24 * 2][8] = 0UL;
	sbf[4 + 24 * 2][9] = 0UL;

	return;
}

/*! \brief Count number of bits set to 1
 *  \param[in] v long word in whihc bits are counted
 *  \returns Count of bits set to 1
 */
unsigned long countBits(unsigned long v)
{
	unsigned long c;
	const int S[] = {1, 2, 4, 8, 16};
	const unsigned long B[] = {
		0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF};

	c = v;
	c = ((c >> S[0]) & B[0]) + (c & B[0]);
	c = ((c >> S[1]) & B[1]) + (c & B[1]);
	c = ((c >> S[2]) & B[2]) + (c & B[2]);
	c = ((c >> S[3]) & B[3]) + (c & B[3]);
	c = ((c >> S[4]) & B[4]) + (c & B[4]);

	return(c);
}

/*! \brief Compute the Checksum for one given word of a subframe
 *  \param[in] source The input data
 *  \param[in] nib Does this word contain non-information-bearing bits?
 *  \returns Computed Checksum
 */
unsigned long computeChecksum(unsigned long source, int nib)
{
	/*
	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
	Bits 29 to  6 = Source data bits, d1, d2, ..., d24
	Bits  5 to  0 = Empty parity bits
	*/

	/*
	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
	Bits 29 to  6 = Data bits transmitted by the SV, D1, D2, ..., D24
	Bits  5 to  0 = Computed parity bits, D25, D26, ..., D30
	*/

	/*
	                  1            2           3
	bit    12 3456 7890 1234 5678 9012 3456 7890
	---    -------------------------------------
	D25    11 1011 0001 1111 0011 0100 1000 0000
	D26    01 1101 1000 1111 1001 1010 0100 0000
	D27    10 1110 1100 0111 1100 1101 0000 0000
	D28    01 0111 0110 0011 1110 0110 1000 0000
	D29    10 1011 1011 0001 1111 0011 0100 0000
	D30    00 1011 0111 1010 1000 1001 1100 0000
	*/

	unsigned long bmask[6] = {
		0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
		0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL };

	unsigned long D;
	unsigned long d = source & 0x3FFFFFC0UL;
	unsigned long D29 = (source>>31)&0x1UL;
	unsigned long D30 = (source>>30)&0x1UL;

	if (nib) // Non-information bearing bits for word 2 and 10
	{
		/*
		Solve bits 23 and 24 to presearve parity check
		with zeros in bits 29 and 30.
		*/

		if ((D30 + countBits(bmask[4] & d)) % 2)
			d ^= (0x1UL<<6);
		if ((D29 + countBits(bmask[5] & d)) % 2)
			d ^= (0x1UL<<7);
	}

	D = d;
	if (D30)
		D ^= 0x3FFFFFC0UL;

	D |= ((D29 + countBits(bmask[0] & d)) % 2) << 5;
	D |= ((D30 + countBits(bmask[1] & d)) % 2) << 4;
	D |= ((D29 + countBits(bmask[2] & d)) % 2) << 3;
	D |= ((D30 + countBits(bmask[3] & d)) % 2) << 2;
	D |= ((D30 + countBits(bmask[4] & d)) % 2) << 1;
	D |= ((D29 + countBits(bmask[5] & d)) % 2);

	D &= 0x3FFFFFFFUL;
	//D |= (source & 0xC0000000UL); // Add D29* and D30* from source data bits

	return(D);
}

/*! \brief Replace all 'E' exponential designators to 'D'
 *  \param str String in which all occurrences of 'E' are replaced with *  'D'
 *  \param len Length of input string in bytes
 *  \returns Number of characters replaced
 */
int replaceExpDesignator(char *str, int len)
{
	int i,n=0;

	for (i=0; i<len; i++)
	{
		if (str[i]=='D')
		{
			n++;
			str[i] = 'E';
		}
	}

	return(n);
}

double subGpsTime(gpstime_t g1, gpstime_t g0)
{
	double dt;

	dt = g1.sec - g0.sec;
	dt += (double)(g1.week - g0.week) * SECONDS_IN_WEEK;

	return(dt);
}

gpstime_t incGpsTime(gpstime_t g0, double dt)
{
	gpstime_t g1;

	g1.week = g0.week;
	g1.sec = g0.sec + dt;

	g1.sec = round(g1.sec*1000.0)/1000.0; // Avoid rounding error

	while (g1.sec>=SECONDS_IN_WEEK)
	{
		g1.sec -= SECONDS_IN_WEEK;
		g1.week++;
	}

	while (g1.sec<0.0)
	{
		g1.sec += SECONDS_IN_WEEK;
		g1.week--;
	}

	return(g1);
}

int readAlmanac(almanac_t alm[MAX_SAT], const char *fname)
{
	FILE *fp;
	int sv;

	char str[MAX_CHAR];
	int ialm = 0;

	// Clear SV IDs
	for (sv = 0; sv < MAX_SAT; sv++)
		alm[sv].id = 0; // Idle satellite

	// Open almanac file
	if (NULL == (fp = fopen(fname, "rt")))
		return(-1);

	// Read almanac file
	while (1)
	{
		if (NULL == fgets(str, MAX_CHAR, fp))
			break;

		if (strlen(str) < 25)
			continue; // Skip empty line

		if (str[0] == '*')
		{
			// ID
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			sv = atoi(str + 26) - 1;

			if (sv<0 || sv>MAX_SAT)
				break;

			// Health
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].health = atoi(str + 26);

			// Eccentricity
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].ecc = atof(str + 26);

			// Time of Applicability(s)
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].toa.sec = atof(str + 26);

			// Orbital Inclination(rad)
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].inc0 = atof(str + 26);

			// Rate of Right Ascen(r/s)
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].omgdot = atof(str + 26);

			// SQRT(A)  (m 1 / 2)
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].sqrta = atof(str + 26);

			// Right Ascen at Week(rad)
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].omg0 = atof(str + 26);

			// Argument of Perigee(rad)
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].aop = atof(str + 26);

			// Mean Anom(rad)
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].m0 = atof(str + 26);

			// Af0(s)
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].af0 = atof(str + 26);

			// Af1(s/s)
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].af1 = atof(str + 26);

			// Week
			if (NULL == fgets(str, MAX_CHAR, fp))
				break;

			alm[sv].toa.week = atoi(str + 26);
			// GPS week rollover
			if (alm[sv].toa.week > 500)
				alm[sv].toa.week += 1024;
			else
				alm[sv].toa.week += 2048;

			// Valid almanac
			alm[sv].id = sv + 1;
			ialm++;
		}
	}

	return(ialm); // Number of valid almanac
}

/*! \brief Read Ephemersi data from the RINEX Navigation file */
/*  \param[out] eph Array of Output SV ephemeris data
 *  \param[out] ionoutc Pointer of ionosphere struct (ionoutc_t)
 *  \param[in] fname File name of the RINEX file
 *  \returns Number of sets of ephemerides in the file
 */
int readRinexNavAll(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname)
{
	FILE *fp;
	int ieph;

	int sv;
	char str[MAX_CHAR];
	char tmp[20];

	datetime_t t;
	gpstime_t g;
	gpstime_t g0;
	double dt;

	int flags = 0x0;

	if (NULL==(fp=fopen(fname, "rt")))
		return(-1);

	// Clear valid flag
	for (ieph=0; ieph<EPHEM_ARRAY_SIZE; ieph++)
		for (sv=0; sv<MAX_SAT; sv++)
			eph[ieph][sv].vflg = 0;

	// Read header lines
	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		if (strncmp(str+60, "END OF HEADER", 13)==0)
			break;
		else if (strncmp(str+60, "ION ALPHA", 9)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha3 = atof(tmp);

			flags |= 0x1;
		}
		else if (strncmp(str+60, "ION BETA", 8)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta3 = atof(tmp);

			flags |= 0x1<<1;
		}
		else if (strncmp(str+60, "DELTA-UTC", 9)==0)
		{
			strncpy(tmp, str+3, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A0 = atof(tmp);

			strncpy(tmp, str+22, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A1 = atof(tmp);

			strncpy(tmp, str+41, 9);
			tmp[9] = 0;
			ionoutc->tot = atoi(tmp);

			strncpy(tmp, str+50, 9);
			tmp[9] = 0;
			ionoutc->wnt = atoi(tmp);

			if (ionoutc->tot%4096==0)
				flags |= 0x1<<2;
		}
		else if (strncmp(str+60, "LEAP SECONDS", 12)==0)
		{
			strncpy(tmp, str, 6);
			tmp[6] = 0;
			ionoutc->dtls = atoi(tmp);

			flags |= 0x1<<3;
		}
	}

	ionoutc->vflg = FALSE;
	if (flags==0xF) // Read all Iono/UTC lines
		ionoutc->vflg = TRUE;

	// Read ephemeris blocks
	g0.week = -1;
	ieph = 0;

	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		// PRN
		strncpy(tmp, str, 2);
		tmp[2] = 0;
		sv = atoi(tmp)-1;

		// EPOCH
		strncpy(tmp, str+3, 2);
		tmp[2] = 0;
		t.y = atoi(tmp) + 2000;

		strncpy(tmp, str+6, 2);
		tmp[2] = 0;
		t.m = atoi(tmp);

		strncpy(tmp, str+9, 2);
		tmp[2] = 0;
		t.d = atoi(tmp);

		strncpy(tmp, str+12, 2);
		tmp[2] = 0;
		t.hh = atoi(tmp);

		strncpy(tmp, str+15, 2);
		tmp[2] = 0;
		t.mm = atoi(tmp);

		strncpy(tmp, str+18, 4);
		tmp[2] = 0;
		t.sec = atof(tmp);

		date2gps(&t, &g);

		if (g0.week==-1)
			g0 = g;

		// Check current time of clock
		dt = subGpsTime(g, g0);

		if (dt>SECONDS_IN_HOUR)
		{
			g0 = g;
			ieph++; // a new set of ephemerides

			if (ieph>=EPHEM_ARRAY_SIZE)
				break;
		}

		// Date and time
		eph[ieph][sv].t = t;

		// SV CLK
		eph[ieph][sv].toc = g;

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19); // tmp[15]='E';
		eph[ieph][sv].af0 = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af1 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af2 = atof(tmp);

		// BROADCAST ORBIT - 1
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iode = (int)atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crs = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].deltan = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].m0 = atof(tmp);

		// BROADCAST ORBIT - 2
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cuc = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].ecc = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cus = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].sqrta = atof(tmp);

		// BROADCAST ORBIT - 3
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.sec = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cic = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omg0 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cis = atof(tmp);

		// BROADCAST ORBIT - 4
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].inc0 = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crc = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].aop = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omgdot = atof(tmp);

		// BROADCAST ORBIT - 5
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].idot = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.week = (int)atof(tmp);

		// BROADCAST ORBIT - 6
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].tgd = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iodc = (int)atof(tmp);

		// BROADCAST ORBIT - 7
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		// Set valid flag
		eph[ieph][sv].vflg = 1;

		// Update the working variables
		eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
		eph[ieph][sv].n = sqrt(GM_EARTH/(eph[ieph][sv].A*eph[ieph][sv].A*eph[ieph][sv].A)) + eph[ieph][sv].deltan;
		eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc*eph[ieph][sv].ecc);
		eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
	}

	fclose(fp);

	if (g0.week>=0)
		ieph += 1; // Number of sets of ephemerides

	return(ieph);
}

// FIXME: for beyond GPS orbit
double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel)
{
	double iono_delay = 0.0;
	double E,phi_u,lam_u,F;

	if (ionoutc->enable==FALSE)
		return (0.0); // No ionospheric delay

	E = azel[1]/PI;
	phi_u = llh[0]/PI;
	lam_u = llh[1]/PI;

	// Obliquity factor
	F = 1.0 + 16.0*pow((0.53 - E),3.0);

	if (ionoutc->vflg==FALSE)
		iono_delay = F*5.0e-9*SPEED_OF_LIGHT;
	else
	{
		double t,psi,phi_i,lam_i,phi_m,phi_m2,phi_m3;
		double AMP,PER,X,X2,X4;

		// Earth's central angle between the user position and the earth projection of
		// ionospheric intersection point (semi-circles)
		psi = 0.0137/(E + 0.11) - 0.022;

		// Geodetic latitude of the earth projection of the ionospheric intersection point
		// (semi-circles)
		phi_i = phi_u + psi*cos(azel[0]);
		if(phi_i>0.416)
			phi_i = 0.416;
		else if(phi_i<-0.416)
			phi_i = -0.416;

		// Geodetic longitude of the earth projection of the ionospheric intersection point
		// (semi-circles)
		lam_i = lam_u + psi*sin(azel[0])/cos(phi_i*PI);

		// Geomagnetic latitude of the earth projection of the ionospheric intersection
		// point (mean ionospheric height assumed 350 km) (semi-circles)
		phi_m = phi_i + 0.064*cos((lam_i - 1.617)*PI);
		phi_m2 = phi_m*phi_m;
		phi_m3 = phi_m2*phi_m;

		AMP = ionoutc->alpha0 + ionoutc->alpha1*phi_m
			+ ionoutc->alpha2*phi_m2 + ionoutc->alpha3*phi_m3;
		if (AMP<0.0)
			AMP = 0.0;

		PER = ionoutc->beta0 + ionoutc->beta1*phi_m
			+ ionoutc->beta2*phi_m2 + ionoutc->beta3*phi_m3;
		if (PER<72000.0)
			PER = 72000.0;

		// Local time (sec)
		t = SECONDS_IN_DAY/2.0*lam_i + g.sec;
		while(t>=SECONDS_IN_DAY)
			t -= SECONDS_IN_DAY;
		while(t<0)
			t += SECONDS_IN_DAY;

		// Phase (radians)
		X = 2.0*PI*(t - 50400.0)/PER;

		if(fabs(X)<1.57)
		{
			X2 = X*X;
			X4 = X2*X2;
			iono_delay = F*(5.0e-9 + AMP*(1.0 - X2/2.0 + X4/24.0))*SPEED_OF_LIGHT;
		}
		else
			iono_delay = F*5.0e-9*SPEED_OF_LIGHT;
	}

	return (iono_delay);
}

/*! \brief Compute range between a satellite and the receiver
 *  \param[out] rho The computed range
 *  \param[in] eph Const pointer of Ephemeris data of the satellite
 *  \param[in] ionoutc Const pointer of Ionosphere structure
 *  \param[in] g GPS time at time of receiving the signal
 *  \param[in] xyz position of the receiver
 *  \param[in] ant_dir receiver antenna direction
 *  \param[in] elvMask elevation mask angle [deg]
 *  \return bool whether the result is valid or not
 */
bool computeRange(range_t *rho, const ephem_t *eph, const ionoutc_t *ionoutc, const gpstime_t g, const double xyz[], const double* ant_dir, const double elvMask)
{
	double pos[3],vel[3],clk[2];
	double los[3];
	double tau;
	double range,rate;
	double xrot,yrot;

	double llh[3],neu[3], neu_ant[3], azel[2];
	double tmat[3][3];

	// SV position at time of the pseudorange observation.
	satpos(*eph, g, pos, vel, clk);

	// Receiver to satellite vector and light-time.
	subVect(los, pos, xyz);
	tau = normVect(los)/SPEED_OF_LIGHT;

	// Extrapolate the satellite position backwards to the transmission time.
	pos[0] -= vel[0]*tau;
	pos[1] -= vel[1]*tau;
	pos[2] -= vel[2]*tau;

	// Earth rotation correction. The change in velocity can be neglected.
	xrot = pos[0] + pos[1]*OMEGA_EARTH*tau;
	yrot = pos[1] - pos[0]*OMEGA_EARTH*tau;
	pos[0] = xrot;
	pos[1] = yrot;

	// Update los vector
	subVect(los, pos, xyz);

	// Azimuth and elevation angles. TODO: which los should be used?
	xyz2llh(xyz, llh);
	ltcmat(llh, tmat);
	ecef2neu(los, tmat, neu);
	neu2ant(neu, ant_dir, neu_ant);
	neu2azel(azel, neu_ant);
	// Eliminate sats under the mask angle
	if (azel[1] * R2D < elvMask) return false;

	rho->azel[0] = azel[0]; rho->azel[1] = azel[1];

	// New observer to satellite vector and satellite range.
	range = normVect(los);
	rho->d = range;

	// Pseudorange.
	rho->range = range - SPEED_OF_LIGHT*clk[0];

	// Relative velocity of SV and receiver.
	rate = dotProd(vel, los)/range;

	// Pseudorange rate.
	rho->rate = rate; // - SPEED_OF_LIGHT*clk[1];

	// Time of application.
	rho->g = g;

	// Add ionospheric delay
	rho->iono_delay = ionosphericDelay(ionoutc, g, llh, rho->azel);
	rho->range += rho->iono_delay;

	return true;
}

/*! \brief Compute the code phase for a given channel (satellite)
 *  \param[out] chan Channel on which we operate (is updated)
 *  \param[in] rho1 Current range, after \a dt has expired
 *  \param[in] dt delta-t (time difference) in seconds
 */
void computeCodePhase(channel_t *chan, range_t rho1, double dt)
{
	double ms;
	int ims;
	double rhorate;

	// Pseudorange rate.
	rhorate = (rho1.range - chan->rho0.range)/dt;

	// Carrier and code frequency.
	chan->f_carr = -rhorate/LAMBDA_L1;
	chan->f_code = CODE_FREQ + chan->f_carr*CARR_TO_CODE;

	// Initial code phase and data bit counters.
	ms = ((subGpsTime(chan->rho0.g,chan->g0)+6.0) - chan->rho0.range/SPEED_OF_LIGHT)*1000.0;

	ims = (int)ms;
	chan->code_phase = (ms-(double)ims)*CA_SEQ_LEN; // in chip

	chan->iword = ims/600; // 1 word = 30 bits = 600 ms
	ims -= chan->iword*600;

	chan->ibit = ims/20; // 1 bit = 20 code = 20 ms
	ims -= chan->ibit*20;

	chan->icode = ims; // 1 code = 1 ms

	chan->codeCA = chan->ca[(int)chan->code_phase]*2-1;
	chan->dataBit = (int)((chan->dwrd[chan->iword]>>(29-chan->ibit)) & 0x1UL)*2-1;

	// Save current pseudorange
	chan->rho0 = rho1;

	return;
}

/*! \brief Read the list of user motions from the input file
 *  \param[out] xyz Output array of ECEF vectors for user motion
 *  \param[[in] filename File name of the text input file
 *  \returns Number of user data motion records read, -1 on error
 */

//int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename)
int readUserMotion(double **xyz, const char *filename)
{
	FILE *fp;
	int numd;
	char str[MAX_CHAR];
	double t,x,y,z;

	if (NULL==(fp=fopen(filename,"rt")))
		return(-1);

	for (numd=0; numd<USER_MOTION_SIZE; numd++)
	{
		if (fgets(str, MAX_CHAR, fp)==NULL)
			break;

		if (EOF==sscanf(str, "%lf,%lf,%lf,%lf", &t, &x, &y, &z)) // Read CSV line
			break;

		xyz[numd][0] = x;
		xyz[numd][1] = y;
		xyz[numd][2] = z;
	}

	fclose(fp);

	return (numd);
}

//int readNmeaGGA(double xyz[USER_MOTION_SIZE][3], const char *filename)
int readNmeaGGA(double **xyz, const char *filename)
{
	FILE *fp;
	int numd = 0;
	char str[MAX_CHAR];
	char *token;
	double llh[3],pos[3];
	char tmp[8];

	if (NULL==(fp=fopen(filename,"rt")))
		return(-1);

	while (1)
	{
		if (fgets(str, MAX_CHAR, fp)==NULL)
			break;

		token = strtok(str, ",");

		if (strncmp(token+3, "GGA", 3)==0)
		{
			token = strtok(NULL, ","); // Date and time

			token = strtok(NULL, ","); // Latitude
			strncpy(tmp, token, 2);
			tmp[2] = 0;

			llh[0] = atof(tmp) + atof(token+2)/60.0;

			token = strtok(NULL, ","); // North or south
			if (token[0]=='S')
				llh[0] *= -1.0;

			llh[0] /= R2D; // in radian

			token = strtok(NULL, ","); // Longitude
			strncpy(tmp, token, 3);
			tmp[3] = 0;

			llh[1] = atof(tmp) + atof(token+3)/60.0;

			token = strtok(NULL, ","); // East or west
			if (token[0]=='W')
				llh[1] *= -1.0;

			llh[1] /= R2D; // in radian

			token = strtok(NULL, ","); // GPS fix
			token = strtok(NULL, ","); // Number of satellites
			token = strtok(NULL, ","); // HDOP

			token = strtok(NULL, ","); // Altitude above meas sea level

			llh[2] = atof(token);

			token = strtok(NULL, ","); // in meter

			token = strtok(NULL, ","); // Geoid height above WGS84 ellipsoid

			llh[2] += atof(token);

			// Convert geodetic position into ECEF coordinates
			llh2xyz(llh, pos);

			xyz[numd][0] = pos[0];
			xyz[numd][1] = pos[1];
			xyz[numd][2] = pos[2];

			// Update the number of track points
			numd++;

			if (numd>=USER_MOTION_SIZE)
				break;
		}
	}

	fclose(fp);

	return (numd);
}

/*! \brief Generate navigation message */
/*  \param[in] g Const receiver time in gps time
 *  \param[out] chan pointer of receiver channel
 *  \param[in] init flag indicating initialize or not
 *  \returns success (1), fail (0)
 */
int generateNavMsg(const gpstime_t g, channel_t *chan, int init)
{
	int iwrd,isbf;
	gpstime_t g0;
	unsigned long wn,tow;
	unsigned sbfwrd;
	unsigned long prevwrd;
	int nib;

	g0.week = g.week;
	g0.sec = (double)(((unsigned long)(g.sec+0.5))/30UL) * 30.0; // Align with the full frame length = 30 sec
	chan->g0 = g0; // Data bit reference time

	wn = (unsigned long)(g0.week%1024);
	tow = ((unsigned long)g0.sec)/6UL;

	// Initialize the subframe 5
	if (init==1)
	{
		prevwrd = 0UL;

		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
		{
			sbfwrd = chan->sbf[4 + chan->ipage * 2][iwrd];

			// Add TOW-count message into HOW
			if (iwrd==1)
				sbfwrd |= ((tow&0x1FFFFUL)<<13);

			// Compute checksum
			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
			chan->dwrd[iwrd] = computeChecksum(sbfwrd, nib);

			prevwrd = chan->dwrd[iwrd];
		}
	}
	else
	{
		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
		{
			chan->dwrd[iwrd] = chan->dwrd[N_DWRD_SBF*N_SBF+iwrd];

			prevwrd = chan->dwrd[iwrd];
		}
		/*
		// Sanity check
		if (((chan->dwrd[1])&(0x1FFFFUL<<13)) != ((tow&0x1FFFFUL)<<13))
		{
			printf("\nWARNING: Invalid TOW in subframe 5.\n");
			return(0);
		}
		*/
	}

	// Generate subframe words
	for (isbf=0; isbf<N_SBF; isbf++)
	{
		tow++;

		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
		{
			if (isbf < 3) // Subframes 1-3
				sbfwrd = chan->sbf[isbf][iwrd];
			else if (isbf == 3) // Subframe 4
				sbfwrd = chan->sbf[3 + chan->ipage * 2][iwrd];
			else // Subframe 5
				sbfwrd = chan->sbf[4 + chan->ipage * 2][iwrd];

			// Add transmission week number to Subframe 1
			if ((isbf==0)&&(iwrd==2))
				sbfwrd |= (wn&0x3FFUL)<<20;

			// Add TOW-count message into HOW
			if (iwrd==1)
				sbfwrd |= ((tow&0x1FFFFUL)<<13);

			// Compute checksum
			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
			chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd] = computeChecksum(sbfwrd, nib);

			prevwrd = chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd];
		}
	}

	// Move to the next pages
	chan->ipage++;
	if (chan->ipage >= 25)
		chan->ipage = 0;

	return(1);
}

/*! \brief Check satellite visibility */
/*  \param[in] eph Const pointer of the ephemeris structure
 *  \param[in] g Const pointer of Receiver time in gps time
 *  \param[in] xyz Array of receiver position
 *  \param[out] chan Receiver channel, compute tx_antenna_gain, and azel
 *  \returns visible (1), invisible (0), or invalid (-1)
 */
int checkSatVisibility(const ephem_t *eph, const gpstime_t *g, const double *xyz, channel_t *chan, double lunar_pos_ecef[3])
{
	// FIXME: for ECI.
	double llh[3],neu[3];
	double pos[3],vel[3],clk[3],los[3];
	double tmat[3][3];
	double el_earth_edge;

    if (eph->vflg != 1)
    {
        return (-1); // Invalid
    }

	xyz2llh(xyz,llh);
	if (llh[2] > 0)
	{
		el_earth_edge = -(0.5 * PI - asin(RADIUS_EARTH /(RADIUS_EARTH + llh[2])));
	}
	else
	{
		// FIXME: for negative altitude?
		el_earth_edge = 0;
	}
	ltcmat(llh, tmat);

    satpos(*eph, *g, pos, vel, clk);
    subVect(los, pos, xyz);
    // Check satellite direction
    double dot_prod_pos_los = dotProd(pos, los);
    if (dot_prod_pos_los < 0)
        {
            // Invisible. Elevation of receiver w.r.t satellite is negative
            return 0;
        }

    // TODO: think better way
    if (llh[2] > 3e8)
        {
            // Check if the sc is behind of moon
            double radius_moon_m = GetRadiusKm(moon_) * 1000;
            double los_from_moon_to_gps[3];
            subVect(los_from_moon_to_gps, pos, lunar_pos_ecef);
            double distance_sc2gps = normVect(los);
            double distance_moon2gps = normVect(los_from_moon_to_gps);
            double zenith_lunar_edge_rad = asin(radius_moon_m / distance_moon2gps);
            // Check elevation only when the distance is bigger than distance to edge
            if (distance_sc2gps > distance_moon2gps * cos(zenith_lunar_edge_rad))
                {
                    // NOTE: should be around 0.004453 rad.
                    double zenith_sc_rad = acos(dotProd(los_from_moon_to_gps, los) / (distance_moon2gps * distance_sc2gps));
                    // Behind the moon
                    if (zenith_sc_rad < zenith_lunar_edge_rad)
                        {
                            // For debug
                            printf("SC is behind the moon!\n");
                            return 0;
                        }
                }
        }

    // NOTE: assuming the GPS satellite attitude is Geocentric
    double ele_receiver_from_sat_rad = 0.5 * PI - acos(dot_prod_pos_los / (normVect(pos) * normVect(los)));
    // Ensure the elevation is 0 < ele < 90 deg by checking the sign of dot product above.
    GetAntennaGain(&chan->gps_sat, (uint8_t)(ele_receiver_from_sat_rad * R2D), &chan->tx_antenna_gain);

    ecef2neu(los, tmat, neu);
    neu2azel(chan->azel, neu);

    // if (azel[1]*R2D > elvMask)
    if (chan->azel[1] > el_earth_edge)  // Check only geometric visibility
        return 1;                 // Visible

    return 0;  // Invisible
}

/*! \brief Allocate visible satellites into each ch. */
/*  \param[out] chan Array of receiver channel
 *  \param[out] allocatedSat Array of ch information accodring to the satellite
 *  \param[in] eph Array of the ephemeris structure
 *  \param[in] env Const pointer of environment structure
 *  \param[in] xyz Array of receiver position
 *  \param[in] ant_dir Array of receiver antenna direction (azi, ele)
 *  \param[in] elvMask elevation mask angle [deg]
 *  \returns Number of GNSS satellites
 */
int allocateChannel(channel_t *chan, int *allocatedSat, const ephem_t* eph, const env_t *env, const double *xyz, const double* ant_dir, double elvMask, FILE* log_file)
{
	int nsat=0;
	int i,sv;

	range_t rho = {0};
	double ref[3]={0.0};
	double r_ref,r_xyz;
	double phase_ini;
	const gpstime_t grx = *(env->g);

	int ipage = 0; // Initial page count

	// Find current page number
	for (i = 0; i < MAX_CHAN; i++)
	{
		if (chan[i].gps_sat.PRN != 0)
		{
			ipage = chan[i].ipage;
			break;
		}
	}

	if (log_file != NULL)
		fprintf(log_file, "%lf", grx.sec);

	// Preparation for moon occultation check
	double tt = ConvGPSTimeToTt(time_system, env->g->week, env->g->sec);
	const double* lunar_pos_i = GetPositionI(moon_, tt);
	double dcm_eci_to_ecef[9];
	GetDcmEciToEcef(frame, tt, dcm_eci_to_ecef);
	double lunar_pos_ecef[3];
	for (i = 0; i < 3; i++)
		{
			lunar_pos_ecef[i] = dcm_eci_to_ecef[3 * i] * lunar_pos_i[0]
								+ dcm_eci_to_ecef[3 * i + 1] * lunar_pos_i[1]
								+ dcm_eci_to_ecef[3 * i + 2] * lunar_pos_i[2];
		}
	// For debug
	// static FILE* fpt = NULL;
	// if (fpt == NULL)
	// 	fpt = fopen("moon_eci.csv", "w+");
	// 	// fpt = fopen("los_moon2sat.csv", "w+");
	// // double los_moon2sat[3];
	// // subVect(los_moon2sat, xyz, lunar_pos_ecef);
	// // double distance_from_moon = normVect(los_moon2sat);
	// // fprintf(fpt, "%lf, %lf, %lf, %lf\n", los_moon2sat[0], los_moon2sat[1], los_moon2sat[2], distance_from_moon);
	// fprintf(fpt, "%lf, %lf, %lf, %lf, %lf\n", tt, lunar_pos_i[0], lunar_pos_i[1], lunar_pos_i[2], normVect(lunar_pos_i));
	// fflush(fpt);

	// Allocate channel
	for (sv=0; sv<MAX_SAT; sv++)
	{
		channel_t channel = {};
		channel.gps_sat = gps_sats[sv];
		if(checkSatVisibility(&eph[sv], env->g, xyz, &channel, lunar_pos_ecef) == 1)
		{
			if (log_file != NULL)
				fprintf(log_file, ",%d", 1);
			nsat++; // Number of visible satellites

			if (allocatedSat[sv]==-1) // Visible but not allocated
			{
				// Allocate new satellite
				bool allocated = false;
				for (i=0; i<MAX_CHAN; i++)
				{
					if (chan[i].gps_sat.PRN == 0)
					{
						if (!computeRange(&rho, &eph[sv], &(env->ionoutc), grx, xyz, ant_dir, elvMask))
							break;

						// Initialize gps_sat, tx_antenna_gain, azel
						chan[i] = channel;

						// Initialize pseudorange
						chan[i].rho0 = rho;
						// Initialize carrier phase
						r_xyz = rho.range;

						chan[i].ipage = ipage;

						// C/A code generation
						codegen(chan[i].ca, chan[i].gps_sat.PRN);

						// Generate subframe
						eph2sbf(eph[sv], env->ionoutc, env->alm, chan[i].sbf);

						// Generate navigation message
						generateNavMsg(grx, &chan[i], 1);

						computeRange(&rho, &eph[sv], &(env->ionoutc), grx, ref, ant_dir, elvMask);
						r_ref = rho.range;

						phase_ini = (2.0*r_ref - r_xyz)/LAMBDA_L1;
						phase_ini -= floor(phase_ini);
						chan[i].carr_phase = (unsigned int)(512 * 65536.0 * phase_ini);

						// Done.
						allocated = true;
						break;
					}
				}

				// Set satellite allocation channel
				if (allocated)
					allocatedSat[sv] = i;
			}
		}
		else
		{
			if (allocatedSat[sv]>=0) // Not visible but allocated
			{
				// Clear channel
				channel_t ch_clear = {};
				chan[allocatedSat[sv]] = ch_clear;

				// Clear satellite allocation flag
				allocatedSat[sv] = -1;
			}
			if (log_file != NULL)
				fprintf(log_file, ",%d", 0);
		}
	}
	if (log_file != NULL)
		fprintf(log_file, "\n");

	return(nsat); // isn't used now.
}

/*! \brief Set receiver position array "xyz" and return the size of simulation "numd". */
/*  \param[out] xyz Array of user motion in ECEF frame
 *  \param[in] opt Pointer of the option_t
 *  \returns Number of simulation length
 */
int setReceiverPosition(double** xyz, const option_t* opt)
{
    int numd;

    for (int i = 0; i < USER_MOTION_SIZE; i++)
    {
        xyz[i] = (double *)malloc(3 * sizeof(double));
        if (xyz[i] == NULL)
        {
            for (int j = i - 1; j >= 0; j--)
                free(xyz[i]);

            printf("ERROR: Failed to allocate user motion array.\n");
            return -1;
        }
    }

    if (!opt->staticLocationMode)
    {
        // Read user motion file
        if (opt->nmeaGGA == true)
            numd = readNmeaGGA(xyz, opt->umfile);
        else
        {
            numd = readUserMotion(xyz, opt->umfile);
        }

        if (numd <= 0)
        {
            printf("ERROR: Failed to open user motion / NMEA GGA file.\n");
            return -1;
        }

        // Set simulation duration
        if (numd > opt->iduration)
            numd = opt->iduration;
    }
    else
    {
        // Static geodetic coordinates input mode: "-l"
        // Added by scateu@gmail.com
        printf("Using static location mode.\n");
        llh2xyz(opt->llh, xyz[0]); // Convert llh to xyz

        numd = opt->iduration;

        for (int iumd = 1; iumd < numd; iumd++)
        {
            xyz[iumd][0] = xyz[0][0];
            xyz[iumd][1] = xyz[0][1];
            xyz[iumd][2] = xyz[0][2];
        }
    }
    // TODO: better to resize xyz array according to the iduration

    printf("xyz = %11.1f, %11.1f, %11.1f\n", xyz[0][0], xyz[0][1], xyz[0][2]);
    printf("llh = %11.6f, %11.6f, %11.1f\n", opt->llh[0]*R2D, opt->llh[1]*R2D, opt->llh[2]);

    return numd;
}

/*! \brief print the debug message about channel */
/*  \param[in] chan Array of receiver channels
 *  \param[in] xyz Array of user position
 */
void printChannelInformation(const channel_t *chan, const double *xyz)
{
    double llh[3];
    printf("xyz = %11.1f, %11.1f, %11.1f\n", xyz[0], xyz[1], xyz[2]);
    xyz2llh(xyz, llh);
    printf("llh = %11.6f, %11.6f, %11.1f\n", llh[0]*R2D, llh[1]*R2D, llh[2]);
    printf("PRN    Azi   Ele PseudoRange   Ion TxGain\n");
    int i;
    for (i = 0; i < MAX_CHAN; i++)
    {
        if (chan[i].gps_sat.PRN > 0)
            printf(" %02d %6.1f %5.1f %11.1f %5.1f %6d\n", chan[i].gps_sat.PRN,
                chan[i].azel[0] * R2D, chan[i].azel[1] * R2D, chan[i].rho0.d, chan[i].rho0.iono_delay, chan[i].tx_antenna_gain);
    }
}

/*! \brief Initialize receiver channel and allocatedSat */
/*  \param[out] chan Array of receiver channel
 *  \param[out] allocatedSat Array of channel allocation information
 *  \param[in] eph Const pointer to the corresponding ephemeris array
 *  \param[in] env Const pointer to the env_t struct
 *  \param[in] xyz Array of receiver position
 *  \param[in] ant_dir Array of receiver antenna direction (azi, ele)
 *  \param[in] elvMask elevation mask angle [deg]
 */
void initializeChannel(channel_t* chan, int* allocatedSat, const ephem_t *eph, const env_t *env, const double *xyz, const double* ant_dir, const double elvmask, FILE* log_file)
{
    // Clear all channels
    int i;
    for (i = 0; i < MAX_CHAN; i++)
    {
        chan[i].gps_sat.PRN = 0;
    }

	// Clear satellite allocation flag
	for (int sv = 0; sv < MAX_SAT; sv++)
		allocatedSat[sv] = -1;

	// Allocate visible satellites
	allocateChannel(chan, allocatedSat, eph, env, xyz, ant_dir, elvmask, log_file);

    printChannelInformation(chan, xyz);
}

/*! \brief Compute receiver observation and gain */
/*  \param[out] chan Pointer of receiver channel
 *  \param[in] eph Array of the corresponding ephemeris
 *  \param[in] env Const pointer to the env_t struct
 *  \param[in] xyz Array of receiver position
 *  \param[in] opt Pointer of option
 *  \param[in] ant_pat Array of receiver antenna pattern
 *  \param[in] elvMask elevation mask angle [deg]
 *  \returns valid(true) or not(false)
 */
bool computeObservation(channel_t* chan, const ephem_t* eph, const env_t* env, const double* xyz, const option_t* opt, const double *ant_pat, const double elvmask)
{
    double delt = 1.0 / (double)tx_samplerate;
    if (chan->gps_sat.PRN > 0)
    {
        // Refresh code phase and data bit counters
        range_t rho;
        int sv = chan->gps_sat.PRN - 1;

        // Compute current pseudorange
        if (!computeRange(&rho, &eph[sv], &(env->ionoutc), *(env->g), xyz, opt->rec_ant_dir, elvmask))
        {
            channel_t ch_clear = {};
            chan = &ch_clear;
            // allocatedSat[sv] = -1;
            return false;
        }
        chan->azel[0] = rho.azel[0];
        chan->azel[1] = rho.azel[1];

        // Update code phase and data bit counters
        computeCodePhase(chan, rho, 0.1);
        chan->carr_phasestep = (int)(512 * 65536.0 * chan->f_carr * delt);

        // Path loss
        double path_loss_for_sim = opt->path_loss_enable ? 20200000.0/rho.d : 1.0;
        double path_loss_db = 20 * log10(4 * PI * rho.d * TX_FREQUENCY / SPEED_OF_LIGHT);

        // Receiver antenna gain
        int ibs = (int)((90.0-rho.azel[1]*R2D)/5.0); // covert elevation to boresight
        double rec_ant_gain = opt->antenna_pattern_enable ? ant_pat[ibs] : 1.0;

        // GPS Tx antenna gain
        const int8_t boresight_tx_gain_db = chan->gps_sat.antenna_gain[0]; // TODO: Id 2's maximum is 16dB
        // Normalize it to avoid the over range
        const double normalized_tx_gain = opt->antenna_pattern_enable ? pow(10.0, (chan->tx_antenna_gain - boresight_tx_gain_db) / 10.0) : 1.0;
        if (!opt->antenna_pattern_enable)
        {
            static const uint8_t required_CN0 = 18;
            static const double boltzmann_const = 228.6;
            static const double rx_GT = -20.8;  // LNA gain 40dB, NF 1.5dB
            static const double boresight_EIRP_and_other_gain = 35 + 3;
            double rx_CN0 = boresight_EIRP_and_other_gain + (chan->tx_antenna_gain - boresight_tx_gain_db) - path_loss_db + rx_GT + boltzmann_const;
            if (rx_CN0 < required_CN0)
            {
                chan->gain = 0;
                return false;
            }
        }

        // Signal gain
        chan->gain = (int)(path_loss_for_sim * rec_ant_gain * normalized_tx_gain * 128.0); // scaled by 2^7
        return true;
    }

    return false;
}

/*! \brief Compute accumulated value of iq sampling */
/*  \param[out] iq_acc Array of iq accumulation
 *  \param[out] chan Array of receiver channels
 */
void computeIQacc(int *iq_acc, channel_t * chan)
{
    double delt = 1.0 / (double)tx_samplerate;
    int i;
    for (i = 0; i < MAX_CHAN; i++)
    {
        if (chan[i].gps_sat.PRN > 0)
        {
            int iTable = (chan[i].carr_phase >> 16) & 511;

            int ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * chan[i].gain;
            int qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * chan[i].gain;

            // Accumulate for all visible satellites
            iq_acc[0] += ip;
            iq_acc[1] += qp;

            // Update code phase
            chan[i].code_phase += chan[i].f_code * delt;

            if (chan[i].code_phase >= CA_SEQ_LEN)
            {
                chan[i].code_phase -= CA_SEQ_LEN;

                chan[i].icode++;

                if (chan[i].icode >= 20)  // 20 C/A codes = 1 navigation data bit
                {
                    chan[i].icode = 0;
                    chan[i].ibit++;

                    if (chan[i].ibit >= 30)  // 30 navigation data bits = 1 word
                    {
                        chan[i].ibit = 0;
                        chan[i].iword++;
                        /*
                        if (chan[i].iword>=N_DWRD)
                                printf("\nWARNING: Subframe word buffer overflow.\n");
                        */
                    }

                    // Set new navigation data bit
                    chan[i].dataBit = (int)((chan[i].dwrd[chan[i].iword] >> (29 - chan[i].ibit)) & 0x1UL) * 2 - 1;
                }
            }

            // Set currnt code chip
            chan[i].codeCA = chan[i].ca[(int)chan[i].code_phase] * 2 - 1;

            // Update carrier phase
            chan[i].carr_phase += chan[i].carr_phasestep;
        }
    }

    // Scaled by 2^7
    iq_acc[0] = (iq_acc[0] + 64) >> 7;
    iq_acc[1] = (iq_acc[1] + 64) >> 7;
}

#ifndef _WIN32
void changemode(int dir)
{
    static struct termios oldt, newt;

    if ( dir == 1 )
    {
        tcgetattr( STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~( ICANON | ECHO ); // non-canonical mode & no echo
        tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    }
    else
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

int _kbhit (void)
{
    struct timeval tv;
    fd_set rdfs;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);

    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);

}

int _getch()
{
	int ch;

	ch = getchar();
#ifdef __APPLE__
	fpurge(stdin); // Clear STDIN buffer
#else
	__fpurge(stdin); // Clear STDIN buffer
#endif
	return ch;
}
#endif

void *gps_task(void *arg)
{
    // NOTE: this didn't work.
    // struct sched_param param;
    // // param.sched_priority = sched_get_priority_max(SCHED_FIFO);  // Max priority
    // param.sched_priority = 20;

    // // Set real time scheduling using SCHED_FIFO
    // if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
	// {
	// 	perror("sched_setscheduler");
	// 	exit(EXIT_FAILURE);
	// }

    sim_t *s = (sim_t *)arg;

    int sv;
    int neph, ieph;
    env_t env;
	gpstime_t g0;

	int i;
	channel_t chan[MAX_CHAN], chan2[MAX_CHAN];
    int allocatedSat[MAX_SAT], allocatedSat2[MAX_SAT];
	double elvmask = 0.0; // in degree TODO: make it be able to specify from option

	short *iq_buff = NULL;

	gpstime_t grx;
	int isamp;

	int iumd;
	int numd;
	char umfile[MAX_CHAR];
	// double xyz[USER_MOTION_SIZE][3], xyz2[USER_MOTION_SIZE][3];
	double **xyz, **xyz2 = NULL;

	char navfile[MAX_CHAR];

	int iq_buff_size = num_iq_samples;
    if (s->ch2_enable)
    {
        iq_buff_size = 2 * num_iq_samples;
    }

	double ant_pat[37];

	datetime_t t0,tmin,tmax;
	gpstime_t gmin,gmax;
	double dt;
	int igrx;

	int iduration;

	int timeoverwrite = FALSE; // Overwirte the TOC and TOE in the RINEX file

	char almfile[MAX_CHAR];
	int nalm;

	int interactive = FALSE;
	int key;
	int key_direction;
	int direction = UNDEF;
	double velocity = 0.0;
	double tmat[3][3];
	double neu[3];

	// Log file of user motion
	const size_t log_dir_str_size = strlen(s->opt.log_dir);
	FILE *log_files[2] = {NULL, NULL};
	FILE *visibility_log_files[2] = {NULL, NULL};
	bool dump_user_pos[2] = {false, false};
	if (log_dir_str_size != 0)
	{
		// TODO: visibility log file
        for (uint8_t i = 0; i < 2; i++)
        {
            if (!s->ch2_enable && i == 1) continue;

            char log_file_name[log_dir_str_size + sizeof("YYYYMMDDhhmmss_ch1.txt")];
            char visibility_file_name[log_dir_str_size + sizeof("YYYYMMDDhhmmss_visibility_ch1.txt")];
            time_t rawtime;
            time(&rawtime);
            strcpy(log_file_name, s->opt.log_dir);
            strftime(&log_file_name[log_dir_str_size], sizeof(log_file_name), "%Y%m%d%H%M%S", localtime(&rawtime));
            strcpy(visibility_file_name, log_file_name);
            const char *suffix = (i == 0) ? "_ch1.txt" : "_ch2.txt";
            strcpy(&log_file_name[log_dir_str_size + 14], suffix);
            const char *suffix2 = (i == 0) ? "_visibility_ch1.txt" : "_visibility_ch2.txt";
            strcpy(&visibility_file_name[log_dir_str_size + 14], suffix2);
            // for debug
            printf("visibility file path: %s\n", visibility_file_name);

            visibility_log_files[i] = fopen(visibility_file_name, "w");
            if (visibility_log_files[i] == NULL)
            {
                perror("Unable to open file");
                goto exit;
            }
            // Write the header
            fprintf(visibility_log_files[i], "t");
            for (uint32_t sv = 0; sv < MAX_SAT; sv++)
			{
				// PRN
				fprintf(visibility_log_files[i], ", %d", sv + 1);
			}
            fprintf(visibility_log_files[i], "\n");

            if ((i == 0 && s->opt.staticLocationMode) ||
                (i == 1 && s->opt2.staticLocationMode)) continue;
            dump_user_pos[i] = true;
            printf("log file path: %s\n", log_file_name);
            log_files[i] = fopen(log_file_name, "w");
            if (log_files[i] == NULL)
            {
                perror("Unable to open file");
                goto exit;
            }
            // Write the header
            fprintf(log_files[i], "t, x, y, z\n");
        }
	}

	////////////////////////////////////////////////////////////
	// Read options
	////////////////////////////////////////////////////////////

	// common options
	strcpy(navfile, s->opt.navfile);
	strcpy(almfile, s->opt.almfile);
	iduration = s->opt.iduration;
	interactive = s->opt.interactive;
	timeoverwrite = s->opt.timeoverwrite;
	env.ionoutc.enable = s->opt.iono_enable;
	g0.week = s->opt.g0.week;
	g0.sec = s->opt.g0.sec;
    env.g = &g0;
	gps2date(&g0, &t0);

	gmin = g0;
	gmax = g0;

	// independent options b/w ch
	strcpy(umfile, s->opt.umfile);

	// Initialize global variables for environment
	// FIXME: better to put them into env_t?
	time_system = TimeSystemInit();
	earth_ = EarthInit(ConvGPSTimeToTt(time_system, g0.week, g0.sec));
	moon_ = MoonInit(g0.week, g0.sec, EarthGravityConst(earth_));
	frame = FrameInit(earth_, moon_, time_system);
    for (sv = 0; sv < MAX_SAT; sv++)
        InitGPSSatellite(&gps_sats[sv], sv + 1);

	////////////////////////////////////////////////////////////
	// Receiver position
	////////////////////////////////////////////////////////////

    // Allocate user motion array
    xyz = (double **)malloc(USER_MOTION_SIZE * sizeof(double **));
    if (xyz == NULL)
    {
        printf("ERROR: Failed to allocate user motion array.\n");
        goto exit;
    }
    numd = setReceiverPosition(xyz, &(s->opt));
    if (numd < 0)
    {
        goto exit;
    }

    if (s->ch2_enable)
    {
        xyz2 = (double **)malloc(USER_MOTION_SIZE * sizeof(double **));
        if (xyz2 == NULL)
        {
            printf("ERROR: Failed to allocate user motion array.\n");
            goto exit;
        }
        int numd2 = setReceiverPosition(xyz2, &(s->opt2));
        if (numd2 < 0)
        {
            goto exit;
        }
        if (numd2 < numd)
        {
            numd = numd2;
        }
    }

	// Initialize the local tangential matrix for interactive mode
	ltcmat(s->opt.llh, tmat);
	if (interactive)
	{
		printf("Enable interactive mode.\n");
		numd = iduration;
	}

	////////////////////////////////////////////////////////////
	// Read ephemeris
	////////////////////////////////////////////////////////////

	neph = readRinexNavAll(env.eph, &(env.ionoutc), navfile);

	if (neph==0)
	{
		printf("ERROR: No ephemeris available.\n");
		goto exit;
	}

    ionoutc_t ionoutc = env.ionoutc;
    ephem_t (*eph)[MAX_SAT] = env.eph;
    if (ionoutc.vflg == TRUE)
        {
		printf("  %12.3e %12.3e %12.3e %12.3e\n",
			ionoutc.alpha0, ionoutc.alpha1, ionoutc.alpha2, ionoutc.alpha3);
		printf("  %12.3e %12.3e %12.3e %12.3e\n",
			ionoutc.beta0, ionoutc.beta1, ionoutc.beta2, ionoutc.beta3);
		printf("   %19.11e %19.11e  %9d %9d\n",
			ionoutc.A0, ionoutc.A1, ionoutc.tot, ionoutc.wnt);
		printf("%6d\n", ionoutc.dtls);
	}

	////////////////////////////////////////////////////////////
	// Set simulation start time
	////////////////////////////////////////////////////////////

	for (sv=0; sv<MAX_SAT; sv++)
	{
		if (eph[0][sv].vflg==1)
		{
			gmin = eph[0][sv].toc;
			tmin = eph[0][sv].t;
			break;
		}
	}

	for (sv=0; sv<MAX_SAT; sv++)
	{
		if (eph[neph-1][sv].vflg == 1)
		{
			gmax = eph[neph-1][sv].toc;
			tmax = eph[neph-1][sv].t;
			break;
		}
	}

	if (g0.week>=0) // Scenario start time has been set.
	{
		if (timeoverwrite==TRUE)
		{
			gpstime_t gtmp;
			datetime_t ttmp;
			double dsec;

			gtmp.week = g0.week;
			gtmp.sec = (double)(((int)(g0.sec))/7200)*7200.0;

			dsec = subGpsTime(gtmp,gmin);

			// Overwrite the UTC reference week number
			env.ionoutc.wnt = ionoutc.wnt = gtmp.week;
			env.ionoutc.tot = ionoutc.tot = (int)gtmp.sec;

			// Overwrite the TOC and TOE to the scenario start time
			for (sv=0; sv<MAX_SAT; sv++)
			{
				for (i=0; i<neph; i++)
				{
					if (eph[i][sv].vflg == 1)
					{
						gtmp = incGpsTime(eph[i][sv].toc, dsec);
						gps2date(&gtmp,&ttmp);
						env.eph[i][sv].toc = gtmp;
						env.eph[i][sv].t = ttmp;

						gtmp = incGpsTime(eph[i][sv].toe, dsec);
						env.eph[i][sv].toe = gtmp;
					}
				}
			}
		}
		else
		{
			if (subGpsTime(g0, gmin)<0.0 || subGpsTime(gmax, g0)<0.0)
			{
				printf("ERROR: Invalid start time.\n");
				printf("tmin = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
					tmin.y, tmin.m, tmin.d, tmin.hh, tmin.mm, tmin.sec,
					gmin.week, gmin.sec);
				printf("tmax = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
					tmax.y, tmax.m, tmax.d, tmax.hh, tmax.mm, tmax.sec,
					gmax.week, gmax.sec);

				goto exit;
			}
		}
	}
	else
	{
		g0 = gmin;
		t0 = tmin;
	}

	printf("Start time = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
		t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, g0.week, g0.sec);
	printf("Duration = %.1f [sec]\n", ((double)numd)/10.0);

	// Select the current set of ephemerides
	ieph = -1;

	for (i=0; i<neph; i++)
	{
		for (sv=0; sv<MAX_SAT; sv++)
		{
			if (eph[i][sv].vflg == 1)
			{
				dt = subGpsTime(g0, eph[i][sv].toc);
				if (dt>=-SECONDS_IN_HOUR && dt<SECONDS_IN_HOUR)
				{
					ieph = i;
					break;
				}
			}
		}

		if (ieph>=0) // ieph has been set
			break;
	}

	if (ieph == -1)
	{
		printf("ERROR: No current set of ephemerides has been found.\n");
		goto exit;
	}

	////////////////////////////////////////////////////////////
	// Read almanac
	////////////////////////////////////////////////////////////

	nalm = readAlmanac(env.alm, almfile);

	if (almfile[0] != 0 && nalm == -1)
	{
		printf("ERROR: Invalid almanac file name.\n");
		goto exit;
	}

	if (nalm > 0)
	{
		if (timeoverwrite == TRUE)
		{
			printf("ERROR: Time overwrite cannot be used with almanac option.\n");
			goto exit;
		}

		// Check TOA
		for (sv = 0; sv < MAX_SAT; sv++)
		{
			if (env.alm[sv].id != 0) // Valid almanac
			{
				dt = subGpsTime(env.alm[sv].toa, g0);

				if (dt<(-4.0*SECONDS_IN_WEEK) || dt>(4.0*SECONDS_IN_WEEK))
				{
					printf("ERROR: Invalid time of almanac.\n");
					goto exit;
				}
			}

		}
	}

	////////////////////////////////////////////////////////////
	// Baseband signal buffer and output file
	////////////////////////////////////////////////////////////

	// Allocate I/Q buffer
	iq_buff = calloc(2 * iq_buff_size, sizeof(short));

	if (iq_buff==NULL)
	{
		printf("ERROR: Failed to allocate 16-bit I/Q buffer.\n");
		goto exit;
	}

	////////////////////////////////////////////////////////////
	// Initialize channels
	////////////////////////////////////////////////////////////

	// Initial reception time
	grx = incGpsTime(g0, 0.0);

    initializeChannel(chan, allocatedSat, eph[ieph], &env, xyz[0], s->opt.rec_ant_dir, elvmask, visibility_log_files[0]);
    if (s->ch2_enable)
    {
        initializeChannel(chan2, allocatedSat2, eph[ieph], &env, xyz2[0], s->opt2.rec_ant_dir, elvmask, visibility_log_files[1]);
    }

	////////////////////////////////////////////////////////////
	// Receiver antenna gain pattern
	////////////////////////////////////////////////////////////

    // TODO: enhance for ch2
	for (i=0; i<37; i++)
		ant_pat[i] = pow(10.0, -ant_pat_db[i]/20.0); // FIXME: should be 10log.

	////////////////////////////////////////////////////////////
	// Generate baseband signals
	////////////////////////////////////////////////////////////
#ifndef _WIN32
	changemode(1); // non-canonical mode & no echo
#endif

	// Update receiver time
	grx = incGpsTime(grx, 0.1);
    env.g = &grx;

	for (iumd=1; iumd<numd; iumd++)
	{
		key = 0; // Initialize key input

		// Press 'q' to abort
		if (_kbhit())
		{
			key = _getch();

			if (key == 'q' || key == 'Q')
				goto abort;
		}

		// Interactive mode NOTE: this is only for ch1
		if (interactive)
		{
			key_direction = UNDEF;

			switch (key)
			{
			case NORTH_KEY:
				key_direction = NORTH;
				break;
			case SOUTH_KEY:
				key_direction = SOUTH;
				break;
			case EAST_KEY:
				key_direction = EAST;
				break;
			case WEST_KEY:
				key_direction = WEST;
				break;
			default:
				break;
			}

			if ((key_direction!=UNDEF)&&(direction==key_direction))
			{
				// Accelerate toward the key direction
				if (velocity<MAX_VEL)
					velocity += DEL_VEL;
			}
			else
			{
				// Deaccelerate and stop
				if (velocity>=0.0)
					velocity -= DEL_VEL;
				else
					direction = key_direction; // then change the direction
			}

			// Stay at the current location
			xyz[iumd][0] = xyz[iumd-1][0];
			xyz[iumd][1] = xyz[iumd-1][1];
			xyz[iumd][2] = xyz[iumd-1][2];

			if ((direction!=UNDEF)&&(velocity>=0.0))
			{
				// Update the user location
				neu[0] = 0.0;
				neu[1] = 0.0;
				neu[2] = 0.0;

				switch(direction)
				{
				case NORTH:
					neu[0] = velocity * 0.1;
					break;
				case SOUTH:
					neu[0] = -velocity * 0.1;
					break;
				case EAST:
					neu[1] = velocity * 0.1;
					break;
				case WEST:
					neu[1] = -velocity * 0.1;
				default:
					break;
				}

				xyz[iumd][0] += tmat[0][0]*neu[0] + tmat[1][0]*neu[1] + tmat[2][0]*neu[2];
				xyz[iumd][1] += tmat[0][1]*neu[0] + tmat[1][1]*neu[1] + tmat[2][1]*neu[2];
				xyz[iumd][2] += tmat[0][2]*neu[0] + tmat[1][2]*neu[1] + tmat[2][2]*neu[2];
			}
		}

		// Logging receiver position with gpstime.
		if (dump_user_pos[0])
			fprintf(log_files[0], "%lf,%lf,%lf,%lf\n", grx.sec, xyz[iumd][0], xyz[iumd][1], xyz[iumd][2]);
		if (dump_user_pos[1])
			fprintf(log_files[1], "%lf,%lf,%lf,%lf\n", grx.sec, xyz2[iumd][0], xyz2[iumd][1], xyz2[iumd][2]);

		for (i=0; i<MAX_CHAN; i++)
		{
            computeObservation(&chan[i], eph[ieph], &env, xyz[iumd], &(s->opt), ant_pat, elvmask);

            if (s->ch2_enable)
            {
                computeObservation(&chan2[i], eph[ieph], &env, xyz2[iumd], &(s->opt2), ant_pat, elvmask);
            }
		}

		for (isamp = 0; isamp < num_iq_samples; isamp++)
		{
            int pos_buffer = 2 * isamp;
            int iq_acc[2] = {0, 0};
            int iq_acc2[2] = {0, 0};
            computeIQacc(iq_acc, chan);
            if (s->ch2_enable)
            {
                pos_buffer = 4 * isamp;
                computeIQacc(iq_acc2, chan2);
            }

			// Store I/Q samples into buffer
			iq_buff[pos_buffer] = (short)iq_acc[0];
			iq_buff[pos_buffer + 1] = (short)iq_acc[1];
            if (s->ch2_enable)
            {
                iq_buff[pos_buffer + 2] = (short)iq_acc2[0];
                iq_buff[pos_buffer + 3] = (short)iq_acc2[1];
            }
		}

		////////////////////////////////////////////////////////////
		// Write into FIFO
		///////////////////////////////////////////////////////////

		if (!s->gps.ready) {
			// Initialization has been done. Ready to create TX task.
			printf("GPS signal generator is ready!\n");
			s->gps.ready = 1;
			pthread_cond_signal(&(s->gps.initialization_done));
		}

		// Wait until FIFO write is ready
		pthread_mutex_lock(&(s->gps.lock));
		while (!is_fifo_write_ready(s))
			pthread_cond_wait(&(s->fifo_write_ready), &(s->gps.lock));
		pthread_mutex_unlock(&(s->gps.lock));

		// Write into FIFO
		memcpy(&(s->fifo[s->head * 2]), iq_buff, iq_buff_size * 2 * sizeof(short));

		s->head += (long)iq_buff_size;
		if (s->head >= fifo_length)
			s->head -= fifo_length;
		pthread_cond_signal(&(s->fifo_read_ready));

		//
		// Update navigation message and channel allocation every 30 seconds
		//

		igrx = (int)(grx.sec*10.0+0.5);

		if (igrx%300==0) // Every 30 seconds
		{
			// Update navigation message
			for (i=0; i<MAX_CHAN; i++)
			{
                if (chan[i].gps_sat.PRN > 0)
                    generateNavMsg(grx, &chan[i], 0);

                if (s->ch2_enable)
                {
                    if (chan2[i].gps_sat.PRN > 0)
                        generateNavMsg(grx, &chan2[i], 0);
                }
            }

			// Refresh ephemeris and subframes
			// Quick and dirty fix. Need more elegant way.
			for (sv=0; sv<MAX_SAT; sv++)
			{
				if (eph[ieph+1][sv].vflg==1)
				{
					dt = subGpsTime(eph[ieph+1][sv].toc, grx);
					if (dt<SECONDS_IN_HOUR)
					{
						ieph++;

						for (i=0; i<MAX_CHAN; i++)
						{
							// Generate new subframes if allocated
							if (chan[i].gps_sat.PRN != 0)
								eph2sbf(eph[ieph][chan[i].gps_sat.PRN - 1], ionoutc, env.alm, chan[i].sbf);
                            if (s->ch2_enable)
                            {
                                if (chan2[i].gps_sat.PRN != 0)
                                    eph2sbf(eph[ieph][chan2[i].gps_sat.PRN - 1], ionoutc, env.alm, chan2[i].sbf);
                            }
						}
					}

					break;
				}
			}

			// Update channel allocation
			allocateChannel(chan, allocatedSat, eph[ieph], &env, xyz[iumd], s->opt.rec_ant_dir, elvmask, visibility_log_files[0]);
            if (s->ch2_enable)
            {
                allocateChannel(chan2, allocatedSat2, eph[ieph], &env, xyz2[iumd], s->opt2.rec_ant_dir, elvmask, visibility_log_files[1]);
            }

            // Show details about simulated channels
            if (s->opt.verb)
            {
                printf("\n");
                gps2date(&grx, &t0);
                printf("%4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                    t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, grx.week, grx.sec);
                // channel 1
                printf("channel 1\n");
                printChannelInformation(chan, xyz[iumd]);

                // channel 2
                if (s->ch2_enable)
                {
                    printf("\nchannel 2\n");
                    printChannelInformation(chan2, xyz2[iumd]);
                }
            }
        }

		// Update receiver time
		grx = incGpsTime(grx, 0.1);
        env.g = &grx;

		// Update time counter
		// printf("\rTime into run = %4.1f", subGpsTime(grx, g0));
		// fflush(stdout);
	}

abort:
#ifndef _WIN32
	changemode(0);
#endif

	// Done!
	s->finished = true;

	// Free I/Q buffer
	free(iq_buff);

	// Free user motion array
	for (i=0; i<USER_MOTION_SIZE; i++)
		free(xyz[i]);
	free(xyz);

	// Close log file
	for (i = 0; i < 2; i++)
	{
		if (dump_user_pos[i])
		{
			fclose(log_files[i]);
			fclose(visibility_log_files[i]);
		}
	}

exit:
	// Close log file
	for (i = 0; i < 2; i++)
	{
		if (dump_user_pos[i])
		{
			fclose(log_files[i]);
			fclose(visibility_log_files[i]);
		}
	}

	printf("Abort.\n");
	return (NULL);
}
