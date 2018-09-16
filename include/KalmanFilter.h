//Kalman Filter for Z
//private static double Q = 0.000035;
//private static double R = 0.0075;
static double Q = 0.000001;
static double R = 0.01;
static double P = 1, X = 0, K;

static void ZmeasurementUpdate()
{
	K = (P + Q) / (P + Q + R);
	P = R * (P + Q) / (R + P + Q);
}

static double Zupdate(double Zmeasurement)
{
	ZmeasurementUpdate();
	double result = X + (Zmeasurement - X) * K;
	X = result;
	return result;
}

//Kalman Filter for X
static double Q2 = 0.00025;
static double R2 = 0.001;
static double P2 = 1, X2 = 0, K2;
static void XmeasurementUpdate()
{
	K2 = (P2 + Q2) / (P2 + Q2 + R2);
	P2 = R2 * (P2 + Q2) / (R2 + P2 + Q2);
}

static double Xupdate(double Xmeasurement)
{
	XmeasurementUpdate();
	double result2 = X2 + (Xmeasurement - X2) * K2;
	X2 = result2;
	return result2;
}

//Kalman Filter for Y
static double Q3 = 0.00025;
static double R3 = 0.001;
static double P3 = 1, X3 = 0, K3;
static void YmeasurementUpdate()
{
	K3 = (P3 + Q3) / (P3 + Q3 + R3);
	P3 = R3 * (P3 + Q3) / (R3 + P3 + Q3);
}

static double Yupdate(double Ymeasurement)
{
	YmeasurementUpdate();
	double result3 = X3 + (Ymeasurement - X3) * K3;
	X3 = result3;
	return result3;
}

//Kalman Filter for Roll
static double Q4 = 0.00025;
static double R4 = 0.001;
static double P4 = 1, X4 = 0, K4;
static void RmeasurementUpdate()
{
	K4 = (P4 + Q4) / (P4 + Q4 + R4);
	P4 = R4 * (P4 + Q4) / (R4 + P4 + Q4);
}

static double Rupdate(double Rmeasurement)
{
	RmeasurementUpdate();
	double result4 = X4 + (Rmeasurement - X4) * K4;
	X4 = result4;
	return result4;
}

//Kalman Filter for Pitch
static double Q5 = 0.00025;
static double R5 = 0.001;
static double P5 = 1, X5 = 0, K5;
static void PmeasurementUpdate()
{
	K5 = (P5 + Q5) / (P5 + Q5 + R5);
	P5 = R5 * (P5 + Q5) / (R5 + P5 + Q5);
}

static double Pupdate(double Pmeasurement)
{
	PmeasurementUpdate();
	double result5 = X5 + (Pmeasurement - X5) * K5;
	X5 = result5;
	return result5;
}

//Kalman Filter for Yaw
static double Q6 = 0.00025;
static double R6 = 0.001;
static double P6 = 1, X6 = 0, K6;
static void YawmeasurementUpdate()
{
	K6 = (P6 + Q6) / (P6 + Q6 + R6);
	P6 = R6 * (P6 + Q6) / (R6 + P6 + Q6);
}

static double Yawupdate(double Yawmeasurement)
{
	YawmeasurementUpdate();
	double result6 = X6 + (Yawmeasurement - X6) * K6;
	X6 = result6;
	return result6;
}

//Kalman Filter for Time
static double Q7 = 0.00025;
static double R7 = 0.001;
static double P7 = 1, X7 = 0, K7;
static void TimemeasurementUpdate()
{
	K7 = (P7 + Q7) / (P7 + Q7 + R7);
	P7 = R7 * (P7 + Q7) / (R7 + P7 + Q7);
}

static double Timeupdate(double Timemeasurement)
{
	TimemeasurementUpdate();
	double result7 = X7 + (Timemeasurement - X7) * K7;
	X7 = result7;
	return result7;
}

//Kalman Filter for XPos
static double Q8 = 0.00025;
static double R8 = 0.001;
static double P8 = 1, X8 = 0, K8;
static void XPosmeasurementUpdate()
{
	K8 = (P8 + Q8) / (P8 + Q8 + R8);
	P8 = R8 * (P8 + Q8) / (R8 + P8 + Q8);
}

static double XPosupdate(double XPosmeasurement)
{
	XPosmeasurementUpdate();
	double result8 = X8 + (XPosmeasurement - X8) * K8;
	X8 = result8;
	return result8;
}

//Kalman Filter for YPos
static double Q9 = 0.00025;
static double R9 = 0.001;
static double P9 = 1, X9 = 0, K9;
static void YPosmeasurementUpdate()
{
	K9 = (P9 + Q9) / (P9 + Q9 + R9);
	P9 = R9 * (P9 + Q9) / (R9 + P9 + Q9);
}

static double YPosupdate(double YPosmeasurement)
{
	YPosmeasurementUpdate();
	double result9 = X9 + (YPosmeasurement - X9) * K9;
	X9 = result9;
	return result9;
}