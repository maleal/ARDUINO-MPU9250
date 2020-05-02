#ifndef _AhrsQuaternionFilters_H
#define _AhrsQuaternionFilters_H

#include <math.h>

class AhrsQuaternionFilters {
	private:
		const int XAXIS = 0;
		const int YAXIS = 1;
		const int ZAXIS = 2;

		float headingAngle[3] = { 0.0,0.0,0.0 };

		float lkpAcc = 0.0;                						// proportional gain governs rate of convergence to accelerometer
		float lkiAcc = 0.0;                						// integral gain governs rate of convergence of gyroscope biases
		float lkpMag = 0.0;                						// proportional gain governs rate of convergence to magnetometer
		float lkiMag = 0.0;                						// integral gain governs rate of convergence of gyroscope biases
		float lhalfT = 0.0;                						// half the sample period
		float lq0 = 0.0, lq1 = 0.0, lq2 = 0.0, lq3 = 0.0;       // quaternion elements representing the estimated orientation
		float lexInt = 0.0, leyInt = 0.0, lezInt = 0.0;  		// scaled integral error

	public:
		////////////////////////////////////////////////////////////////////////////////
// argUpdate
////////////////////////////////////////////////////////////////////////////////
		void headingUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt);
		void headingEulerAngles();
		void initializeBaseHeadingParam(float rollAngle, float pitchAngle, float yawAngle);

		
	
};


#endif //_AhrsQuaternionFilters_H
