#include "Quaternion.h"
void ComplemenatryFilter(Quaternion* q, double gyro[3], double accel[3], double mag[3],
	double AccelGain, double MagGain, double LerpThreshold, double SamplingPeriod, int DoMagnetometer);

void GyroPrediction(Quaternion* q, double gyro[3], double SamplingPeriod);

void AccelCorrection(Quaternion* q, double accel[3], double AccelGain, double LerpThreshold);

void MagCorrection(Quaternion* q, double mag[3], double MagGain, double LerpThreshold);

void OrientationInitialize(Quaternion* q, double accel[3], double mag[3]);