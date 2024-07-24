#include "ComplementaryFilter.h"
#define GravitationalAcceleration 9.8

//�����Ʒ� ����: z(+)

//���ʹϾ� �ʱ�ȭ
//���ӵ� ������ z(+), �ڱ��� ������ xz��鿡 ��ġ�ϴ� ��ǥ�迡 ���� ���ʹϾ�
void OrientationInitialize(Quaternion* q, double accel[3], double mag[3])
{
	double AccelNorm = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	double MagNorm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	double NormalizedAccel[3], NormalizedMag[3], PredictedMagneticfield[3];
	double Gamma;
	NormalizedAccel[0] = accel[0] / AccelNorm;
	NormalizedAccel[1] = accel[1] / AccelNorm;
	NormalizedAccel[2] = accel[2] / AccelNorm;
	NormalizedMag[0] = mag[0] / MagNorm;
	NormalizedMag[1] = mag[1] / MagNorm;
	NormalizedMag[2] = mag[2] / MagNorm;

	if (NormalizedAccel[2] > 0)
	{
		Quaternion_set(sqrt((NormalizedAccel[2] + 1) / 2), -NormalizedAccel[1] / sqrt(2 * (NormalizedAccel[2] + 1)),
			NormalizedAccel[0] / sqrt(2 * (NormalizedAccel[2] + 1)), 0, q);
	}
	else
	{
		Quaternion_set(-NormalizedAccel[1] / sqrt(2 * (-NormalizedAccel[2] + 1)), sqrt((-NormalizedAccel[2] + 1) / 2),
			0, NormalizedAccel[0] / sqrt(2 * (NormalizedAccel[2] + 1)), q);
	}

	Quaternion q_conjugate, q_delta;
	Quaternion_conjugate(q, &q_conjugate);
	Quaternion_rotate(&q_conjugate, NormalizedMag, PredictedMagneticfield);
	Gamma = PredictedMagneticfield[0] * PredictedMagneticfield[0] + PredictedMagneticfield[1] * PredictedMagneticfield[1];
	
	
	if (NormalizedMag[0] > 0)
	{
		Quaternion_set(sqrt(Gamma + PredictedMagneticfield[0] * sqrt(Gamma)) / sqrt(2 * Gamma), 0, 0,
			PredictedMagneticfield[1] / sqrt(2 * (Gamma + PredictedMagneticfield[0] * sqrt(Gamma))), &q_delta);

	}
	else
	{
		Quaternion_set(PredictedMagneticfield[1] / sqrt(2 * (Gamma - PredictedMagneticfield[0] * sqrt(Gamma))), 0, 0,
			sqrt(Gamma - PredictedMagneticfield[0] * sqrt(Gamma)) / sqrt(2 * Gamma), &q_delta);
	}

	Quaternion_multiply(q, &q_delta, q);
}

void ComplemenatryFilter(Quaternion* q, double gyro[3], double accel[3], double mag[3],
	double AccelGain, double MagGain, double LerpThreshold, double SamplingPeriod, int DoMagnetometer)
{
	//���̷η� ���� ���ʹϾ�
	GyroPrediction(q, gyro, SamplingPeriod);
	//���ӵ���� ����
	AccelCorrection(q, accel, AccelGain, LerpThreshold);
	//���ڱ��� ����
	if(DoMagnetometer != 0)
		MagCorrection(q, mag, MagGain, LerpThreshold);
}

void GyroPrediction(Quaternion* q, double gyro[3], double SamplingPeriod)
{
	Quaternion gyro_q, delta;
	//������ �ߴ� ������ ���ʹϾ� ���
	Quaternion_set(0, gyro[0], gyro[1], gyro[2], &gyro_q);
	Quaternion_multiply(&gyro_q, q, &delta);
	Quaternion_ratio(&delta, 0.5 * SamplingPeriod, &delta);
	Quaternion_addition(q, &delta, q);
	Quaternion_normalize(q, q);
}

void AccelCorrection(Quaternion* q, double accel[3], double AccelGain, double LerpThreshold)
{
	//������ ���ӵ��� ����
	double AccelNorm = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	//���ӵ��� ������ ������
	double MagnitudeError = fabs(AccelNorm - GravitationalAcceleration) / GravitationalAcceleration;
	double GainFactor = 0.0;
	double threshold1 = 0.1, threshold2 = 0.2;
	double NormalizedAccel[3], PredictedGravity[3];
	//������ ����ȭ
	NormalizedAccel[0] = accel[0] / AccelNorm;
	NormalizedAccel[1] = accel[1] / AccelNorm;
	NormalizedAccel[2] = accel[2] / AccelNorm;

	//���ӵ��� �߷°��ӵ��� ���̰� ũ�� ������ AccelGain ����
	if (MagnitudeError < threshold1)
	{
		GainFactor = 1;
	}
	else if ((MagnitudeError >= threshold1) && (MagnitudeError < threshold2))
	{
		GainFactor = (MagnitudeError - threshold2) / (threshold1 - threshold2);
	}
	else
	{
		return;
	}
	AccelGain = AccelGain * GainFactor;
	
	//���ӵ���� ���� ��Ÿ ���ʹϾ�
	Quaternion q_conjugate, q_delta, I;
	Quaternion_setIdentity(&I);
	Quaternion_conjugate(q, &q_conjugate);
	Quaternion_rotate(&q_conjugate, NormalizedAccel, PredictedGravity);
	Quaternion_set(sqrt((PredictedGravity[2] + 1)/2), -PredictedGravity[1]/sqrt(2*(PredictedGravity[2] + 1)), 
		PredictedGravity[0] / sqrt(2 * (PredictedGravity[2] + 1)), 0, &q_delta);
	
	//AccelGain��ŭ ��Ÿ���ʹϾȿ� ����ġ�� �־ ���� ���ʹϾ��� ��ȭ��Ŵ.
	//��Ÿ���ʹϾ��� (1, 0, 0, 0)�� ������ lerp, �ƴϸ� slerp ����
	if (q_delta.w > LerpThreshold)
	{
		Quaternion_lerp(&I, &q_delta, AccelGain, &q_delta);
		Quaternion_normalize(&q_delta, &q_delta);
	}
	else
	{
		Quaternion_slerp(&I, &q_delta, AccelGain, &q_delta);
	}
	Quaternion_multiply(q, &q_delta, q);
}

void MagCorrection(Quaternion* q, double mag[3], double MagGain, double LerpThreshold)
{
	double MagNorm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	double NormalizedMag[3], PredictedMagneticfield[3];
	double Gamma;
	//������ ����ȭ
	NormalizedMag[0] = mag[0] / MagNorm;
	NormalizedMag[1] = mag[1] / MagNorm;
	NormalizedMag[2] = mag[2] / MagNorm;

	//���ڱ��� ���� ��Ÿ ���ʹϾ�
	Quaternion q_conjugate, q_delta, I;
	Quaternion_setIdentity(&I);
	Quaternion_conjugate(q, &q_conjugate);
	Quaternion_rotate(&q_conjugate, NormalizedMag, PredictedMagneticfield);
	Gamma = PredictedMagneticfield[0] * PredictedMagneticfield[0] + PredictedMagneticfield[1] * PredictedMagneticfield[1];
	Quaternion_set(sqrt(Gamma + PredictedMagneticfield[0] * sqrt(Gamma))/sqrt(2 * Gamma), 0, 0,
		PredictedMagneticfield[1] / sqrt(2 * (Gamma + PredictedMagneticfield[0] * sqrt(Gamma))), &q_delta);

	//MagGain��ŭ ��Ÿ���ʹϾȿ� ����ġ�� �־ ���� ���ʹϾ��� ��ȭ��Ŵ.
	//��Ÿ���ʹϾ��� (1, 0, 0, 0)�� ������ lerp, �ƴϸ� slerp ����
	if (q_delta.w > LerpThreshold)
	{
		Quaternion_lerp(&I, &q_delta, MagGain, &q_delta);
		Quaternion_normalize(&q_delta, &q_delta);
	}
	else
	{
		Quaternion_slerp(&I, &q_delta, MagGain, &q_delta);
	}
	Quaternion_multiply(q, &q_delta, q);
}