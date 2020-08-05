//=============================================================================================
// Madgwick.c
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=============================================================================================

#include "BMX160.h"



BMX160::BMX160()
	: ax(0), ay(0), az(0), gx(0), gy(0), gz(0), q0(1), q1(0), q2(0), q3(0)
{
	beta = 0.1f; // 2 * Gain proportionnel (Madgwick)
}

void BMX160::setAcceleration(float _ax, float _ay, float _az)
{
	// Composantes
	ax = _ax;
	ay = _ay;
	az = _az;

	// Module de l'acc�l�ration
	a = sqrt(ax * ax + ay * ay + az * az);
}


void BMX160::setRotation(float _gx, float _gy, float _gz)
{
	// Composantes
	gx = _gx;
	gy = _gy;
	gz = _gz;
}


float BMX160::acceleration(int _axe)
{
	if (_axe == 0) {
		return ax;
	}
	else if (_axe == 1) {
		return ay;
	}
	else if (_axe == 2) {
		return az;
	}
	else {
		return 0.0f;
	}
}


float BMX160::rotation(int _axe)
{
	if (_axe == 0) {
		return gx;
	}
	else if (_axe == 1) {
		return gy;
	}
	else if (_axe == 2) {
		return gz;
	}
	else {
		return 0.0f;
	}
}


// Racine carr�e inverse (pour la fct. orientationUpdate)
// Voir: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}


void BMX160::orientationUpdate(float _delta_t)
{
	float recipNorm;
	float _ax, _ay, _az;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	
	// Taux de variation du quaternion depuis le gyroscope ("-" devant les composantes car on veut corriger ces rotations)
	qDot1 = 0.5f * (-q1 * - gx - q2 * - gy - q3 * - gz);
	qDot2 = 0.5f * (q0 * - gx + q2 * - gz - q3 * - gy);
	qDot3 = 0.5f * (q0 * - gy - q1 * - gz + q3 * - gx);
	qDot4 = 0.5f * (q0 * - gz + q1 * - gy - q2 * - gx);

	// Calcule la r�troaction uniquement si la mesure de l'acc�l�rom�tre est valide (�vite les 0 dans la normalisation de l'acc�l�rom�tre)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise la mesure de l'acc�l�rom�tre
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		_ax = ax * recipNorm;
		_ay = ay * recipNorm;
		_az = az * recipNorm;

		// Variables auxiliaires pour �viter l'arithm�tique r�p�t�e (moins de CPU load)
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// �tape corrective d'algorithme de descente de gradient
		s0 = _4q0 * q2q2 + _2q2 * _ax + _4q0 * q1q1 - _2q1 * _ay;
		s1 = _4q1 * q3q3 - _2q3 * _ax + 4.0f * q0q0 * q1 - _2q0 * _ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * _az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * _ax + _4q2 * q3q3 - _2q3 * _ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * _az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * _ax + 4.0f * q2q2 * q3 - _2q2 * _ay;

		// Normalise la magnitude du pas
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Applique l'�tape de r�troaction (facteur beta)
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Int�gre le taux de changement de quaternion pour produire le quaternion
	q0 += qDot1 * _delta_t;
	q1 += qDot2 * _delta_t;
	q2 += qDot3 * _delta_t;
	q3 += qDot4 * _delta_t;

	// Normalise le quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void BMX160::updateAccelerationOrientation()
{
	// Quaternion(q0, q1, q2, q3) x Acc�l�ration(0, ax, ay, az)
	float qa0, qa1, qa2, qa3;
	qa0 = -(q1 * ax) - (q2 * ay) - (q3 * az);
	qa1 = (q0 * ax) + (q2 * az) - (q3 * ay);
	qa2 = (q0 * ay) - (q1 * az) + (q3 * ax);
	qa3 = (q0 * az) + (q1 * ay) - (q2 * ax);

	// Conjugu� du quaternion (q*)
	float cq0, cq1, cq2, cq3;
	cq0 = q0;
	cq1 = - q1;
	cq2 = - q2;
	cq3 = - q3;

	// QuaternionMultAcc(qa0, qa1, qa2, qa3) x ConjQuaternion(cq0, cq1, cq2, cq3)
	float qaq_0, qaq_1, qaq_2, qaq_3;
	qaq_0 = qa0 * cq0 - qa1 * cq1 - qa2 * cq2 - qa3 * cq3;
	qaq_1 = qa0 * cq1 + qa1 * cq0 + qa2 * cq3 - qa3 * cq2;
	qaq_2 = qa0 * cq2 - qa1 * cq3 + qa2 * cq0 + qa3 * cq1;
	qaq_3 = qa0 * cq3 + qa1 * cq2 - qa2 * cq1 + qa3 * cq0;

	// Mise � jour des acc�l�rations
	ax = qaq_1;
	ay = qaq_2;
	az = qaq_3;
	a = sqrt(ax * ax + ay * ay + az * az);
}


BMX160::~BMX160()
{
}