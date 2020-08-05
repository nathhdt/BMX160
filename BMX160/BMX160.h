#pragma once
#include <string>


using namespace std;

class BMX160
{
private:
	float beta;			                              	   // Gain algorithme de Madgwick

	float ax, ay, az, a;                                   // Vecteur acc�leration (acc�l�rom�tre), a = module
	float gx, gy, gz;                                      // Vecteur rotation (gyroscope)

	float q0, q1, q2, q3;                                  // Quaternion de rotation. Dans le setup, q(w, x, y, z) = q(q0, q1, q2, q3) = q(1, 0, 0, 0) = rotation nulle au d�part
public:
	BMX160();                                              // Constructeur de l'objet

	void setAcceleration(float _ax, float _ay, float _az); // Met � jour les acc�l�rations mesur�es
	void setRotation(float _gx, float _gy, float _gz);     // Met � jour les rotations mesur�es

	float acceleration(int _axe);                          // Retourne l'acc�l�ration. 0 = ax, 1 = ay, 2 = az
	float rotation(int _axe);                              // Retourne l'acc�l�ration. 0 = gx, 1 = gy, 2 = gz

	void orientationUpdate(float _delta_t);                // Calcul du quaternion de rotation en fct. des donn�es du gyroscope (ALGO. DE MADGWICK)

	void updateAccelerationOrientation();                  // Met � jour le vecteur (ax, ay, az) en fct. du quaternion (recalcul des composantes sur les 3 axes en fct. de la rotation mesur�e par le gyroscope)

	~BMX160();                                             // Destructeur de l'objet
};

