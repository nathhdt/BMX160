#pragma once
#include <string>


using namespace std;

class BMX160
{
private:
	float beta;			                              	   // Gain algorithme de Madgwick

	float ax, ay, az, a;                                   // Vecteur acc�leration (acc�l�rom�tre), a = module
	float gx, gy, gz;                                      // Vecteur rotation (gyroscope)

	float vx, vy, vz, v;                                   // Vecteur vitesse (par int�gration), v = module
	float px, py, pz, p;                                   // Vecteur position (par int�gration), p = module

	float q0, q1, q2, q3;                                  // Quaternion de rotation. Dans le setup, q(w, x, y, z) = q(q0, q1, q2, q3) = q(1, 0, 0, 0) = rotation nulle au d�part

	float phi, theta, psy;                                 // Angles d'Euler (angles de rotation autour des axes)

	float old_ax, old_ay, old_az;                          // Valeurs tampons de a pour int�gration vitesse & position
	float old_vx, old_vy, old_vz;
public:
	BMX160();                                              // Constructeur de l'objet

	void setAcceleration(float _ax, float _ay, float _az); // Met � jour les acc�l�rations mesur�es
	void setRotation(float _gx, float _gy, float _gz);     // Met � jour les rotations mesur�es

	void setOldAcceleration(float _old_ax, float _old_ay, float _old_az); // Met � jour les acc�l�rations mesur�es pr�c�demment

	float acceleration(int _axe);                          // Retourne l'acc�l�ration. 0 = ax, 1 = ay, 2 = az
	float rotation(int _axe);                              // Retourne l'acc�l�ration. 0 = gx, 1 = gy, 2 = gz

	float velocity(int _axe);                              // Retourne la vitesse. 0 = vx, 1 = vy, 2 = vz
	float position(int _axe);                              // Retourne la position. 0 = px, 1 = py, 2 = pz

	void orientationUpdate(float _delta_t);                // Calcul du quaternion de rotation en fct. des donn�es du gyroscope (ALGO. DE MADGWICK)

	void updateAccelerationOrientation();                  // Met � jour le vecteur (ax, ay, az) en fct. du quaternion (recalcul des composantes sur les 3 axes en fct. de la rotation mesur�e par le gyroscope)

	void integrationSpeedPosition(float _delta_t);                       // Calcul des points de vitesse & position par int�gration

	void updateEulerAngles();                              // Met � jour les angles d'Euler
	float eulerAngle(int _axe);                            // Retourne l'angle d'Euler demand�. 0 = phi (x), 1 = theta (y), 2 = psi (z) en degr�s

	~BMX160();                                             // Destructeur de l'objet
};

