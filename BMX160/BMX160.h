#pragma once
#include <string>


using namespace std;

class BMX160
{
private:
	float beta;			                              	   // Gain algorithme de Madgwick

	float ax, ay, az, a;                                   // Vecteur accéleration (accéléromètre), a = module
	float gx, gy, gz;                                      // Vecteur rotation (gyroscope)

	float vx, vy, vz, v;                                   // Vecteur vitesse (par intégration), v = module
	float px, py, pz, p;                                   // Vecteur position (par intégration), p = module

	float q0, q1, q2, q3;                                  // Quaternion de rotation. Dans le setup, q(w, x, y, z) = q(q0, q1, q2, q3) = q(1, 0, 0, 0) = rotation nulle au départ

	float phi, theta, psy;                                 // Angles d'Euler (angles de rotation autour des axes)

	float old_ax, old_ay, old_az;                          // Valeurs tampons de a pour intégration vitesse & position
	float old_vx, old_vy, old_vz;
public:
	BMX160();                                              // Constructeur de l'objet

	void setAcceleration(float _ax, float _ay, float _az); // Met à jour les accélérations mesurées
	void setRotation(float _gx, float _gy, float _gz);     // Met à jour les rotations mesurées

	void setOldAcceleration(float _old_ax, float _old_ay, float _old_az); // Met à jour les accélérations mesurées précédemment

	float acceleration(int _axe);                          // Retourne l'accélération. 0 = ax, 1 = ay, 2 = az
	float rotation(int _axe);                              // Retourne l'accélération. 0 = gx, 1 = gy, 2 = gz

	float velocity(int _axe);                              // Retourne la vitesse. 0 = vx, 1 = vy, 2 = vz
	float position(int _axe);                              // Retourne la position. 0 = px, 1 = py, 2 = pz

	void orientationUpdate(float _delta_t);                // Calcul du quaternion de rotation en fct. des données du gyroscope (ALGO. DE MADGWICK)

	void updateAccelerationOrientation();                  // Met à jour le vecteur (ax, ay, az) en fct. du quaternion (recalcul des composantes sur les 3 axes en fct. de la rotation mesurée par le gyroscope)

	void integrationSpeedPosition(float _delta_t);                       // Calcul des points de vitesse & position par intégration

	void updateEulerAngles();                              // Met à jour les angles d'Euler
	float eulerAngle(int _axe);                            // Retourne l'angle d'Euler demandé. 0 = phi (x), 1 = theta (y), 2 = psi (z) en degrés

	~BMX160();                                             // Destructeur de l'objet
};

