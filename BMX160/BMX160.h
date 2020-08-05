#pragma once
#include <string>


using namespace std;

class BMX160
{
private:
	float beta;			                              	   // Gain algorithme de Madgwick

	float ax, ay, az, a;                                   // Vecteur accéleration (accéléromètre), a = module
	float gx, gy, gz;                                      // Vecteur rotation (gyroscope)

	float q0, q1, q2, q3;                                  // Quaternion de rotation. Dans le setup, q(w, x, y, z) = q(q0, q1, q2, q3) = q(1, 0, 0, 0) = rotation nulle au départ
public:
	BMX160();                                              // Constructeur de l'objet

	void setAcceleration(float _ax, float _ay, float _az); // Met à jour les accélérations mesurées
	void setRotation(float _gx, float _gy, float _gz);     // Met à jour les rotations mesurées

	float acceleration(int _axe);                          // Retourne l'accélération. 0 = ax, 1 = ay, 2 = az
	float rotation(int _axe);                              // Retourne l'accélération. 0 = gx, 1 = gy, 2 = gz

	void orientationUpdate(float _delta_t);                // Calcul du quaternion de rotation en fct. des données du gyroscope (ALGO. DE MADGWICK)

	void updateAccelerationOrientation();                  // Met à jour le vecteur (ax, ay, az) en fct. du quaternion (recalcul des composantes sur les 3 axes en fct. de la rotation mesurée par le gyroscope)

	~BMX160();                                             // Destructeur de l'objet
};

