#include "bmx160.h" // Biblioth�que du capteur
#include <future>   // Biblioth�que pour threads
#include <iostream>
#include <fstream>
#include <iomanip>  // Pour utiliser "setprecision"
#include <sstream>
#include <string>


using namespace std;


// Fonctions de d�coupe ligne CSV
void splitter(const string &chaine, char delimiteur, vector<float> &elements)
{
	stringstream ss(chaine);
	string sousChaine;
	while (getline(ss, sousChaine, delimiteur))
	{
		float sousChaineFloat = std::stof(sousChaine);
		elements.push_back(sousChaineFloat);
	}
}
vector<float> split(const string &chaine, char delimiteur)
{
	vector<float> elements;
	splitter(chaine, delimiteur, elements);
	return elements;
}


// Fonction de lecture CSV
int lectureCSV(string _nomFichierCSV)
{
	//On cr�e l'objet calistoGravity (acc�l�rom�tre BMX160)
	BMX160 * calistoGravity;
	calistoGravity = new BMX160;

	// A chaque fois qu'on r�cup�re des donn�es de l'acc�l�rom�tre, on r�-ex�cute cette boucle
	// delta_t = temps entre la r�ception de 2 donn�es du gyroscope, ici on le d�finit sur 1 seconde
	float delta_t = 1; //Seconde

	//Lecture du CSV (ici devrait se situer la routine de r�cup�ration des donn�es du Calisto)
	string ligne;

	ifstream streamLectureCSV(_nomFichierCSV.c_str());

	if (streamLectureCSV.is_open()) //Si le CSV s'ouvre sans soucis
	{
		//Ouverture du fichier de sortie (qui contiendra les donn�es trait�es par l'algorithme)
		ofstream streamCSVSortie("datas_result.csv");

		if (streamCSVSortie)
		{
			int iterations = 0;
			while (getline(streamLectureCSV, ligne))
			{
				iterations++;
				vector<float> donnees = split(ligne, ';');
				cout << endl << endl << "IMU Datas        | Iteration n." << iterations << endl;

				// Convertir des degr�s/s en radians/s
				//gx *= 0.0174533f;
				//gy *= 0.0174533f;
				//gz *= 0.0174533f;

				// Mise � jour des donn�es acc�l�rom�tre et gyroscope de l'objet
				calistoGravity->setAcceleration(donnees[7], donnees[8], donnees[9]); // (� mettre en rad/s)
				calistoGravity->setRotation(donnees[4], donnees[5], donnees[6]);

				// Calcul du quaternion de rotation
				calistoGravity->orientationUpdate(delta_t);

				// Recalcul vecteur acc�l�ration
				calistoGravity->updateAccelerationOrientation();

				//Ecriture dans datas_result.csv
				streamCSVSortie << fixed;
				streamCSVSortie << std::setprecision(6) << (int)donnees[0] << ";" << (int)donnees[1] << ";" << (int)donnees[2] << ";" << (int)donnees[3] << ";0;0;0;" << calistoGravity->acceleration(0) << ";" << calistoGravity->acceleration(1) << ";" << calistoGravity->acceleration(2) << endl;
			}
			streamLectureCSV.close();
		}
		else
		{
			cout << "Error while writing into data_result.csv" << endl;
		}
	}
	else
	{
		cout << endl << "Impossible d'ouvrir le dataset" << endl;
		return 1;
	}

	return 0;
}

int main()
{
	bool dataRead = false; // Le fichier CSV n'a pas encore �t� lu (false)

	while (dataRead == false) // Tant que le fichier CSV n'a pas �t� lu, on est dans le while
	{
		// Demande � l'utilisateur le fichier CSV d'entr�e
		string nomFichierCSV;
		cout << "Entrer le nom du fichier CSV: ";
		cin >> nomFichierCSV;

		// Clear console
		system("cls");

		// Ouverture d'un thread pour la lecture du fichier CSV
		cout << nomFichierCSV << " est en cours de traitement..." << endl;
		future csvReadThread = async(&lectureCSV, nomFichierCSV);

		// On r�cup�re le r�sultat de l'ex�cution du thread
		int resultatThreadCSV = csvReadThread.get();

		// Si le thread de lecture s'est bien pass�, retourne 0 (on quitte donc le while, fin du programme)
		if (resultatThreadCSV == 0)
		{
			dataRead = true;
		}
	}

	// La lecture du CSV est termin�e
	system("pause");
}