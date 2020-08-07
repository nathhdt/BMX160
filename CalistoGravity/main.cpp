#include "bmx160.h" // Bibliothèque du capteur
#include <future>   // Bibliothèque pour threads
#include <iomanip>  // Affichage console
#include <iostream>
#include <fstream>
#include <iomanip>  // Pour utiliser "setprecision"
#include <sstream>
#include <string>


using namespace std;


// Fonctions de découpe ligne CSV
void splitter(const string &chaine, char delimiteur, vector<string> &elements)
{
	stringstream ss(chaine);
	string sousChaine;
	while (getline(ss, sousChaine, delimiteur))
	{
		elements.push_back(sousChaine);
	}
}
vector<string> split(const string &chaine, char delimiteur)
{
	vector<string> elements;
	splitter(chaine, delimiteur, elements);
	return elements;
}


// Fonction de lecture CSV
int lectureCSV(string _nomFichierCSV)
{
	//On crée l'objet calistoGravity (accéléromètre BMX160)
	BMX160 * calistoGravity;
	calistoGravity = new BMX160;

	//Variables pour calculer le delta_t
	int hrs_old = 0;

	//Lecture du CSV (ici devrait se situer la routine de récupération des données du Calisto)
	string ligne;

	ifstream streamLectureCSV(_nomFichierCSV.c_str());

	if (streamLectureCSV.is_open()) //Si le CSV s'ouvre sans soucis
	{
		//Ouverture du fichier de sortie (qui contiendra les données traitées par l'algorithme)
		ofstream streamCSVSortie("datas_result.csv");

		if (streamCSVSortie)
		{
			int iterations = -2;

			//En-tête fichier sortie CSV
			streamCSVSortie << fixed;
			streamCSVSortie << std::setprecision(6) << "id;histo;tag;hrs;gx;gy;gz;ax;ay;az;vx;vy;vz;px;py;pz" << endl;

			while (getline(streamLectureCSV, ligne))
			{
				if (iterations == -2) {    // Première ligne, titres des colonnes du tableau
					iterations++;
				}
				else if (iterations == -1) // Seconde ligne, premières données du tableau
				{
					iterations++;
					vector<string> donnees = split(ligne, ';');
					hrs_old = std::stoi(donnees[3]);

					calistoGravity->setOldAcceleration(stof(donnees[7]), stof(donnees[8]), stof(donnees[9])); // (à mettre en m/s²)
				}
				else
				{
					iterations++;
					vector<string> donnees = split(ligne, ';');
					cout << endl << endl << endl << "IMU Datas        | Iteration n." << iterations << endl;

					// A chaque fois qu'on récupère des données de l'accéléromètre, on ré-exécute cette boucle
					// delta_t = temps entre la réception de 2 données du capteur, ici on le définit sur 1 seconde

					float delta_t = (stoi(donnees[3]) - hrs_old); // Valeur d'avant - Valeur de maintenant (colonne hrs du CSV)
					delta_t /= 1000;                              // delta_t exprimé en seconde (ms/1000 -> s)
					hrs_old = stoi(donnees[3]);                   // Nouvelle valeur devient l'ancienne valeur pour préparer la prochaine itération
					cout << "Delta T          | " << delta_t << "s" << endl;

					// Mise à jour des données accéléromètre et gyroscope de l'objet

					// /!\ Les dimensions du fichier CSV doivent être en rad/s pour gx, gy, gz et m/s² pour ax, ay, az)
					calistoGravity->setAcceleration(stof(donnees[7]), stof(donnees[8]), stof(donnees[9])); // (à mettre en m/s²)
					calistoGravity->setRotation(stof(donnees[4]), stof(donnees[5]), stof(donnees[6]));     // (à mettre en rad/s)

					// Calcul du quaternion de rotation
					calistoGravity->orientationUpdate(delta_t);

					// Calcul & affichage angles d'Euler
					calistoGravity->updateEulerAngles();
					cout << "Euler angles     | x : " << setw(10) << calistoGravity->eulerAngle(0) << "deg		y : " << setw(10) << calistoGravity->eulerAngle(1) << "deg		z : " << setw(10) << calistoGravity->eulerAngle(2) << "deg" << endl;

					cout << "_________________|______________________________________________________________________________________________" << endl;

					// Recalcul vecteur accélération
					cout << "ACCELERA. (m/s2) | ax : " << setw(10) << calistoGravity->acceleration(0) << "		ay : " << setw(10) << calistoGravity->acceleration(1) << "		az : " << setw(10) << calistoGravity->acceleration(2) << "		|a| : " << setw(10) << calistoGravity->acceleration(3) << endl;
					calistoGravity->updateAccelerationOrientation();
					cout << "ACCELERAT. (new) | ax : " << setw(10) << calistoGravity->acceleration(0) << "		ay : " << setw(10) << calistoGravity->acceleration(1) << "		az : " << setw(10) << calistoGravity->acceleration(2) << "		|a| : " << setw(10) << calistoGravity->acceleration(3) << endl;

					// Calcul vitesse & position (par intégration)
					calistoGravity->integrationSpeedPosition(delta_t);
					cout << "VELOCITY   (m/s) | vx : " << setw(10) << calistoGravity->velocity(0) << "		vy : " << setw(10) << calistoGravity->velocity(1) << "		vz : " << setw(10)	<< calistoGravity->velocity(2) << "		|v| : " << setw(10) << calistoGravity->velocity(3) << endl;
					cout << "POSITION     (m) | px : " << setw(10) << calistoGravity->position(0) << "		py : " << setw(10) << calistoGravity->position(1) << "		pz : " << setw(10) << calistoGravity->position(2) << "		|p| : " << setw(10) << calistoGravity->position(3) << endl;

					//Ecriture dans datas_result.csv
					streamCSVSortie << fixed;
					streamCSVSortie << std::setprecision(6) << stoi(donnees[0]) << ";" << stoi(donnees[1]) << ";" << stoi(donnees[2]) << ";" << stoi(donnees[3]) << ";0;0;0;" << calistoGravity->acceleration(0) << ";" << calistoGravity->acceleration(1) << ";" << calistoGravity->acceleration(2) << ";" << calistoGravity->velocity(0) << ";" << calistoGravity->velocity(1) << ";" << calistoGravity->velocity(2) << ";" << calistoGravity->position(0) << ";" << calistoGravity->position(1) << ";" << calistoGravity->position(2) << endl;
				}

				
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
		cout << endl << "Unable to open dataset" << endl;
		return 1;
	}

	return 0;
}

int main()
{
	bool dataRead = false; // Le fichier CSV n'a pas encore été lu (false)

	while (dataRead == false) // Tant que le fichier CSV n'a pas été lu, on est dans le while
	{
		// Demande à l'utilisateur le fichier CSV d'entrée
		string nomFichierCSV;
		cout << "Enter CSV dataset filename: ";
		cin >> nomFichierCSV;

		// Clear console
		system("cls");

		// Ouverture d'un thread pour la lecture du fichier CSV
		cout << nomFichierCSV << " is being processed..." << endl;
		future csvReadThread = async(&lectureCSV, nomFichierCSV);

		// On récupère le résultat de l'exécution du thread
		int resultatThreadCSV = csvReadThread.get();

		// Si le thread de lecture s'est bien passé, retourne 0 (on quitte donc le while, fin du programme)
		if (resultatThreadCSV == 0)
		{
			dataRead = true;
		}
	}

	// La lecture du CSV est terminée
	cout << endl << endl <<"Open datas_result.csv to get algorithm results." << endl << endl;
	system("pause");
}