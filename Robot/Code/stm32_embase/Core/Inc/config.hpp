#pragma once

/* Consigne de vitesse max envoyée au shield moteurs (en pulse per second) */
#define MAX_VITESSE 600.0

#define DEG_PER_FULL_STEP 1.8

#define BASE_MOVEMENT_DIST_M 0.05

// Nombre max de tâches stockées (buffer circulaire)
#define MAX_TASK_COUNT 300

#define BASE_MOTOR_DELAY_MS 100

#define coeff_1x 1
#define coeff_1y 1
#define coeff_1z 1

#define coeff_2x 1
#define coeff_2y 1
#define coeff_2z 1

#define coeff_3x 1
#define coeff_3y 1
#define coeff_3z 1

//Caracteristiques géométriques
#define diametre_embase 0.13 //en m
#define Rayon_Roue 0.055 //en m
#define coeff_Y 0.866025404 //sqrt(3)/2

#define W_MAX 10 //en /rad/s

/*
	Utile ?
#define V_MAX 1.5 //en /m/s
#define Delay_Rotation 2000 //en ms
#define Delay_Translation 10000 //en ms
*/
#define Coeff_erreur_y 1
#define Coeff_erreur_x 1
#define Coeff_erreur_z 45/44

#define Vitesse_moy 0.283 //en m/s
#define Vitesse_Rotation_moy 45 //en °/s
#define Nombre_instructions 100 //Nombre d'instruction maximum avant overflow
