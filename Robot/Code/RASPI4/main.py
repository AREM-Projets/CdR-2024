"""Programme principal du robot de la CDR 2024



https://rplidar.readthedocs.io/en/latest/

angles mesures en deg
distances mesures en mm !!
"""

import time
import serial
from multiprocessing import Process, Queue
from rplidar import RPLidar
import RPi.GPIO as GPIO
from tkinter import *


#parametres raspi
PIN_TIRETTE = 16
PIN_SELECTEUR_EQUIPE = 15 #23
DUREE_VIE_SCAN_LIDAR = 0.5 #duree de vie d'un scan lidar en secondes (au bout de ce temps une mesure du lidar ne sera pas prise en compte)
SEUIL_TEMPS_DETECTION_OBSTACLE = 0.100 #s #ecartement max entre deux mesures lidar considerees comme voisines temporellement
TIMEOUT_ROBOT = 1000 #s (16,6 min)
SEUIL_DANGER_ARRET_COMPLET = 3 #nombre de detections LIDAR au bout duquel on stoppe le robot puis on le redemarre si plus d'obstacle

#parametres LIDAR
SEUIL_DETECTION = 350 #mm
FOV = 100 #deg (champ de vision centré)

#parametres actionneur
TEMPS_ACTION = 7 #s duree de l'action de l'actionneur (pousser un panneau solaire)
#commandes actionneur
ACTIVER_PANNEAU_SOLAIRE = b'e'

#commandes embase
EQUIPE_BLEUE = b'b'
EQUIPE_JAUNE = b'j'
PROG_3_PANNEAUX = b'3'
PROG_6_PANNEAUX = b'6'
INIT = b'i' #restart sans redemarrer la carte
START = b's'
WAIT = b'w'
OK = b'k'
#messages de l'embase
POS_PANNEAU_OK = b'p'






def affichage_score(file_score, file_equipe):
    """
    Processus qui affiche et met a jour le score du robot
    """
    print("Demarrage affichage score...")

    def update_affichage_score():
        if (file_equipe.get() == '1'):
            score_label.config(text="Equipe Jaune\nScore robot: "+str(file_score.get()))
        else:
            score_label.config(text="Equipe Bleue\nScore robot: "+str(file_score.get()))

        fenetre.after(100, update_affichage_score)



    fenetre = Tk()
    score_label = Label(fenetre, text="Equipe ?\nScore robot: 0", font='Times 50')
    score_label.pack()

    fenetre.after(100, update_affichage_score)


    fenetre.mainloop()







def lidar(file_scans):
    print("Demarrage LIDAR...")
    lidar = RPLidar('/dev/rplidar')

    lidar.reset()
    time.sleep(1)

    while True:
        try:

            for i, scan in enumerate(lidar.iter_scans()):
                #print('%d: Got %d measurments' % (i, len(scan)))

                for mesure in scan:

                    try:
                        date = time.monotonic()
                        angle = int(mesure[1])
                        distance = int(mesure[2])

                        #publication de la mesure du lidar uniquement si on detecte un obstacle a moins de 20 cm
                        if((0 < distance < SEUIL_DETECTION)):
                            file_scans.put(f"{date},{angle},{distance}")

                    except:
                        pass

                lidar.clean_input()

        except:
            #si le lidar plante pour une quelconque raison on le redemarre
            lidar.stop()
            print("Fail du LIDAR, restart...")
            lidar.reset()
            time.sleep(2)















def main(file_scans, file_score, file_equipe):
    print("Demarrage MAIN...")

    score = 25 #score estime
    danger = 0 #niveau de danger obstacle
    t_derniere_mesure = 0 #date de la derniere mesure recup du lidar
    flag_embase_en_mouvement = False
    delta_t_mes = 0





    #setup de la tirette et du selecteur equipe
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(PIN_TIRETTE, GPIO.IN, pull_up_down = GPIO.PUD_UP) #set la pin 16 en input pulldown pour la tirette
    GPIO.setup(PIN_SELECTEUR_EQUIPE, GPIO.IN, pull_up_down = GPIO.PUD_UP) #set la pin 26 en input pulldown pour le sélecteur équipe à 0 par défaut : 0:Bleu; 1:Jaune

    equipe = GPIO.input(PIN_SELECTEUR_EQUIPE) #0: bleu; 1:jaune
    file_equipe.put(str(equipe))

    print("tirette:", GPIO.input(PIN_TIRETTE))
    print("équipe:", equipe)



    #connexion a l'embase
    port_embase = serial.Serial('/dev/embase', 115200, timeout=1) #timeout en secondes
    print("[OK] Connexion embase")

    if(equipe == 1):
        port_embase.write(EQUIPE_JAUNE)
    else:
        port_embase.write(EQUIPE_BLEUE)

    port_embase.write(PROG_6_PANNEAUX) #version du code raspi pour 3 panneaux

    #port_embase.write(WAIT) #essai pour que le robot ne roule pas au debut ?
    port_embase.write(INIT)

    


    #connextion a la carte des actionneurs
    port_actionneur = serial.Serial('/dev/actionneur', 9600)
    print("[OK] Connexion actionneur")

    #bloquage tant que la tirette n'est pas tiree
    while(GPIO.input(PIN_TIRETTE)): pass
    t_start = time.monotonic() #on enregistre la date de depart


    port_embase.write(START)
    flag_embase_en_mouvement = True








    print("boucle principale")

    #-------------------------------------------------------------------------------------------------

    while( time.monotonic() - t_start < 90 ):
        file_score.put(score)
        print("ecart mesures lidar:", delta_t_mes)
        print("niveau de danger:", danger)
        print("flag mvt:", flag_embase_en_mouvement)


        #controle lidar
        if (not file_scans.empty()):
            scan = file_scans.get().split(',') #recuperation du dernier scan du lidar
            scan_date = float(scan[0])
            scan_angle = float(scan[1])
            scan_dist = float(scan[2])

            delta_t_mes = scan_date - t_derniere_mesure
            t_derniere_mesure = scan_date

            if (delta_t_mes > SEUIL_TEMPS_DETECTION_OBSTACLE):
                danger = 0
            else:
                danger += 1

            # #controle redemarrage
            # if (not flag_embase_en_mouvement): #si l'embase est a l'arret
            #     if (scan_date > DUREE_VIE_SCAN_LIDAR): #si le dernier scan est vieux, on redemarre
            #         # ici potentiel probleme: si il n'y a plus de mesure du lidar dispo on passe pas par ici
            #         # hypothese potentiellement foireuse: on a toujours des mesures dispos...
            #         port_embase.write(START)
            #         flag_embase_en_mouvement = True

        

        #controle danger
        if (danger > SEUIL_DANGER_ARRET_COMPLET and flag_embase_en_mouvement):
            port_embase.write(WAIT)
            flag_embase_en_mouvement = False
            time.sleep(0.5) #timing

        elif (danger < SEUIL_DANGER_ARRET_COMPLET and not flag_embase_en_mouvement):
            port_embase.write(START)
            flag_embase_en_mouvement = True

        


            # if( (t_derniere_mesure - scan_date < DUREE_VIE_SCAN_LIDAR) ):     #and (-FOV/2 < int(scan[1]) < FOV/2)):
            #     #si le dernier scan n'est pas perime, on fait des trucs avec

                
            #     if (equipe == 0):
            #         #bleu
            #         if ( (360-FOV/2 < int(scan[1]) < 360) or  (0 < int(scan[1]) < FOV/2) ): #on regarde devanT
            #             print("\nSCAN: ({}) {}".format(time.monotonic(), scan))
            #             port_embase.write(WAIT) #si ya un truc devant on s'arrete

                    
            #     else:
            #         #jaune
            #         if (180-FOV/2 < int(scan[1]) < 180+FOV/2): #on regarde derriere
            #             print("\nSCAN: ({}) {}".format(time.monotonic(), scan))
            #             port_embase.write(WAIT)

                        



        #controle actionneur
        if (port_embase.in_waiting):
            print("Lecture message embase")
            message = port_embase.read()
            print(message)
            port_embase.write(OK)

            if (message == POS_PANNEAU_OK):
                #port_embase.write(WAIT)
                port_actionneur.write(ACTIVER_PANNEAU_SOLAIRE) #on actionne le moteur
                time.sleep(TEMPS_ACTION) #on attends la fin de l'actionneur
                port_embase.write(START) #on autorise l'embase a poursuivre



    #-------------------------------------------------------------------------------------------------












if __name__ == "__main__":
    #gestion au plus haut niveau du fonctionnement du robot


    # Files de communication entre processus
    file_scans = Queue() # stocke les valeurs envoyees par le lidar si il detecte
    file_score = Queue() #stocke les valeurs de score calculées pour les passer au processus d'affichage
    file_equipe =Queue() #stocke l'equipe au debut du match

    # Creation des processus concourants pour le robot
    lidar_process =             Process(target=lidar, args=(file_scans,))
    affichage_score_process =   Process(target=affichage_score, args=(file_score,file_equipe))
    main_process =              Process(target=main, args=(file_scans, file_score, file_equipe))


    # Demarrage des processus, attente de 90 secondes et arret des processus
    print("Demarrage des processus...")

    lidar_process.start()
    affichage_score_process.start()
    main_process.start()
    print("Processus demarres.\n")

    time.sleep(TIMEOUT_ROBOT) #secondes

    print("Arret des processus...")
    lidar_process.terminate()
    main_process.terminate()
    print("Processus arretes.\n")