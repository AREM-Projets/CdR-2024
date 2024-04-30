import RPi.GPIO as GPIO
from tkinter import *
from time import sleep

score = 0

def update_affichage_score():
    global score

    score =score + 1
    score_label.config(text="Score robot: "+str(score))
    fenetre.after(100, update_affichage_score)




PIN_TIRETTE = 16
PIN_SELECTEUR_EQUIPE = 23


GPIO.setmode(GPIO.BCM)

GPIO.setup(PIN_TIRETTE, GPIO.IN, pull_up_down = GPIO.PUD_UP) #set la pin 16 en input pulldown pour la tirette
GPIO.setup(PIN_SELECTEUR_EQUIPE, GPIO.IN, pull_up_down = GPIO.PUD_UP) #set la pin 26 en input pulldown pour le sélecteur équipe à 0 par défaut : 0:Bleu; 1:Jaune

equipe = GPIO.input(PIN_SELECTEUR_EQUIPE)

print("tirette:", GPIO.input(PIN_TIRETTE))
print("selecteur:", equipe)




fenetre = Tk()
score_label = Label(fenetre, text="Score robot: 0", font='Times 50')
score_label.pack()

fenetre.after(100, update_affichage_score)


fenetre.mainloop()




while(not GPIO.input(PIN_TIRETTE)): pass

print("Demarrage !")