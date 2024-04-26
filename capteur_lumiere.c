
// Programme montrant le r�glage des registres GPIO
// l'�tat du bouton S1 (P1.1) est recopi� sur la led LED1 (P1.0)
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdio.h>
#include <stdlib.h>

uint32_t compteur_gauche = 0; //compteur_gauche tourne lorsque le moteur gauche tourne
uint32_t compteur_droite = 0; //compteur_droit tourne lorsque le moteur droit tourne
uint8_t demi_tour_recent = 0; //0 : pas de demi-tour r�cent // 1 : on a fait un demi_tour r�cemment
uint8_t virage = 0; //0 : pas de virage enregistr� // 1 : on a fait un virage � droite // 2 : on a fait un virage � gauche
uint8_t etat_led = 0b00000000; //Cette variable contiendra l'�tat (haut ou bas) des diff�rents capteurs de lignes

uint16_t n,p,x; // n, p et x
float t,d; // t et d

uint16_t nb_lancement_depuis_blanc = 0;

void SysTick_Handler(void)
{

    P7->DIR |= (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); // Les capteurs de lignes passent en sortie
    P7->OUT |= (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); //On applique un front montant sur ces broches
    __delay_cycles(30); //On attend un petit temps
    P7->DIR &=~ (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7); //On fait passer les capteurs en entr�e

    __delay_cycles(3000); //Apr�s avoir attendu ce temps la (c'est-�-dire 1000 microseconde), si le capteur voit du blanc, alors le signal sur la broche vaut 0
    //Si le capteur voit du noir alors la broche vaut 1

    etat_led = P7->IN; // etat_led contient l'�tat des capteurs de lignes (0 ou 1)

    if (etat_led == 0b00000000) //si tous les capteurs voient du blanc
    {
        nb_lancement_depuis_blanc++; //On incr�mente cette variable
    }
    else
    {
        nb_lancement_depuis_blanc = 0; //Sinon, on remet sa valeur � 0
    }
    //Si cette fonction se lance 150 et qu'� chaque fois, tous les capteurs voient du blanc alors nb_lancement_depuis_blanc vaut 150 et la fonction clafete() va se lancer
}

void TA2_N_IRQHandler(void)
{
    if (TIMER_A2-> CCTL[1] & CCIFG)
    {
        TIMER_A2->CCTL[1] &= ~CCIFG;
        n=n+1;
        if (n==2)
        {
            p= TIMER_A2->CCR[1]-x;
            t = p*0.00000533;
            d= t*17000; // 340M/S en cm + division par 2
        }
        else
        {
            x =TIMER_A2->CCR[1];
        }
    }
}

void TA1_N_IRQHandler(void){ //G�re la p�riode d'envoi de signal

    if(TIMER_A1->CTL & TAIFG)
    {
        n=0;

        P5->SEL0 &= ~BIT6;
        P5->SEL1 &= ~BIT6;
        P5->DIR |= BIT6; //Configuration de la broche en sortie
        P5->OUT |= BIT6; //Envoye d'un signal de d�clenchement
        __delay_cycles(30);
        P5->OUT &= ~BIT6;

        TIMER_A1->CTL&= ~TAIFG;
        P5->DIR &= ~BIT6; //On repasse en entr�e
        P5->SEL0 |= BIT6;
        P5->SEL1 &= ~BIT6;
    }
}


void PORT6_IRQHandler()
{

    if(  (P6->IFG & BIT0)) // Si on a un front descendant sur la broche P6.0 configur�e en GPIO
    {
        P6->IFG &=~BIT0; //Remise du drapeau � 0
        compteur_gauche++; //On incr�mente compteur_gauche
    }

    if( (P6->IFG & BIT4) == BIT4 ) // Si on a un front descendant sur la broche P6.4 configur�e en GPIO
    {
        P6->IFG &=~BIT4; //Remise du drapeau � 0
        compteur_droite++; //On incr�mente compteur_droite
    }
}

void capteur_ultrason()
{
    //Configuration du Timer A1
    TIMER_A1->CTL = TASSEL__ACLK|MC__UP|TACLR|TAIE;
    TIMER_A1->CCR[0] = 3200-1; // pour faire une interruptions toutes les 0.2s

    //Configuration du Timer A2
    TIMER_A2->CTL = TASSEL__SMCLK|MC__CONTINUOUS|TACLR;
    TIMER_A2->CCTL[1] = CAP|CCIS__CCIA|SCS|CM__BOTH|CCIE;

    //Autorisation des interruptions pour ces deux Timers
    NVIC_EnableIRQ(TA1_N_IRQn);
    NVIC_EnableIRQ(TA2_N_IRQn);
    __enable_irq();
}


void moteur()
{
    //R�glage de ces deux broches en GPIO
    P5->SEL0 &=~(BIT4|BIT5);
    P5->SEL1 &=~(BIT4|BIT5);

    //Mise en sortie de ces deux broches. Si on met la sortie � un niveau haut --> MARCHE ARRIERE. Si on met la sortie � un niveau bas --> MARCHE AVANT. P5.4 --> moteur gauche. P5.5 --> moteur droit
    P5->DIR |= (BIT4|BIT5);
    P5->OUT &=~ (BIT5|BIT4); //Marche avant ici

    //P3.7 et P3.6 permettent l'activation ou la d�sactivation des moteurs. P3.7 --> moteur gauche. P3.6 --> moteur droit
    P3->SEL0 &=~(BIT6|BIT7);
    P3->SEL1 &=~(BIT6|BIT7);
    P3->DIR |= (BIT6|BIT7);
    P3->OUT |= (BIT6|BIT7); //Niveau haut ici donc les moteurs sont activ�s ! Si on est sur un niveau bas, les moteurs sont d�sactiv�s

    //Configuration du Timer A0 en mode croissant, sans autorisation des interruptions. On l'utilise en mode compare pour g�n�rer un signal ici
    TIMER_A0->CTL = TASSEL__SMCLK|MC__UP|TACLR;
    TIMER_A0->CCR[0] = 240-1; //TA0.0 compte 20 microseconde

    //R�glage de la sortie sur le port P2.6 (pour le moteur droit) et P2.7 (pour le moteur gauche)
    P2->SEL0 |= (BIT6|BIT7);
    P2->SEL1 &=~ (BIT6|BIT7);
    P2->DIR |= (BIT6|BIT7);

    TIMER_A0->CCR[3] = 200; // TA0.3 compte 16,6 microseconde
    TIMER_A0->CCR[4] = 200; // TA0.4 compte 16,6 microseconde

    //S�lection du mode Set/Reset pour les deux signaux
    TIMER_A0->CCTL[3] = OUTMOD_3;
    TIMER_A0->CCTL[4] = OUTMOD_3;
}

void demi_tour()
{
    //R�glage du signal des codeurs


    // R�glage du codeur gauche P10.5 : C'est un GPIO qui re�oit un signal en fonction des tours que font les roues. Donc ici on l'utilise en entr�e. Sur cette broche, on re�oit donc un signal qu'on va utiliser
    P10->SEL1 &=~(BIT5);
    P10->SEL0 &=~(BIT5);

    P10->DIR &=~ (BIT5);


    // On utilise un port en GPIO et on lui injecte le signal g�n�r� par les codeurs. Donc on met cette broche en entr�e.
    //On utilise la broche P6.0
    P6->SEL0 &=~ BIT0;
    P6->SEL1 &=~ BIT0;
    P6->DIR &=~BIT0;
    P6->REN &=~BIT0;

    //On autorise les interruptions sur ce GPIO
    P6->IES |= BIT0; //Interruption sur un front descendant
    P6->IE = BIT0; //Activation des interruptions juste sur cette broche
    P6->IFG = 0; //On efface les drapeaux

    NVIC_EnableIRQ(PORT6_IRQn);
    NVIC_SetPriority(PORT6_IRQn, 1); //Piorit� plus grande pour incr�menter les variables globales compteur_gauche et compteur_droit que pour utiliser les bumpers

    P1->OUT |= BIT0; // On allume une LED pendant le demi_tour

    P3->OUT &=~ (BIT6|BIT7); //On arr�te le moteur droite et gauche
    __delay_cycles(600000); //On attend 0,2 secondes
    P3->OUT |= (BIT6|BIT7); //On met en marche les moteurs
    P5->OUT |= (BIT5|BIT4); //On met les moteurs en marche arri�re
    TIMER_A0->CCR[3] = 220; //On baisse la vitesse des moteurs en r�glant le rapport cyclique du signal PWM
    TIMER_A0->CCR[4] = 220;
    compteur_gauche = 0; //On r�initialise compteur_gauche
    while(compteur_gauche < 60){} //On attend que cette variable atteigne la valeur 60


    //Un moteur marche avant, l'autre marche arri�re pour que le robot tourne sur lui-m�me
    P5->OUT &=~ (BIT5);
    P5->OUT |= BIT4;
    // On remet les moteurs � une vitesse normale
    TIMER_A0->CCR[3] = 200;
    TIMER_A0->CCR[4] = 200;
    compteur_gauche = 0; //On r�initialise compteur_gauche
    while(compteur_gauche < 360){} //On attend que cette variable atteigne la valeur 360 et donc que le robot fasse un demi-tour


    P5->OUT &=~ (BIT5|BIT4); //On retourne en marche avant

    P1->OUT &=~BIT0; // On �teint la LED
}


void deux_tours_demi()
{
    //R�glage du signal des codeurs


    // R�glage du codeur gauche P10.5 : C'est un GPIO qui re�oit un signal en fonction des tours que font les roues. Donc ici on l'utilise en entr�e. Sur cette broche, on re�oit donc un signal qu'on va utiliser
    P10->SEL1 &=~(BIT5);
    P10->SEL0 &=~(BIT5);

    P10->DIR &=~ (BIT5);


    // On utilise un port en GPIO et on lui injecte le signal g�n�r� par les codeurs. Donc on met cette broche en entr�e.
    //On utilise la broche P6.0
    P6->SEL0 &=~ BIT0;
    P6->SEL1 &=~ BIT0;
    P6->DIR &=~BIT0;

    P6->REN &=~BIT0;

    //On autorise les interruptions sur ce GPIO
    P6->IES |= BIT0; //Interruption sur un front descendant
    P6->IE = BIT0; //Activation des interruptions juste sur notre broche
    P6->IFG = 0; //On efface les drapeaux

    NVIC_EnableIRQ(PORT6_IRQn);
    NVIC_SetPriority(PORT6_IRQn, 1); //Piorit� plus grande pour incr�menter les variables globales compteur_gauche et compteur_droit que pour utiliser les bumpers


    //Un moteur marche avant, l'autre marche arri�re pour que le robot tourne sur lui-m�me
    P5->OUT &=~ (BIT5);
    P5->OUT |= BIT4;
    compteur_gauche = 0; //On r�initialise compteur_gauche
    while(compteur_gauche < 1880){} //On attend trois tours, c'est-�-dire que cette vairable atteigne 1880

}


void quart_tour_droite()
{
    // R�glage du codeur gauche P10.5 : C'est un GPIO qui re�oit un signal en fonction des tours que font les roues. Donc ici on l'utilise en entr�e. Sur cette broche, on re�oit donc un signal qu'on va utiliser
    P10->SEL1 &=~(BIT5);
    P10->SEL0 &=~(BIT5);
    P10->DIR &=~ (BIT5);

    // On utilise un port en GPIO et on lui injecte le signal g�n�r� par les codeurs. Donc on met cette broche en entr�e.
    //On utilise la broche P6.0
    P6->SEL0 &=~ BIT0;
    P6->SEL1 &=~ BIT0;
    P6->DIR &=~BIT0;

    P6->REN &=~BIT0;

    //On autorise les interruptions sur ce GPIO
    P6->IES |= BIT0; //Interruptions sur les fronts descendants
    P6->IE = BIT0; //Activation des interruptions juste sur notre broche
    P6->IFG = 0; //On efface les drapeaux

    NVIC_EnableIRQ(PORT6_IRQn);

    //On allume 2 LEDs pour le virage � droite
    P8->OUT |= (BIT5|BIT7);

    P3->OUT &=~ (BIT6|BIT7); //On arr�te le moteur droite et gauche
    __delay_cycles(600000); //On attend 0,2 secondes
    P3->OUT |= (BIT6|BIT7); //On met en marche les moteurs
    P5->OUT |= (BIT5|BIT4); //On met les moteurs en marche arri�re
    TIMER_A0->CCR[3] = 220; //On baisse la vitesse
    TIMER_A0->CCR[4] = 220;
    compteur_gauche = 0; //On r�initialise compteur_gauche
    while(compteur_gauche < 80) //On attend que cette variable atteigne la valeur 80
    {}


    P5->OUT &=~ (BIT5|BIT4); //On met les moteurs en marche avant
    P3->OUT &=~ (BIT6); //On arr�te le moteur droite
    TIMER_A0->CCR[3] = 200; //On remet les vitesses normales
    TIMER_A0->CCR[4] = 200;
    compteur_gauche = 0; //On r�initialise compteur_gauche
    while(compteur_gauche < 360) //On attend que cette variable atteigne la valeur 360 et donc que le robot ait fait un virage � droite
    {}

    P3->OUT |= (BIT6); //On remet le moteur droite en route

    //On �teint 2 LEDs � la fin du virage
    P8->OUT &=~ (BIT5|BIT7);

}

void quart_tour_gauche()
{
    // R�glage du codeur droit P10.4 : C'est un GPIO qui re�oit un signal en fonction des tours que font les roues. Donc ici on l'utilise en entr�e. Sur cette broche, on re�oit donc un signal qu'on va utiliser
    P10->SEL1 &=~(BIT4);
    P10->SEL0 &=~(BIT4);

    P10->DIR &=~ (BIT4);

    // On utilise un port en GPIO et on lui injecte le signal g�n�r� par les codeurs. Donc on met cette broche en entr�e.
    //On utilise la broche P6.4
    P6->SEL0 &=~ BIT4;
    P6->SEL1 &=~ BIT4;
    P6->DIR &=~BIT4;

    P6->REN &=~BIT4;

    //On autorise les interruptions sur ce GPIO
    P6->IES |= BIT4; //Interruptions sur les fronts descendants
    P6->IE = BIT4; //Activation des interruptions juste sur notre broche
    P6->IFG = 0; //On efface les drapeaux

    NVIC_EnableIRQ(PORT6_IRQn);

    //On allume 2 LEDs pour le virage � gauche
    P8->OUT |= (BIT0|BIT6);

    P3->OUT &=~ (BIT6|BIT7); //On arr�te le moteur droite et gauche
    __delay_cycles(600000); //On attend 0,2 secondes
    P3->OUT |= (BIT6|BIT7); //On met en marche les moteurs
    P5->OUT |= (BIT5|BIT4); //On met les moteurs en marche arri�re
    TIMER_A0->CCR[3] = 220; //On baisse la vitesse
    TIMER_A0->CCR[4] = 220;
    compteur_droite = 0; //On r�initialise compteur_droite
    while(compteur_droite < 80){} //On attend que cette variable atteigne la valeur 80


    P5->OUT &=~ (BIT5|BIT4); //Marche avant pour les deux moteurs
    P3->OUT &=~ (BIT7); //On arr�te le moteur gauche
    TIMER_A0->CCR[3] = 200; //On remet des vitesses normales
    TIMER_A0->CCR[4] = 200;
    compteur_droite = 0; //On r�initialise compteur_droite
    while(compteur_droite < 360){} //On attend que cette variable atteigne la valeur 360 et donc que le robot ait fait un virage � gauche

    P3->OUT |= (BIT7); //On remet le moteur gauche en route

    //On �teint les 2 LEDs � la fin du virage
    P8->OUT &=~ (BIT0|BIT6);
}

void bumper()
{
    //R�glage en mode GPIO
    P4->SEL0 &=~(BIT0|BIT2|BIT3|BIT5|BIT6|BIT7);
    P4->SEL1 &=~(BIT0|BIT2|BIT3|BIT5|BIT6|BIT7);
    P4->DIR &=~(BIT0|BIT2|BIT3|BIT5|BIT6|BIT7); //R�glage de la direction en entr�e
    P4->REN |=(BIT0|BIT2|BIT3|BIT5|BIT6|BIT7); //Les bumpers ont besoin d'une r�sistance de pull
    P4->OUT |=(BIT0|BIT2|BIT3|BIT5|BIT6|BIT7); //R�sistance de pull-up

    //On active les interruptions sur le port 4
    //Quand on appuie, le niveau passe � 0. Donc on veut des interruptions sur les fronts descendants
    P4->IES |= (BIT0|BIT2|BIT3|BIT5|BIT6|BIT7); //Interruptions sur front descendant
    P4->IE |= (BIT0|BIT2|BIT3|BIT5|BIT6|BIT7); //On active les interruptions sur toutes les broches des bumpers
    P4->IFG &=~(BIT0|BIT2|BIT3|BIT5|BIT6|BIT7); //On efface les drapeaux d'interruptions

    NVIC_EnableIRQ(PORT4_IRQn);
    NVIC_SetPriority(PORT4_IRQn, 2); //On r�gle la priorit� plus basse que pour le port 6. Quand on va appuyer sur les bumpers, on va sortir de la routine d'interruption li�e aux bumpers pour pouvoir incr�menter les variables globales compteur_gauche et compteur_droit et continuer � avoir des informations sur la distance que parcourt le robot
}


void PORT4_IRQHandler()
{
    if( (P4->IFG & BIT0) == BIT0 ) //Si on appuie sur le bumper tout � droite
    {
        demi_tour(); //On fait un demi-tour
        demi_tour_recent = 1;
        P4->IFG = 0; //On remet tout le registre � 0 car m�me si on a appuy� sur plusieurs bumpers, on veut seulement un seul demi-tour
    }
    if( (P4->IFG & BIT2) == BIT2 ) //Si on appuie sur le deuxi�me bumper en partant de la droite
    {
        demi_tour(); //On fait un demi-tour
        demi_tour_recent = 1;
        P4->IFG = 0; //On remet tout le registre � 0 car m�me si on a appuy� sur plusieurs bumpers, on veut seulement un seul demi-tour
    }
    if( (P4->IFG & BIT3) == BIT3 ) //Si on appuie sur le troisi�me bumper en partant de la droite
    {
        demi_tour(); //On fait un demi-tour
        demi_tour_recent = 1;
        P4->IFG = 0; //On remet tout le registre � 0 car m�me si on a appuy� sur plusieurs bumpers, on veut seulement un seul demi-tour
    }

    if( (P4->IFG & BIT5) == BIT5 ) //Si on appuie sur le troisi�me bumper en partant de la gauche
    {
        demi_tour(); //On fait un demi-tour
        demi_tour_recent = 1;
        P4->IFG = 0; //On remet tout le registre � 0 car m�me si on a appuy� sur plusieurs bumpers, on veut seulement un seul demi-tour
    }

    if( (P4->IFG & BIT6) == BIT6 ) //Si on appuie sur le deuxi�me bumper en partant de la gauche
    {
        demi_tour(); //On fait un demi-tour
        demi_tour_recent = 1;
        P4->IFG = 0; //On remet tout le registre � 0 car m�me si on a appuy� sur plusieurs bumpers, on veut seulement un seul demi-tour
    }

    if( (P4->IFG & BIT7) == BIT7 ) //Si on appuie sur le bumper tout � gauche
    {
        demi_tour(); //On fait un demi-tour
        demi_tour_recent = 1;
        P4->IFG = 0; //On remet tout le registre � 0 car m�me si on a appuy� sur plusieurs bumpers, on veut seulement un seul demi-tour
    }
}


void capteur_lumiere()
{
    //P5.3 g�re les LEDs paires. Mode GPIO, en sortie, et � un niveau haut pour cette broche --> activation des LEDs paires
    P5->SEL0 &=~BIT3;
    P5->SEL1 &=~BIT3;
    P5->DIR |= BIT3;
    P5->OUT |= BIT3;

    //P9.2 g�re les LEDs impaires. Mode GPIO, en sortie, et � un niveau haut pour cette broche --> activation des LEDs impaires
    P9->SEL0 &=~BIT2;
    P9->SEL1 &=~BIT2;
    P9->DIR |= BIT2;
    P9->OUT |= BIT2;

    //On r�gle les 8 capteurs de lumi�re, en mode GPIO, en sortie (pour le moment) et � un niveau bas
    P7->SEL0 &=~ (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
    P7->SEL1 &=~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
    P7->DIR |= (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);
    P7->OUT &=~ (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7);

}
void led()
{
    //P2.2 --> LED bleue sur le microcontr�leur
    //R�glage de cette broche en mode GPIO, en sortie et � un niveau haut pour l'allumer
    P2->SEL0 &=~ BIT2;
    P2->SEL1 &=~BIT2;
    P2->DIR |=BIT2;
    P2->OUT |= BIT2; //On allume la LED bleue

    //P1.0 --> LED rouge sur le microcontr�leur
    //R�glage de cette broche en mode GPIO, en sortie et � un niveau bas pour l'�teindre
    P1->SEL0 &=~ BIT0;
    P1->SEL1 &=~BIT0;
    P1->DIR |=BIT0;
    P1->OUT &=~ BIT0;

    //P8.0 --> LED avant gauche sur le robot
    //P8.5 --> LED avant droite sur le robot
    //P8.6 --> LED arri�re gauche sur le robot
    //P8.7 --> LED arri�re droite sur le robot
    //R�glage de ces broches en mode GPIO, en sortie et � un niveau bas pour les �teindre
    P8->SEL0 &=~(BIT0 | BIT5 | BIT6 | BIT7);
    P8->SEL1 &=~(BIT0 | BIT5 | BIT6 | BIT7);
    P8->DIR |= (BIT0 | BIT5 | BIT6 | BIT7);
    P8->OUT &=~(BIT0 | BIT5 | BIT6 | BIT7);

}

void clafete()
{
    //On allume toutes les LEDs
    P8->OUT |= (BIT0 | BIT5 | BIT6 | BIT7);
    P1->OUT |= BIT0;

    deux_tours_demi(); //Le robot fait deux tours et demi sur lui-m�me

    //On �teint les LEDs
    P8->OUT &= ~(BIT0 | BIT5 | BIT6 | BIT7);
    P1->OUT &=~ BIT0;

    capteur_ultrason(); // On initialise le capteur � ultrason

    P5->OUT |= (BIT5|BIT4); // On va en marche arri�re

    // Quand on va en marche arri�re, on allume les deux LEDS arri�res
    P8->OUT |= (BIT6 | BIT7);


    while(d < 250 || d > 450)
    {}    // On sort de cette boucle quand le test n'est plus vrai, c'est-�-dire quand distance > 250 && distance < 450
    //� ce moment on d�cide alors d'arr�ter les moteurs

    P3->OUT &=~ (BIT6|BIT7); // On arr�te les moteurs

    // On �teint toutes les LEDS quand on s'arr�te
    P2->OUT &=~BIT2;
    P8->OUT &=~(BIT6 | BIT7);

    while(1) //On bloque le programme dans une boucle infinie pour que le robot ne bouge plus
    {}
    //"""""""""""" FIN DU PROGRAMME """"""""""""//
}

void main(void)

{
    //Initilisation des horloges
    // Afficheur Graphique : on regle l'horloge SMCLK � 12 Mhz : MODOSC/2 = 12MHz
    CS_initClockSignal(CS_SMCLK, CS_MODOSC_SELECT, CS_CLOCK_DIVIDER_2);
    // Afficheur Graphique : on regle l'horloge MCLK � 3 Mhz
    CS_initClockSignal(CS_MCLK, CS_MODOSC_SELECT, CS_CLOCK_DIVIDER_8);
    // Afficheur Graphique : on regle l'horloge ACLK � 16 kHz
    CS_initClockSignal(CS_ACLK, CS_MODOSC_SELECT, CS_CLOCK_DIVIDER_2);


    ////////////////

    bumper(); //On initialise les bumpers
    capteur_lumiere(); //On initialise les capteurs de lumi�re
    moteur(); //On initialise les moteurs : on injecte des signaux PWM aux moteurs donc le robot va avancer
    led(); //On initialise les LEDs

    SysTick_enableModule();      //Activation module SysTick
    SysTick_setPeriod(30000);    // 100 interruptions par secondes
    SysTick_enableInterrupt();   //Autorisation des interruptions pour le Systick

    while(1) //Boucle infinie
    { //dans cette boucle, on va traiter les diff�rents cas pour que le robot puisse suivre la ligne

        //P7.3 et P7.4 sont les capteurs centraux. S'ils sont noirs, on avance tout droit
        if( ((etat_led & BIT3) == BIT3) && ((etat_led & BIT4) == BIT4) )
        {
            TIMER_A0->CCR[3] = 200; //On met des vitesse normales sur les moteurs
            TIMER_A0->CCR[4] = 200;
        }

        //si le capteur central droite voit du blanc et que le capteur central gauche voit du noir. On doit l�g�rement ralentir le moteur gauche pour que le robot se recentre
        if( ((etat_led & BIT3) == 0) && (etat_led & BIT4) == BIT4)
        { //CCR[4] --> signal du moteur gauche et CCR[3] --> signal du moteur droite.
            TIMER_A0->CCR[3] = 200;
            TIMER_A0->CCR[4] = 210; //On ralentit le moteur gauche
        }

        //si le capteur central gauche voit du blanc et que le capteur central droit voit du noir. On doit l�g�rement ralentir le moteur droit pour que le robot se recentre
        if( ((etat_led & BIT3) == BIT3) && (etat_led & BIT4) == 0)
        { //CCR[4] --> signal du moteur gauche et CCR[3] --> signal du moteur droite.
            TIMER_A0->CCR[3] = 210; //On ralentit le moteur droit
            TIMER_A0->CCR[4] = 200;
        }


        //Si le capteur p�riph�rique droit est activ�
       if( (etat_led & BIT0) == BIT0)
       {
           P3->OUT &=~ (BIT6|BIT7); //On arr�te les moteurs
           __delay_cycles(1500000); //On attend une demi seconde. Pendant ce temps, la fonction d'interruption du Systick va se lancer plusieurs fois. On va donc avoir une mise � jour de l'�tat des capteurs de ligne et de la variable etat_led
           P3->OUT |= (BIT6|BIT7); //On remet les moteurs en marche

           if( ((etat_led & BIT7) == BIT7) && ((etat_led & BIT3) == BIT3) && ((etat_led & BIT4) == BIT4) ) //Si le capteur p�riph�rique gauche ainsi que les capteurs centraux sont activ�s. le robot est donc sur une ligne noire horizontale
           {

               if(demi_tour_recent == 1) // Si on a fait un demi_tour r�cemment
               {
                   if(virage == 1) // Si notre dernier virage �tait � droite
                   {
                       quart_tour_droite(); //Alors on fait un virage � droite
                   }
                   if(virage == 2) // Si notre dernier virage �tait � gauche
                   {
                       quart_tour_gauche(); //Alors on fait un virage � gauche
                   }
                   virage = 0; //On remet � 0 ces deux variables pour indiquer que ce cas particulier a �t� trait�
                   demi_tour_recent = 0;
               }
               else //Dans ce cas, on a pas fait de demi_tour r�cemment
               {
                   virage = 0;
                   demi_tour_recent = 0;
                   quart_tour_droite(); //On d�cide de faire un virage � droite de mani�re arbitraire
               }
           }
           else //Si le capteur p�riph�rique gauche n'est pas activ�. Alors le robot voit seulement une ligne noire sur sa droite
           {
               virage = 1; //Mise � 1 de la variable virage pour indiquer que le dernier virage �tait � droite
               quart_tour_droite(); //Le robot fait un virage � droite
           }

       }

       //Si le capteur p�riph�rique gauche est activ�
       if( (etat_led & BIT7) == BIT7)
       {
           P3->OUT &=~ (BIT6|BIT7); //On arr�te les moteurs
           __delay_cycles(1500000); //On attend une demi seconde. Pendant ce temps, la fonction d'interruption du Systick va se lancer plusieurs fois. On va donc avoir une mise � jour de l'�tat des capteurs de ligne et de la variable etat_led
           P3->OUT |= (BIT6|BIT7); //On remet les moteurs en marche

           if( ((etat_led & BIT0) == BIT0) && ((etat_led & BIT3) == BIT3) && ((etat_led & BIT4) == BIT4)) //Si le capteur p�riph�rique droit ainsi que les capteurs centraux sont activ�s. le robot est donc sur une ligne noire horizontale
           {

               if(demi_tour_recent == 1) // Si on a fait un demi_tour r�cemment
               {
                   if(virage == 1) // Si notre dernier virage �tait � droite
                   {
                       quart_tour_droite(); //Alors on fait un virage � droite
                   }
                   if(virage == 2) // Si notre dernier virage �tait � gauche
                   {
                       quart_tour_gauche(); //Alors on fait un virage � gauche
                   }
                   virage = 0;
                   demi_tour_recent = 0; //On remet � 0 ces deux variables pour indiquer que ce cas particulier a �t� trait�
               }
               else //Dans ce cas, on a pas fait de demi_tour r�cemment
               {
                   virage = 0;
                   demi_tour_recent = 0;
                   quart_tour_droite(); //On d�cide de faire un virage � droite de mani�re arbitraire
               }
           }

           else //Si le capteur p�riph�rique droit n'est pas activ�. Alors le robot voit seulement une ligne noire sur sa gauche
           {
               virage = 2; //Mise � 2 de la variable virage pour indiquer que le dernier virage �tait � gauche
               quart_tour_gauche(); //Le robot fait un virage � gauche
           }
       }

        if(etat_led == 0b00000000) //Si tous les capteurs ne voient que du blanc
        {
            TIMER_A0->CCR[3] = 200; //On remet des vitesses normales
            TIMER_A0->CCR[4] = 200;

            if(nb_lancement_depuis_blanc > 150) //Si la fonction d'interruption du Systick s'est lanc� 150 fois d'affil� avec tous les capteurs qui voient du blanc
            { //Cela correspond � une dur�e de 1,5 secondes
                clafete(); //Alors on lance la fonction clafete() qui permet au robot de faire sa c�l�bration
            }
        }

    }
}
