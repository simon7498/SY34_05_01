/**
 * TP SY34 
 *
 * Nom binôme :
 *
 * Version :
 *
 */




// PIC18F46J50 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config WDTEN = OFF       // Watchdog Timer (Enabled)
#pragma config PLLDIV = 2       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset (Enabled)
#pragma config XINST = OFF       // Extended Instruction Set (Enabled)

// CONFIG1H
#pragma config CPUDIV = OSC3_PLL3    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
#pragma config OSC = HSPLL      // Oscillator (HS+PLL, USB-HS+PLL)
#pragma config T1DIG = ON       // T1OSCEN Enforcement (Secondary Oscillator clock source may be selected)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator (High-power operation)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor (Enabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Postscaler (1:32768)

// CONFIG3L
#pragma config DSWDTOSC = INTOSCREF// DSWDT Clock Select (DSWDT uses INTRC)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = ON     // Deep Sleep BOR (Enabled)
#pragma config DSWDTEN = ON     // Deep Sleep Watchdog Timer (Enabled)
#pragma config DSWDTPS = G2     // Deep Sleep Watchdog Postscaler (1:2,147,483,648 (25.7 days))

// CONFIG3H
#pragma config IOL1WAY = ON     // IOLOCK One-Way Set Enable bit (The IOLOCK bit (PPSCON<0>) can be set once)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

// CONFIG4L
#pragma config WPFP = PAGE_63   // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 63)
#pragma config WPEND = PAGE_WPFP// Write/Erase Protect Region Select (valid when WPDIS = 0) (Page WPFP<5:0> through Configuration Words erase/write protected)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region (Configuration Words page not erase/write-protected)

// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<5:0>/WPEND region ignored)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>




/************************ HEADERS ****************************************/
#include "VT100.h"
#include <string.h>
#include "system.h"
#include "system_config.h"
#include "miwi/miwi_api.h"
#include <stdarg.h>
#include "lcd.h"
/************************** PROTOTYPES ************************************/





/************************** VARIABLES ************************************/
        
extern API_UINT16_UNION                 myPANID;        						// mon PANID
extern uint8_t                          myLongAddress[MY_ADDRESS_LENGTH];		// mon adresse IEEE
extern API_UINT16_UNION                 myShortAddress;                     	// mon adresse logique
extern ACTIVE_SCAN_RESULT               ActiveScanResults[ACTIVE_SCAN_RESULT_SIZE];		// table des actives scan

//#define NO_TERM
#define PSEUDO_MAX_LENGTH  8
#define MY_CHANNEL         11

#define DATA    0xAA
#define CMD     0x55

extern RECEIVED_MESSAGE  rxMessage;

char myPseudo[PSEUDO_MAX_LENGTH+1];
char destinataire[] = "0x0100";
uint8_t 							TX_Index_Unicast;					// index pour la transmission unicast
int nbPushRB0 = 0;	
int nbPushRB1 = 0;	
int nbPushRB2 = 0;

    
/****************************************************/
/*                   Prototypes                     */
/****************************************************/

void getPseudo(char *);
void RX(void);
void TX(void);
void broadcastData(char *,...);
void initChat(void);
void initNwk(void);
void sendMeassageBroadcast(char * pseudo,int nbPush);
void sendMeassageUnicast(char * pseudo,int nbPush,char  * destinataire);



void main (void)
{

    // Initialisation Carte
    SYSTEM_Initialize();	

    // Initialisation Uart
    uartInitialize();

    // Identification 
    initChat();

    // Connexion réseau
    initNwk();

    while(1){
      RX();
      TX();
    }

}


/**
 * Initialisation du chat : message d'invite et acquisition du pseudo
 */
void initChat(void){
#ifdef NO_TERM
    LCDBacklightON();
    memcpy(myPseudo, "bot", strlen("bot")+1);
    LCDDisplay((char *)"Pseudo : bot",0, true);
#else
    vT100ClearScreen();
    vT100SetCursorPos(0,0);
    uartPrint("Saisissez votre pseudo : ");
    getPseudo(myPseudo);	
    uartPrint("\r\n Bonjour : ");
    uartPrint(myPseudo);
    uartPrint("!\r\n");
#endif    
}


void initNwk(){
uint8_t respondingDevices;
bool found;    
uint8_t index;

MiApp_ProtocolInit(false);


if(MiApp_SetChannel(MY_CHANNEL) == false){			// Réglage canal 11
    #ifdef NO_TERM
        LCDDisplay((char *)"Err selection canal",0, true);
    #else
        uartPrint("Erreur : selection de canal");
    #endif
    goto fin;
}

respondingDevices = MiApp_SearchConnection(10,1L<<MY_CHANNEL);
found = false;
if(respondingDevices !=0){
    for(index = 0; index< respondingDevices;index++){
        if(found = (ActiveScanResults[index].PANID.Val == MY_PAN_ID))
            break;
    }
}
// found existing PAN controller
if(found){  
    MiApp_ConnectionMode(ENABLE_ACTIVE_SCAN_RSP);
    if(MiApp_EstablishConnection(index, CONN_MODE_DIRECT)==0xFF){
    #ifdef NO_TERM
        LCDDisplay((char *)"Err connexion refusee",0, true);
    #else
        uartPrint("Erreur : connexion refusee");
    #endif
        goto fin;
    }
    #ifdef NO_TERM
    LCDDisplay((char *)"Connexion OK",0, true);
    #else
        uartPrint("Connexion reussie sur PAN existant\n\r");
    #endif
// nobody    
}else{
    MiApp_ConnectionMode(ENABLE_ALL_CONN);
    if(!MiApp_StartConnection(START_CONN_DIRECT,0,0)){
    #ifdef NO_TERM
        LCDDisplay((char *)"Er : conn. refusee",0, true);
    #else
        uartPrint("Erreur : creation refusee");    
    #endif
        goto fin;
    }
    #ifdef NO_TERM
        LCDDisplay((char *)"Nouveau PAN !",0, true);
    #else
    uartPrint("Creation d'un nouveau PAN\n\r");
    #endif
}
#ifdef NO_TERM
    sprintf(LCDText,"Adresse : 0X%04x",myShortAddress);
    sprintf(&LCDText[16],"sur PAN : 0X%04x",MY_PAN_ID);
    LCD_Update();
#else
    uartPrint("Votre adresse est : 0x");   
    uartHexaPrint((uint8_t *)&myShortAddress,2);
    uartPrint("\r\n");  
#endif

    return;
    fin: while(1);
}

/**
 * Récupération du pseudo de l'utilisateur
 * @param pseudo : une table de 9 octets contenant le pseudo terminé par un 0 
 */
void getPseudo(char * pseudo){
    int i = 0;
    do{
    if(uartIsChar()){
        pseudo[i++] = uartRead(); 
    }
    }while((pseudo[i-1]!=0x0D)||(i>=PSEUDO_MAX_LENGTH));
    pseudo[i-1] = 0;
    
}


/**
 * Gestion des messages entrants
 */
void RX(void){
    if(MiApp_MessageAvailable())
    {
        uartPrint(rxMessage.Payload);
        uartPrint("\n\r");
		MiApp_DiscardMessage();     
    }
}

/**
 * Gestion des messages sortants
 */
void TX(void){
    
    if(!PORTBbits.RB2) {
		nbPushRB2++;
		sendMeassageBroadcast(myPseudo,nbPushRB2);
	}

	if(!PORTBbits.RB0) {
		nbPushRB0++;
        if(myShortAddress.v[1] == 0) {
            destinataire[3] = '1';
        }
        else if(myShortAddress.v[1] == 1) {
            destinataire[3] = '0';
        }
        else
            destinataire[3] = '2';
        
		sendMeassageUnicast(myPseudo,nbPushRB0,destinataire);
	}
   
}




// modifierTX_BUFFER_SIZE

void sendMeassageBroadcast(char * pseudo,int nbPush)
{
	MiApp_FlushTx();					// vide la pile Tx
	char chaine1[] = "Pseudo : ";
	char chaine2[] = ", message broadcast ";
	int i = 0;
	while(chaine1[i] != 0) {
		MiApp_WriteData(chaine1[i]);		// remplissage de la pile d'émission avec "Pseudo : "
		i++;
	}
	i = 0;
	while(pseudo[i] != 0) {
		MiApp_WriteData(pseudo[i]);		// remplissage de la pile d'émission avec le pseudo
		i++;
	}
	i = 0;
	while(chaine2[i] != 0) {
		MiApp_WriteData(chaine2[i]);		// remplissage de la pile d'émission avec ", message broadcast "
		i++;
	}
	char nbPushChar[] = "00";
    nbPushChar[0]+=nbPush/10;
    nbPushChar[1]+=nbPush%10;
	MiApp_WriteData(nbPushChar[0]);			// remplissage de la pile d'émission le nb de push
	MiApp_WriteData(nbPushChar[1]);
    MiApp_WriteData("\0");
	
	if(!MiApp_BroadcastPacket(false))
		uartPrint("\n\rErreur emission message broadast\n");
    else uartPrint("\n\rMessage broadast envoye !\n");
}


void sendMeassageUnicast(char * pseudo,int nbPush,char  * destinataire) 
{
	
	MiApp_FlushTx();					// vide la pile Tx
	char chaine1[] = "Pseudo : ";
	char chaine2[] = ", message unicast ";
	char chaine3[] = " vers ";
	int i = 0;
	while(chaine1[i] != 0) {
		MiApp_WriteData(chaine1[i]);		// remplissage de la pile d'émission avec "Pseudo : "
		i++;
	}
	i = 0;
	while(pseudo[i] != 0) {
		MiApp_WriteData(pseudo[i]);		// remplissage de la pile d'émission avec le pseudo
		i++;
	}
	i = 0;
	while(chaine2[i] != 0) {
		MiApp_WriteData(chaine2[i]);		// remplissage de la pile d'émission avec ", message broadcast "
		i++;
	}
	char nbPushChar[] = "00";
    nbPushChar[0]+=nbPush/10;
    nbPushChar[1]+=nbPush%10;
	MiApp_WriteData(nbPushChar[0]);			// remplissage de la pile d'émission le nb de push
	MiApp_WriteData(nbPushChar[1]);

	i = 0;
	while(chaine3[i] != 0) {
		MiApp_WriteData(chaine3[i]);		// remplissage de la pile d'émission avec " vers "
		i++;
	}
	i = 0;
	while(destinataire[i] != 0) {
		MiApp_WriteData(destinataire[i]);		// remplissage de la pile d'émission avec le num de destinataire
		i++;
	}
	MiApp_WriteData("\0");
	if(!MiApp_UnicastConnection(TX_Index_Unicast,false))
		uartPrint("\n\rErreur emission message unicast\n");
	
}