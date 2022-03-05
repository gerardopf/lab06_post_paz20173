/*	
    Archivo:		lab06_lab_pgr.s
    Dispositivo:	PIC16F887
    Autor:		Gerardo Paz 20173
    Compilador:		pic-as (v2.30), MPLABX V6.00

    Programa:		Temporizadores 
    Hardware:		Led intermitente en RB0 (Timer2)
			Leds contador en puerto A (Timer1)

    Creado:			02/03/22
    Última modificación:	02/03/22	
*/

PROCESSOR 16F887
#include <xc.inc>

; CONFIG1
CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
CONFIG  PWRTE = OFF            ; Power-up Timer Enable bit (PWRT enabled)
CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
CONFIG  LVP = OFF              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)
    
 /*---------------- Macros ---------------*/
 reset_timer0 macro
    BANKSEL TMR0
    MOVLW   252		    // Cargamos N en W
    MOVWF   TMR0	    // Cargamos N en el TMR0, listo el retardo
    BCF	    T0IF	    // Limpiar bandera de interrupción
    ENDM
    
 reset_timer1 macro
    BANKSEL T1CON
    MOVLW   0x0B
    MOVWF   TMR1H	// B en parte alta
    MOVLW   0xDC	
    MOVWF   TMR1L	// DC en parte baja
    BCF	    TMR1IF	// Limpiar bandera
    ENDM   
    
 /*---------------- RESET ----------------*/
 PSECT resVect, class=CODE, abs, delta=2	
 ORG 00h					
 resVect:
       PAGESEL main
       GOTO    main

 /*--------------- Variables --------------*/ 
 PSECT udata_shr	   
 W_TEMP:	    DS  1	    
 STATUS_TEMP:	    DS  1
    
 CONTT1:	    DS  1
 CONTT2:	    DS	1
 SEGUNDOS:	    DS	1
 DECENAS:	    DS	1
    
 PSECT udata_bank0
 VALOR:	    DS	1
 SELECTOR:  DS	1	
 DISPLAY:   DS	2
   
 /*-------------- Interrupciones ---------------*/   
 PSECT intVect, class=CODE, abs, delta=2    
 ORG 04h
 
 push:
    MOVWF   W_TEMP	    // Movemos W en la temporal
    SWAPF   STATUS, W	    // Pasar el SWAP de STATUS a W
    MOVWF   STATUS_TEMP	    // Guardar STATUS SWAP en W	
    
 isr:    
    BANKSEL PORTA
    BTFSC   T0IF
    CALL    int_timer0
    
    BTFSC   TMR1IF	    // Revisar bandera Timer1
    CALL    int_timer1
    
    BTFSC   TMR2IF	    // revisar interrupción Timer2
    CALL    int_timer2
    
 pop:
    SWAPF   STATUS_TEMP, W  // Regresamos STATUS a su orden original y lo guaramos en W
    MOVWF   STATUS	    // Mover W a STATUS
    SWAPF   W_TEMP, F	    // Invertimos W_TEMP y se guarda en F
    SWAPF   W_TEMP, W	    // Volvemos a invertir W_TEMP para llevarlo a W
    RETFIE
      
 /*------------ Subrutinas de interrupción ------------*/
 int_timer1:
    reset_timer1
    //BANKSEL PORTA
    INCF    CONTT1	// incrementar contador
    MOVF    CONTT1, W
    SUBLW   2
    BTFSC   ZERO	// 2 * 500ms = 1seg
    GOTO    int_ledt1
    RETURN
    
 int_ledt1:
    //BANKSEL PORTA
    CLRF    CONTT1	// limpiar contador Timer1
    INCF    PORTA	// incrementar A
    INCF    SEGUNDOS
    RETURN
    
 int_timer2:
    BCF	    TMR2IF	// limpiar bandera
    INCF    CONTT2
    MOVF    CONTT2, W
    SUBLW   10		// verificar si el contador del timer2 es 2
    BTFSC   ZERO	// si es 2: 2*250ms = 500ms
    GOTO    int_ledt2	// a los 500ms enciende el led
    RETURN
    
 int_ledt2:
    ;BANKSEL PORTB
    CLRF    CONTT2	// limpiar contador Ttimer2
    INCF    PORTB	// incrementar RB0
    RETURN
    
 int_timer0:
    reset_timer0    
    CLRF    PORTE		    //Para que no exista contención    
    BTFSC   SELECTOR, 0
    GOTO    display1
  
 display0:
    MOVF    DISPLAY+1, W
    MOVWF   PORTC
    BSF	    PORTE, 0	    //Mostrar decenas
    GOTO    cambio_display
    
 display1:  
    MOVF    DISPLAY, W
    MOVWF   PORTC
    BSF	    PORTE, 1	    //Mostrar unidades
    GOTO    cambio_display   
    
 cambio_display:
    MOVLW   1
    XORWF   SELECTOR, F	    // 
    RETURN
    
 /*----------------- COONFIGURACIÓN uC --------------------*/
 PSECT code, delta=2, abs	
 ORG 100h			// Dirección 100% seguro de que ya pasó el reseteo
 
 main:
    CALL    setup_io
    CALL    setup_int
    CALL    setup_clk
    CALL    setup_timer0
    CALL    setup_timer1
    CALL    setup_timer2
    BANKSEL PORTA
    
 loop:
    CALL    division
    CALL    preparar_7seg
    GOTO    loop
 
 /*----------------- Subrutinas -----------------*/
 division:
    MOVF    SEGUNDOS, W
    SUBLW   10		//10 - W
    BTFSC   ZERO
    CALL    inc_decenas
    
    RETURN
    
 inc_decenas:
    CLRF    SEGUNDOS
    INCF    DECENAS
    
    MOVF    DECENAS, W
    SUBLW   6		//6 - W
    BTFSC   ZERO
    CLRF    DECENAS
    
    RETURN
    
 preparar_7seg:
    MOVF    SEGUNDOS, W
    CALL    tabla	    //Codificación
    MOVWF   DISPLAY	    //Preparar Display con segundos
    
    MOVF    DECENAS, W
    CALL    tabla
    MOVWF   DISPLAY+1	    //Display preparado con decenas
    RETURN
    
 setup_io:
    BANKSEL ANSEL
    CLRF    ANSEL
    CLRF    ANSELH  // i/o digitales
    
    BANKSEL TRISA
    CLRF    TRISA	// Puerto A out
    CLRF    TRISC	// Puerto C out
    
    BCF	    TRISB, 0	// RB0 out
    BCF	    TRISE, 0	// RE0 out
    BCF	    TRISE, 1	// RE1 out
    
    BANKSEL PORTA
    CLRF    PORTA   // Limpiar puertos
    CLRF    PORTB
    CLRF    PORTC
    CLRF    PORTE
    
    CLRF    CONTT1    // limpiar contadores
    CLRF    CONTT2
    RETURN
    
 setup_int:
    BANKSEL TRISA
    BSF	    TMR1IE  // interrupción Timer1 ON
    BSF	    TMR2IE  // interrupción Timer2 ON 
    
    BANKSEL PORTA
    BSF	    GIE	    // interrupciones globales ON
    BSF	    PEIE    // interrupciones de periféricos ON
    BSF	    T0IE    // interrupciones del Timer1 ON
    
    BCF	    T0IF    // limpiar bandera Timer0
    BCF	    TMR1IF  // limpiar bandera Timer1
    BCF	    TMR2IF  // limpiar bandera Timer2
    RETURN
    
 setup_clk:
    BANKSEL OSCCON
    BSF	    SCS		//Activar oscilador interno
    
    BSF	IRCF2		// 4 MHz(100)	
    BSF IRCF1		
    BCF IRCF0		
    RETURN
    
 setup_timer0:
    /*
	retraso de 1 ms
	Fosc = 4MHz
	PS = 256
	T = 4 * Tosc * TMR0 * PS
	Tosc = 1/Fosc
	TMR0 = T * Fosc / (4 * PS) = 256-N
	N = 256 - (1ms * 4MHz / (4 * 256))
	N = 252 aprox
    */
    BANKSEL OPTION_REG
    
    BCF T0CS	    // Funcionar como temporizador (usando reloj interno)
    BCF PSA	    // Asignar Prescaler al Timer0
    
    BSF PS2	    // Prescaler de 1:256
    BSF PS1	    
    BSF PS0	    
   
    BANKSEL PORTA
    reset_timer0    // Asignar retardo
    RETURN  

 setup_timer1:
    /*
	Desbordamiento 500 ms
	N = 65536 - (td / (Prescaler * ti))
	td = 500 ms
	ti = 4/4 MHz
	N = 3036 = 0xBDC
    */
    BANKSEL T1CON
    BCF	TMR1GE	    // Siempre va a estar contando
    
    BSF	T1CKPS1	    //Prescaler :8
    BSF	T1CKPS0
    
    BCF	T1OSCEN		// Low power oscillator apagado
    BCF	TMR1CS		// reloj interno
    
    BSF	TMR1ON		// Timer 1 encendido
    reset_timer1	// Asignar valores iniciales 
    RETURN
    
 setup_timer2:
    /*
	Desbordamiento 50ms
	Ttmr2if = pre * PR2 * post * 4/Fosc
	PR2 = Ttmr2if / (pre * post * 4/Fosc)
	PR2 = 195
	
	Ttmr2if = Tiempo para TMR2 = PR2
	PR2 = valor del registro PR2
    */
    BANKSEL PORTA

    BSF	TOUTPS3	    // Postscaler 1:16
    BSF	TOUTPS2
    BSF	TOUTPS1
    BSF	TOUTPS0

    BSF	T2CKPS1	    // Prescaler 1:16
    BSF	T2CKPS0
    
    BSF	TMR2ON  //Timer2 ON
    
    BANKSEL TRISA
    MOVLW   195	    
    MOVWF   PR2	    // mover 244 al PR2
    CLRF    TMR2    // limpiar el valor del timer2
    BANKSEL PORTA   // Cambio al puerto de las banderas
    BCF	    TMR2IF  // limpiar bandera timer2  
    RETURN

 ORG 200h   
 tabla:
    CLRF    PCLATH
    BSF	    PCLATH, 1	//LATH en posición 1
    
    ANDLW   0x0F	//No sobrepasar el tamaño de la tabla (<16)
    ADDWF   PCL		//PC = PCLATH + PCL 
    
    RETLW   00111111B	//0
    RETLW   00000110B	//1
    RETLW   01011011B	//2
    RETLW   01001111B	//3
    RETLW   01100110B	//4
    RETLW   01101101B	//5
    RETLW   01111101B	//6
    RETLW   00000111B	//7
    RETLW   01111111B	//8
    RETLW   01101111B	//9
    RETLW   01110111B	//A
    RETLW   01111100B	//B
    RETLW   00111001B	//C
    RETLW   01011110B	//D
    RETLW   01111001B	//E
    RETLW   01110001B	//F

END