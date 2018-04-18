
#define _XTAL_FREQ 4000000


#define RS RD2    // LCD
#define EN RD3
#define D4 RD4
#define D5 RD5
#define D6 RD6
#define D7 RD7   // LCD
#define t 2000   //delay time for push actuator.


#include <xc.h>
#include "lcd.h";
#include <stdlib.h>
void place_notice(char num);
void remove_notice(int var);
void stepper(char motor,char direction,int size);
void down(void);

// BEGIN CONFIG
#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = ON // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)
//END CONFIG










void main()
{

  
  CMCON = 0x07; // To turn off comparators
  ADCON1 = 0x06; // To turn off analog to digital converters
  TRISB0 = 1;  //B0 as input pin.
  TRISB1 = 0; //  B1 as output port
  TRISB2 = 0;
  TRISB4 = 0;
  TRISB5 = 0;
  TRISB6 = 0;
  TRISB7 = 0;


  TRISE = 0; //LATEST TEST //PORT E AS OUTPUT.
  RE0=0;
  RE1=0;
  RE2=0;
  TRISA0 = 1; // PORT A  A-0 pin as input pin.  *** THIS IS THE IR INPUT***
  TRISA1 = 0; // PORT A  A-1 pin as output pin.  ***THIS IS PRINTER CONTROLLING OUTPUT PIN***
  TRISA2 = 1; // PORT A  A-2 PIN AS INPUT PIN.   ***THIS IS A SELECTING PUSH BUTTON IN LCD***
  TRISA3 = 1; // PORT A  A-3 PIN AS INPUT PIN.   ***THIS IS A confirming PUSH BUTTON IN LCD***
  //TRISE0 = 0; // FOR TEST ONLY PORT A A-4 AS OUTPUT PIN FOR LED.*******************************************************************
  //PORTB = 0x0F;
  //INSTED OF ABOVE INSTRUCTION
  RB1=0;
  RB2=0;
  RB4=0;    /*STEPPER*/
  RB5=0;
  RB6=0;
  RB7=0;    /*STEPPER END*/

  RA1 = 0; //initialize printer con. output pin as 0....

  TRISC0 = 1; //C0 PIN AS INPUT PIN.


  int n_buff[]= {0,0,0,0,0,0,1};  // NOTICE BUFFER.******there is a static ON bit at the end.**********
  int s_buff[]= {1,2,3,4,5,6};  // SEQUENCE BUFFER.
  int count = 0;
  char s[20];
  int buff; ////

        __delay_ms(1000);
        TRISD = 0x00;
        Lcd_Init();
       // Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Write_String("SELECT NOTICE");
        



  do
  {

    if(RA0 == 1)   // RA0 = IR INPUT   // POLL -1-
    {
        __delay_ms(300);

        if(RA0 == 1)
        {
        __delay_ms(400); // because printing may be still happening...
        RA1 = 1;   // Disable the printer temporerily.
        

        char x=0,y=0;
        while(y==0)     // CHECKING THE NOTICE BUFFER.
        {

            if( n_buff[x] == 0)
            { 
                place_notice(x);
                n_buff[x] = 1;    // update the n_buffer.
                y = 1;
            }
           // x++;  not used here now.

             if (x == 6)  // THIS CONDITION HAPPENS IF THE NOTICE BOARD OVERFLOW.
            {

                remove_notice(s_buff[0]);
                /*char buff;*/                        // HERE THE SEQUENCE BUFFER RE ARRENGING GOING ON.
                buff = s_buff[0];
                for(char z=0;z<5;z++)
                {
                    s_buff[z] = s_buff[z+1];
                }
                s_buff[5] = buff ;


               /* char holder; */          // HERE THE NOTICE BUFFER RE ARRENGING GOING ON.
                /*holder = s_buff[0];*/    //this is a wrong use of variable.
                n_buff[buff-1] = 0;   // edited [holder -1]
                x = -1;                 //get back x to -1 otherwice x increment greater than 6.

            }
            x++;  // increment.
        }

        RA1 = 0; // ENABLE PRINTER
        }
    }
   
/////////////////////////////////////// LCD ////////////////////

 
       
     if(RA2 == 0)       // DON'T FORGET DE BOUNCE    RA2----notice selecting button.   //POLL -2-
        {
            count++;
            Lcd_Set_Cursor(2,1);
             itoa(s, count, 10);   //count display
             Lcd_Write_String(s);  //count display
             __delay_ms(300);
             /*Lcd_Clear();*/      //CHECK THE SUITABILITY.

            if(count == 6)
                {
                    count = 0;
                }
        }
////////////////////////////////REMOVE CONFIRM -START-////////////
            if(RA3 == 0)              //  RA3 ----- REMOVE CONFIRMING BUTTON. // POLL -3-
            {
                __delay_ms(200);    //to get rid of debounce...
                
                remove_notice(count);   // removing notice by the user.
                n_buff[count-1]=0;        // update the n_buff

                /******Update s_buffer[]*******/
                char p=0,q=0,n;
                while(p == 0)
                {
                    if(s_buff[q] == count)
                    {

                        p = 1;

                    }
                q++;
                }
                q--;
                for(n=q;n<5;n++)
                {
                    s_buff[n] = s_buff[n+1];

                }
                s_buff[5] = count;

               /***s_buffer[] update END***/



                





            }

////////////////////////////////REMOVE CONFIRM -END/////////////////

////////////////////////////////////// LCD ///////////////////

 }while(1);



}


void place_notice(char num) // argument is char type.
{
    if(num == 0)
    {

        RE2 = 1;          //GRIPPER ON (grip the paper.)                       /*NOTICE*/
        stepper(1,1,330);  //GOING UP                                  ////////////////////////////
                                                                      //   RE2-----GRIP         //
        stepper(2,1,3072);   //GOING VERTICALLY                          //   RE1-----PUSH         //
       
        ////*****GRIPPER START*****

        RE1 = 1;          //PUSH ON                                   ////////////////////////////
        RE0 = 0;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

        RE1 = 0;          //PUSH OFF
        RE0 = 0;
        RE2 = 0;          //GRIPPER OFF (relese the paper.)
        
        RE1 = 0;        //PUSH BACK
        RE0 = 1;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PULL.

        RE1 = 0;      //PULL OFF
        RE0 = 0;

        /////******GRIPPER END****
        stepper(2,2,3072);   //GOING BACK VERTICALLY
        //stepper(1,2,100);  //GOING DOWN TO NORMAL POSITION.
        down();
    }

    if(num == 1)
    {
        
        RE2 = 1;          //GRIPPER ON (grip the paper.)                       /*NOTICE*/
        stepper(1,1,330);  //GOING UP                                  ////////////////////////////
                                                                 //   RE2-----GRIP         //
                                                                 //   RE1-----PUSH         //
       
        ////*****GRIPPER START*****

        RE1 = 1;          //PUSH ON                                   ////////////////////////////
        RE0 = 0;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

        RE1 = 0;          //PUSH OFF
        RE0 = 0;
        RE2 = 0;          //GRIPPER OFF (relese the paper.)
        
        RE1 = 0;        //PUSH BACK
        RE0 = 1;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PULL.

        RE1 = 0;      //PULL OFF
        RE0 = 0;

        /////******GRIPPER END****
        down();


    }

    if(num == 2)
    {

        RE2 = 1;          //GRIPPER ON (grip the paper.)                       /*NOTICE*/
        stepper(1,1,330);  //GOING UP                                  ////////////////////////////
                                                                      //   RE2-----GRIP         //
        stepper(2,2,3072);   //GOING VERTICALLY                          //   RE1-----PUSH         //

        ////*****GRIPPER START*****

        RE1 = 1;          //PUSH ON                                   ////////////////////////////
        RE0 = 0;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

        RE1 = 0;          //PUSH OFF
        RE0 = 0;
        RE2 = 0;          //GRIPPER OFF (relese the paper.)

        RE1 = 0;        //PUSH BACK
        RE0 = 1;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PULL.

        RE1 = 0;      //PULL OFF
        RE0 = 0;

        /////******GRIPPER END****
        stepper(2,1,3072);   //GOING BACK VERTICALLY
        down();


    }

    if(num == 3)
    {
        RE2 = 1;          //GRIPPER ON (grip the paper.)                       /*NOTICE*/
        stepper(1,1,170);  //GOING UP                                  ////////////////////////////
                                                                      //   RE2-----GRIP         //
        stepper(2,1,3072);   //GOING VERTICALLY                          //   RE1-----PUSH         //

        ////*****GRIPPER START*****

        RE1 = 1;          //PUSH ON                                   ////////////////////////////
        RE0 = 0;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

        RE1 = 0;          //PUSH OFF
        RE0 = 0;
        RE2 = 0;          //GRIPPER OFF (relese the paper.)

        RE1 = 0;        //PUSH BACK
        RE0 = 1;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PULL.

        RE1 = 0;      //PULL OFF
        RE0 = 0;

        /////******GRIPPER END****
        stepper(2,2,3072);   //GOING BACK VERTICALLY
        down();


    }

    if(num == 4)
    {
        RE2 = 1;          //GRIPPER ON (grip the paper.)                       /*NOTICE*/
        stepper(1,1,170);  //GOING UP                                  ////////////////////////////
                                                                      //   RE2-----GRIP         //
                                                                       //   RE1-----PUSH         //

        ////*****GRIPPER START*****

        RE1 = 1;          //PUSH ON                                   ////////////////////////////
        RE0 = 0;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

        RE1 = 0;          //PUSH OFF
        RE0 = 0;
        RE2 = 0;          //GRIPPER OFF (relese the paper.)

        RE1 = 0;        //PUSH BACK
        RE0 = 1;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PULL.

        RE1 = 0;      //PULL OFF
        RE0 = 0;

        /////******GRIPPER END****
        down();


    }

    if(num == 5)
    {
        RE2 = 1;          //GRIPPER ON (grip the paper.)                       /*NOTICE*/
        stepper(1,1,170);  //GOING UP                                  ////////////////////////////
                                                                      //   RE2-----GRIP         //
        stepper(2,2,3072);   //GOING VERTICALLY                          //   RE1-----PUSH         //

        ////*****GRIPPER START*****

        RE1 = 1;          //PUSH ON                                   ////////////////////////////
        RE0 = 0;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

        RE1 = 0;          //PUSH OFF
        RE0 = 0;
        RE2 = 0;          //GRIPPER OFF (relese the paper.)

        RE1 = 0;        //PUSH BACK
        RE0 = 1;
        __delay_ms(t);  //GIVE TIME FOR ACTUATOR PULL.

        RE1 = 0;      //PULL OFF
        RE0 = 0;

        /////******GRIPPER END****
        stepper(2,1,3072);   //GOING BACK VERTICALLY
        //stepper(1,2,100);  //GOING DOWN TO NORMAL POSITION.
        down();


    }

    // here at last don't forget to set all pins of stepper to low state to prevent from burning the coils.
}

void remove_notice(int var)  // don't forget argument is char type.
{

     if(var == 1)
    {
         stepper(1,1,330); //GO TO THE SELECTED NOTICE.
         stepper(2,1,3072);

         RE1 = 1;          //PUSH ON                                   ////////////////////////////
         RE0 = 0;
         __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(1,2,15); //GO DOWN BIT.
      
         RE2 = 1;          //GRIPPER ON (grip the paper.)

         stepper(1,1,15); //GO UP BIT.

         RE1 = 0;        //PUSH BACK
         RE0 = 1;
         __delay_ms(t);


         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         down();   //go down
         
         RE2 = 0;          //GRIPPER OFF (release the paper.)
         __delay_ms(300);
         stepper(1,1,20); // go up bit
         stepper(2,2,3072);
         down();  // go down
    }

    if(var == 2)
    {

         stepper(1,1,330); //GO TO THE SELECTED NOTICE.


         RE1 = 1;          //PUSH ON                                   ////////////////////////////
         RE0 = 0;
         __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(1,2,15); //GO DOWN BIT.

         RE2 = 1;          //GRIPPER ON (grip the paper.)

         stepper(1,1,15); //GO UP BIT.

         RE1 = 0;        //PUSH BACK
         RE0 = 1;
         __delay_ms(t);


         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(2,1,3072);
         down();   //go down
         RE2 = 0;          //GRIPPER OFF (relese the paper.)
         __delay_ms(300);
         stepper(1,1,20);
         stepper(2,2,3072);
         down();
    }

    if(var == 3)
    {
         stepper(1,1,330); //GO TO THE SELECTED NOTICE.
         stepper(2,2,3072);

         RE1 = 1;          //PUSH ON                                   ////////////////////////////
         RE0 = 0;
         __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(1,2,15); //GO DOWN BIT.

         RE2 = 1;          //GRIPPER ON (grip the paper.)

         stepper(1,1,15); //GO UP BIT.

         RE1 = 0;        //PUSH BACK
         RE0 = 1;
         __delay_ms(t);


         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(2,1,6144);
         down();   //go down

         RE2 = 0;          //GRIPPER OFF (release the paper.)
         __delay_ms(300);
         stepper(1,1,20);
         stepper(2,2,3072);
         down();
    }

    if(var == 4)
    {
         stepper(1,1,170); //GO TO THE SELECTED NOTICE.
         stepper(2,1,3072);

         RE1 = 1;          //PUSH ON                                   ////////////////////////////
         RE0 = 0;
         __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(1,2,15); //GO DOWN BIT.

         RE2 = 1;          //GRIPPER ON (grip the paper.)

         stepper(1,1,15); //GO UP BIT.

         RE1 = 0;        //PUSH BACK
         RE0 = 1;
         __delay_ms(t);


         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         down();   //go down

         RE2 = 0;          //GRIPPER OFF (release the paper.)
         __delay_ms(300);
         stepper(1,1,20);
         stepper(2,2,3072);
         down();

    }

    if(var == 5)
    {
         stepper(1,1,170); //GO TO THE SELECTED NOTICE.

         RE1 = 1;          //PUSH ON                                   ////////////////////////////
         RE0 = 0;
         __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(1,2,15); //GO DOWN BIT.

         RE2 = 1;          //GRIPPER ON (grip the paper.)

         stepper(1,1,15); //GO UP BIT.

         RE1 = 0;        //PUSH BACK
         RE0 = 1;
         __delay_ms(t);


         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(2,1,3072);

         down();   //go down

         RE2 = 0;          //GRIPPER OFF (release the paper.)
         __delay_ms(300);
         stepper(1,1,20);
         stepper(2,2,3072);
         down();

    }

    if(var == 6)
    {
         stepper(1,1,170); //GO TO THE SELECTED NOTICE.
         stepper(2,2,3072);

         RE1 = 1;          //PUSH ON                                   ////////////////////////////
         RE0 = 0;
         __delay_ms(t);  //GIVE TIME FOR ACTUATOR PUSH.

         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(1,2,15); //GO DOWN BIT.

         RE2 = 1;          //GRIPPER ON (grip the paper.)

         stepper(1,1,15); //GO UP BIT.

         RE1 = 0;        //PUSH BACK
         RE0 = 1;
         __delay_ms(t);


         RE1 = 0;          //PUSH OFF
         RE0 = 0;

         stepper(2,1,6144);
         down();   //go down

         RE2 = 0;          //GRIPPER OFF (release the paper.)
         __delay_ms(300);
         stepper(1,1,20);
         stepper(2,2,3072);
         down();


    }
                               // here at last don't forget to set all pins of stepper to low state prevent from burning the coils.

}

void stepper(char motor,char direction,int size)   //Function for the controll of both  motors.
{
    if(motor == 1)                          ////////////FOR GEAR MOTOR 1/////////////
    {                                                              /*NOTICE*/
    if (direction == 1)                            ///////////////////////////////////////////
    {                                              // Motor (1) = Vertical movement motor.  //
        int count=0;                                           // Motor (2) = Horizontal movement motor.//
        char state,last_state=0;                                          // Direction (1) = Up or Right hand side.//
      
        while((2*size)>count)
        {                                           
      
         RB1 = 1; //MOTOR ON                                        // Direction (2) = Down or Left hand side//
         RB2 =0;                                             ///////////////////////////////////////////
         state = RB0;

         if(state != last_state)
         {

             count++;
             last_state = state;

         }

        } 

        RB1 = 0; //MOTOR OFF
        RB2 =0;
         
         
    }
    else
    {


       int count1=0;
        char state1,last_state1=0;

        while((2*size)>count1)
        {

         RB1 = 0; //MOTOR ON
         RB2 = 1;
         state1 = RB0;

         if(state1 != last_state1)
         {

             count1++;
             last_state1 = state1;

         }

        }

        RB1 = 0; //MOTOR OFF
        RB2 = 0;
        
        



    }

    }


    else                     //////////////// FOR STEPPER MOTOR 2//////////////////////////
    {
    if (direction == 1)
    {

        for(int x=0;x<size;x++)
        {

            RB4=1;
            RB5=1;
            RB6=0;
            RB7=0;
            __delay_ms(2);
            RB4=0;
            RB5=1;
            RB6=1;
            RB7=0;
            __delay_ms(2);
            RB4=0;
            RB5=0;
            RB6=1;
            RB7=1;
            __delay_ms(2);
            RB4=1;
            RB5=0;
            RB6=0;
            RB7=1;
            __delay_ms(2);

        }
        RB4=0;
        RB7=0;
    }
    else
    {

    for(int x=0;x<size;x++)
        {

             RB4=1;
            RB5=0;
            RB6=0;
            RB7=1;
            __delay_ms(2);
            RB4=0;
            RB5=0;
            RB6=1;
            RB7=1;
            __delay_ms(2);
            RB4=0;
            RB5=1;
            RB6=1;
            RB7=0;
            __delay_ms(2);
            RB4=1;
            RB5=1;
            RB6=0;
            RB7=0;
            __delay_ms(2);

        }
            RB4=0;
            RB5=0;

    }

    }


}


void down(void)
{
    while(RC0 == 1)
    {
        RB1 = 0;
        RB2 = 1;
    }


    RB1 = 0;
    RB2 = 0;
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    