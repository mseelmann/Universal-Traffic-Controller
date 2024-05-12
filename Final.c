/* Final Project
 *
 * The Traffic Light setup is connected to the Nucleo-FF446RE board as follows:
 * PB0-PB4 for NSG, NSY, NSR, EWG, EWY, and PB6 for EWR
 *
 * The 7-segment display is connected to the board as follows:
 * PA4 to PA10
 * 
 * The LCD controller is connected to the board as follows:
 * PC4-PC7 for LCD D0-D7, respectively.
 * PB5 for LCD R/S
 * LCD R/W is tied to ground
 * PB7 for LCD EN
 *
 * PC13 for push button interrupt
 * PC0-PC3 for switch input
 */

#include "stm32f4xx.h"

/* LCD */
#define RS 0x20     /* PB5 mask for reg select */
#define EN 0x80     /* PB7 mask for enable */
void LCD_nibble_write(char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);
void display(int state);

/* Urban with Crosswalk Interrupt */
void urban_init(void);
void pedestrian(void);
int pressed = 0; /*interrupt variable */

/* Rural FSM */
int currState = 0;
int input = 0;

void delayMs(int n);

int main(void) {
		RCC->AHB1ENR |= 7; /* enable GPIOA, GPIOB, and GPIOC clocks */
		
		/* initialize LCD controller */
    LCD_init();
		
		urban_init();
		
		/* Switch */
		GPIOC->PUPDR = 0x00000055; /* enable internal pull-up resistors for pins C0-C3 */

    while(1) {
		/* RURAL */
			while((GPIOC->IDR & 0x03)==0x00) {
					display(10);
					
					/* FSM Setup */
					struct State {
						int output;
						int delay;
						int nextState[4];
					};
			
					typedef const struct State rural;
						#define S0 0
						#define S1 1
						#define S2 2
						#define S3 3
			
					rural FSM[4] = {
						{0x41,3000,{S0,S0,S1,S1}},      /* state 0: NSG/EWR, 3s */
						{0x42,1500,{S2,S2,S2,S2}},    /* state 1:  NSY/EWR, 1.5s */
						{0x0C,3000,{S2,S3,S2,S3}},      /* state 2: NSR/EWG, 3s */
						{0x14,1500,{S0,S0,S0,S0}},    /* state 3: NSR,EWY, 1.5s */
					};
					
					display(currState);
					GPIOB->ODR = (FSM[currState].output); 
					delayMs(FSM[currState].delay);
					input = (GPIOC->IDR & 0x0000000C)>>2;
					currState = FSM[currState].nextState[input]; 
				}
		/* URBAN */
				while((GPIOC->IDR & 0x03)==0x01) {
					display(11);
				
					/* FSM Setup */
					struct State {
						int output;
						int delay;
						int nextState[4];
					};
			
					typedef const struct State urban;
						#define S0 0
						#define S1 1
						#define S2 2
						#define S3 3
			
					urban FSM[4] = {
						{0x41,3000,{S1,S1,S1,S1}},      /* state 0: NSG/EWR, 3s */
						{0x42,1500,{S2,S2,S2,S2}},    /* state 1:  NSY/EWR, 1.5s */
						{0x0C,3000,{S3,S3,S3,S3}},      /* state 2: NSR/EWG, 3s */
						{0x14,1500,{S0,S0,S0,S0}},    /* state 3: NSR,EWY, 1.5s */
					};
					
					display(currState);
					if((currState==0 || currState==2) && pressed==1) {
						display(4);
						/* if so, then call pedestrian */
						pedestrian();
					}
					GPIOB->ODR = (FSM[currState].output); 
					delayMs(FSM[currState].delay);
					input = (GPIOC->IDR & 0x0000000C)>>2;
					currState = FSM[currState].nextState[input];
				} 	
				/* 4-WAY YIELD */
				while((GPIOC->IDR & 0x03)==0x02) {
					display(12);
					display(5);
					
					GPIOB->ODR = 0x12;
					delayMs(1000);
					GPIOB->ODR = 0x00;
					delayMs(500);
				}
				/* 4-WAY STOP */
				while((GPIOC->IDR & 0x03)==0x03) {
					display(13);
					display(6);
					
					GPIOB->ODR = 0x44;
					delayMs(1000);
					GPIOB->ODR = 0x00;
					delayMs(500);
				}
    }
}

void display(int state) {
			if(state==10) {
					LCD_command(0x80); /* set cursor to first line */
					LCD_data('M');
					LCD_data('O');
					LCD_data('D');
					LCD_data('E');
					LCD_data(' ');
					LCD_data('0');
					LCD_data(':');
					LCD_data(' ');
					LCD_data('R');
					LCD_data('U');
					LCD_data('R');
					LCD_data('A');
					LCD_data('L');
			} else if(state==11) {
					LCD_command(0x80); 
					LCD_data('M');
					LCD_data('O');
					LCD_data('D');
					LCD_data('E');
					LCD_data(' ');
					LCD_data('1');
					LCD_data(':');
					LCD_data(' ');
					LCD_data('U');
					LCD_data('R');
					LCD_data('B');
					LCD_data('A');
					LCD_data('N');
			} else if(state==12) {
					LCD_command(0x80); 
					LCD_data('M');
					LCD_data('O');
					LCD_data('D');
					LCD_data('E');
					LCD_data(' ');
					LCD_data('2');
					LCD_data(':');
					LCD_data(' ');
					LCD_data('Y');
					LCD_data('I');
					LCD_data('E');
					LCD_data('L');
					LCD_data('D');
			} else if(state==13) {
					LCD_command(0x80); 
					LCD_data('M');
					LCD_data('O');
					LCD_data('D');
					LCD_data('E');
					LCD_data(' ');
					LCD_data('3');
					LCD_data(':');
					LCD_data(' ');
					LCD_data('S');
					LCD_data('T');
					LCD_data('O');
					LCD_data('P');
					LCD_data(' ');
			} else if(state==0) {
					LCD_command(0xC0); /* set cursor to second line */
					LCD_data('E');
					LCD_data('W');
					LCD_data(' ');
					LCD_data('R');
					LCD_data('E');
					LCD_data('D');
					LCD_data(' ');
					LCD_data(' ');
					LCD_data('N');
					LCD_data('S');
					LCD_data(' ');
					LCD_data('G');
					LCD_data('R');
					LCD_data('N');
				  LCD_data(' ');
			} else if(state==1) {
					LCD_command(0xC0);
					LCD_data('E');
					LCD_data('W');
					LCD_data(' ');
					LCD_data('R');
					LCD_data('E');
					LCD_data('D');
					LCD_data(' ');
					LCD_data(' ');
					LCD_data('N');
					LCD_data('S');
					LCD_data(' ');
					LCD_data('Y');
					LCD_data('L');
					LCD_data('W');
				  LCD_data(' ');
			} else if(state==2) {
					LCD_command(0xC0);
					LCD_data('E');
					LCD_data('W');
					LCD_data(' ');
					LCD_data('G');
					LCD_data('R');
					LCD_data('N');
					LCD_data(' ');
					LCD_data(' ');
					LCD_data('N');
					LCD_data('S');
					LCD_data(' ');
					LCD_data('R');
					LCD_data('E');
					LCD_data('D');
				  LCD_data(' ');
			} else if(state==3) {
					LCD_command(0xC0);
					LCD_data('E');
					LCD_data('W');
					LCD_data(' ');
					LCD_data('Y');
					LCD_data('L');
					LCD_data('W');
					LCD_data(' ');
					LCD_data(' ');
					LCD_data('N');
					LCD_data('S');
					LCD_data(' ');
					LCD_data('R');
					LCD_data('E');
					LCD_data('D');
				  LCD_data(' ');
			} else if(state==4) {
					LCD_command(0xC0);
					LCD_data('P');
					LCD_data('E');
					LCD_data('D');
					LCD_data('E');
					LCD_data('S');
					LCD_data('T');
					LCD_data('R');
					LCD_data('I');
					LCD_data('A');
					LCD_data('N');
					LCD_data(' ');
					LCD_data('X');
					LCD_data(' ');
					LCD_data(' ');
				  LCD_data(' ');
			} else if(state==5) {
					LCD_command(0xC0);
					LCD_data('E');
					LCD_data('W');
					LCD_data('/');
					LCD_data('N');
					LCD_data('S');
					LCD_data(' ');
					LCD_data('B');
					LCD_data('L');
					LCD_data('I');
					LCD_data('N');
					LCD_data('K');
          LCD_data(' ');
					LCD_data('Y');
					LCD_data('L');
					LCD_data('W');
			} else if(state==6) {
					LCD_command(0xC0);
					LCD_data('E');
					LCD_data('W');
					LCD_data('/');
					LCD_data('N');
					LCD_data('S');
					LCD_data(' ');
					LCD_data('B');
					LCD_data('L');
					LCD_data('I');
					LCD_data('N');
					LCD_data('K');
          LCD_data(' ');
					LCD_data('R');
					LCD_data('E');
					LCD_data('D');
			}
}

void urban_init(void) {
		__disable_irq(); /* global disable IRQs */
		RCC->APB2ENR |= 0x4000; /* enable SYSCFG clock */
		GPIOB->MODER &= ~0x0000FFFF; /* clear pin mode */
		GPIOB->MODER = 0x00005555; /* set B pins to output mode */
		GPIOA->MODER &= ~0x00FFFF00; /*clear pin mode */
		GPIOA->MODER |= 0x00555500; /* set A pins to output mode */
	
		/* configure PC13 for push button interrupt */
		GPIOC->MODER &= ~0x0C000000;        /* clear pin mode to input mode */
		SYSCFG->EXTICR[3] &= ~0x00F0;       /* clear port selection for EXTI13 */
		SYSCFG->EXTICR[3] |= 0x0020;        /* select port C for EXTI13 */
		EXTI->IMR |= 0x2000;                /* unmask EXTI13 */
		EXTI->FTSR |= 0x2000;               /* select falling edge trigger */
		NVIC_EnableIRQ(EXTI15_10_IRQn);
    __enable_irq();                     /* global enable IRQs */	
}

/* Interrupt Handler */
void EXTI15_10_IRQHandler(void) {
        pressed = 1; /* it's true that button was pressed*/
        EXTI->PR = 0x2000;  /* clear interrupt pending flag */
}

/* Countdown */
void pedestrian(void) {
	GPIOB->ODR = 0x000000044; /* NSR, EWR ON */
	int countdown[10] = {0x6f, 0x7f, 0x07, 0x7d, 0x6d, 0x66, 0x4f, 0x5b, 0x06, 0x3f};
	for(int i=0; i<10; i++) {
		GPIOA->ODR = (countdown[i])<<4; /* display number */
		delayMs(750);
	}
	/* Blink 0 three times */
	for(int j=0; j<3; j++) { 
		GPIOA->ODR = (0x00)<<4;
		delayMs(750);
		GPIOA->ODR = (0x3f)<<4;
		delayMs(750);
	}
	GPIOA->ODR = 0x00; /* clear display */
	pressed = 0; /* reset button pressed to false */
}

/* initialize GPIOB/C then initialize LCD controller */
void LCD_init(void) {
    PORTS_init();

    delayMs(20);                /* LCD controller reset sequence */
    LCD_nibble_write(0x30, 0);
    delayMs(5);
    LCD_nibble_write(0x30, 0);
    delayMs(1);
    LCD_nibble_write(0x30, 0);
    delayMs(1);

    LCD_nibble_write(0x20, 0);  /* use 4-bit data mode */
    delayMs(1);
    LCD_command(0x28);          /* set 4-bit data, 2-line, 5x7 font */
    LCD_command(0x06);          /* move cursor right */
    LCD_command(0x01);          /* clear screen, move cursor to home */
    LCD_command(0x0F);          /* turn on display, cursor blinking */
}

void PORTS_init(void) {
    /* PORTB 5 for LCD R/S */
    /* PORTB 7 for LCD EN */
    GPIOB->MODER &= ~0x0000CC00;    /* clear pin mode */
    GPIOB->MODER |=  0x00004400;    /* set pin output mode */
    GPIOB->BSRR = 0x00800000;       /* turn off EN */

    /* PC4-PC7 for LCD D4-D7, respectively. */
    GPIOC->MODER &= ~0x0000FF00;    /* clear pin mode */
    GPIOC->MODER |=  0x00005500;    /* set pin output mode */
}

void LCD_nibble_write(char data, unsigned char control) {
    /* populate data bits */
    GPIOC->BSRR = 0x00F00000;       /* clear data bits */
    GPIOC->BSRR = data & 0xF0;      /* set data bits */

    /* set R/S bit */
    if (control & RS)
        GPIOB->BSRR = RS;
    else
        GPIOB->BSRR = RS << 16;

    /* pulse E */
    GPIOB->BSRR = EN;
    delayMs(0);
    GPIOB->BSRR = EN << 16;
}

void LCD_command(unsigned char command) {
    LCD_nibble_write(command & 0xF0, 0);    /* upper nibble first */
    LCD_nibble_write(command << 4, 0);      /* then lower nibble */

    if (command < 4)
        delayMs(2);         /* command 1 and 2 needs up to 1.64ms */
    else
        delayMs(1);         /* all others 40 us */
}

void LCD_data(char data) {
    LCD_nibble_write(data & 0xF0, RS);      /* upper nibble first */
    LCD_nibble_write(data << 4, RS);        /* then lower nibble */

    delayMs(1);
}

/* 16 MHz SYSCLK */
void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}
