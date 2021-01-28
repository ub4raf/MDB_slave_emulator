#include <msp430.h>
#include <stdio.h>
#include <string.h>

#define uint8_t char

#define MDB_BUF_MAX_LENGTH  16
uint8_t mdm_buffer[MDB_BUF_MAX_LENGTH]={0};
uint8_t mdb_buf_index=0;

/// coin changer:
#define ADDR_MASK   0xF8
#define ADDR_DEVICE 0x08

#define MDB_ACK     0x00
#define MDB_NACK    0xFF
#define MDB_RET     0xAA


#define TX_BUFFER_SIZE 256

char tx_buffer[TX_BUFFER_SIZE]={0};
#if TX_BUFFER_SIZE <= 256
uint8_t tx_wr_index=0,tx_rd_index=0,tx_counter=0;
#else
uint16_t tx_wr_index=0,tx_rd_index=0,tx_counter=0;
#endif

char message [16]={'\0'};

const uint8_t   setup_coin_message[23]=
                            { 0x02,//level 2 protocol (simply)
                              0x16,0x43,//currency "RUB": 0x1000 + 0x643 (rub)
                              0x64,// Coin Scaling Factor   (100)
                              0x02,// Decimal Places        (2)
                              0x00,0xFF,// 8 types of coins:
                              0x01,0x02,0x05,0x0A,// value of coins: 1 2 5 10
                              0x19,0x32,0x64,0xC8,// value of coins: 25 50 100 200
                              0x00,0x00,0x00,0x00,///// value of unused coins
                              0x00,0x00,0x00,0x00///// value of unused coins
                    };

const uint8_t   status_coin_message[18]={
                            0x00,0x00,/// coin tubes are free
                            0xFF,0xFF,0xFF,0xFF,/// in used tubes > 255 coins
                            0xFF,0xFF,0xFF,0xFF,////
                            0x00,0x00,0x00,0x00,////
                            0x00,0x00,0x00,0x00/////
                    };

const uint8_t   dispense_100_coin_message[3]={
                                          0x57,0xFF,0x01/// see p66 of MDB standart
};

/*******************************************************************************
*Function:    sendByte
*Description: Sends a byte
*******************************************************************************/
void sendByte(char data)
{
  while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
  UCA0TXBUF = data;
}

/*******************************************************************************
*Function:    receiveByte
*Description: receives a byte
*******************************************************************************/
char receiveByte()
{
  while( !(UCA0IFG & UCRXIFG));       // wait for RX flag
  return UCA0RXBUF;
}

/******************************************************************************
 * function for sending a NULL terminated string via UART interface
 *****************************************************************************/

void UART_putchar (char data)
{

    while (tx_counter == TX_BUFFER_SIZE); //  ждать освобождени¤ буфера
        //__disable_interrupt();
           tx_buffer[tx_wr_index++]=data;    /// заносить в кольцевой буфер символ
            #if TX_BUFFER_SIZE != 256
               if (tx_wr_index == TX_BUFFER_SIZE) tx_wr_index=0;
            #endif
               ++tx_counter;
               UCA1IE |= UCTXIE;
}

void UART_puts (char *string)
{
  unsigned int i;
  for(i=0 ; i<strlen(string) ; i++)
  {
      UART_putchar(string[i]);
  }
}


/******************************************************************************
 * function for initializing peripherals
 *****************************************************************************/
void init(void)
    {
      /*
       * Set the UCS to work with REFO (ACLK) and DCO (MCLK, SMCLK)
       */
      UCSCTL6 |= XT1OFF + XT2OFF;
      UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
      UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

      // Loop until XT1,XT2 & DCO fault flag is cleared
      do
      {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
                                                // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                      // Clear fault flags
      }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

      /*
       *  setup UART interface
       */
      P4SEL |= BIT5 + BIT4;                     // P5.6,7 = USCI_A0 TXD/RXD
      UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
      UCA1CTL1 |= UCSSEL_2;                     // SMCLK
      UCA1BR0 = 6;                              // 1MHz 9600 (see User's Guide)
      UCA1BR1 = 0;                              // 1MHz 9600
      UCA1MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
                                                // over sampling
      UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
      UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

      P3SEL |= BIT3 + BIT4;                     // P5.6,7 = USCI_A0 TXD/RXD
      UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
      UCA0CTL1 |= UCSSEL_2|UCDORM;                     // SMCLK
      UCA0CTL0 |= UCMODE_2;
      UCA0BR0 = 6;                              // 1MHz 9600 (see User's Guide)
      UCA0BR1 = 0;                              // 1MHz 9600
      UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
                                                // over sampling
      UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
      UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

      __bis_SR_register(GIE);
    }

uint8_t mdb_chk_calc(uint8_t *data,uint8_t len) /// CHK byte is simply summ
    {
        uint8_t i;
        uint8_t out=0;
        for(i=0;i<len;i++)
            out+=data[i];
        return out;
    }

void    mdb_message_answer (uint8_t *data,uint8_t len)  //  slave full message answer
    {
        if(!len)
            return;
        uint8_t i;
        for(i=0;i<len;i++)
            sendByte(data[i]);
        while (!(UCA0IFG&UCTXIFG));//need to wait, else CURRENT byte will be send w addr bit.
        UCA0CTL1 |= UCTXADDR;
        sendByte(mdb_chk_calc(data,len));
    }

void    mdb_ack_answer (uint8_t data)   //   slave 1B answer
    {
        UCA0CTL1 |= UCTXADDR;
        sendByte(data);
    }

void    mdb_message_send (uint8_t addr, uint8_t *data, uint8_t len) //  master debug emulator
    {
        UCA0CTL1 |= UCTXADDR;
        sendByte(addr);
        uint8_t i;
        for(i=0;i<len;i++)
            sendByte(data[i]);
        sendByte(mdb_chk_calc(data,len)+addr);
    }

void set_5ms_timer(void)
    {}
void stop_5ms_timer(void)
    {}
void timer_5ms_callback(void)
    {
        UART_puts("tout");
        mdb_buf_index=0;///exit
        UCA0CTL1|=UCDORM;
    }

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	  init();

	  UART_puts("\t\t\t\t\t\t\t\t\t\t\t\n\r");
	  UART_puts("Welcome to MDB coin changer emulator\n");

	  //buttons
	  P2DIR&=~BIT1;
	  P4DIR&=~BIT1;
	  P1REN|=BIT1;
      P1REN|=BIT1;
      // leds
      P1DIR |= BIT0;
      P1OUT &= ~BIT0;
      P4DIR |= BIT7;
      P4OUT &= ~BIT7;

	  while(1)
	      {

	      }

	return 0;
}

void    mdb_process(uint8_t *buffer,uint8_t buf_len)
    {
        char i;
        if(!buf_len)
            return;
        UART_puts("MDB_process:\t");
        if(buf_len==1)
            switch(buffer[0])
                {
                    /*case MDB_ACK:
                        UART_puts("ACK\n");
                    break;
                    case MDB_NACK:
                        UART_puts("NACK\n");
                    break;
                    case MDB_RET:
                        UART_puts("RET\n");
                    break;*/
                    case 0x08://RESET 08H Command for changer to self-reset
                        UART_puts("RST\n");
                        mdb_ack_answer(MDB_ACK);
                    break;
                    case 0x09://SETUP * 09H Request for changer setup information.
                        UART_puts("STP\n");
                        mdb_message_answer((char*)setup_coin_message,23);
                    break;
                    case 0x0A://TUBE STATUS 0AH Request for changer tube status.
                        UART_puts("TSTT\n");
                        mdb_message_answer((char*)status_coin_message,18);
                    break;
                    case 0x0B://POLL 0BH Request for changer activity status.
                        UART_puts("POLL:");
                        if(P2IN&BIT1)   /////// PLACE YOUR CODE HERE
                            {
                                mdb_ack_answer(MDB_ACK);    /// no events
                                UART_puts("ACK\n");
                            }
                        else
                            {
                                mdb_message_answer((char*)dispense_100_coin_message,3);
                                UART_puts("100\n"); ////    100 coins dispensed
                            }
                    break;

                }
        else
            {
                switch(buffer[0])
                    {
                        case 0x0C://COIN TYPE 0CH Signifies coin types accepted and\
                            allowable coin dispensing. This\
                            command is followed by setup data.\
                            See command format section.
                            UART_puts("TYPE\t");
                            mdb_ack_answer(MDB_ACK);///BTN
                            for(i=0;i<buf_len;i++)
                                {
                                    sprintf(message,"%02X ",buffer[i]);
                                    UART_puts(message);
                                }
                            UART_puts("\n");
                        break;
                        case 0x0D://DISPENSE 0DH Command to dispense a coin type.\
                            Followed by coin type to dispense.\
                            See command format section.
                            UART_puts("DISP\t");
                            mdb_ack_answer(MDB_ACK);///BTN
                            for(i=0;i<buf_len;i++)
                                {
                                    sprintf(message,"%02X ",buffer[i]);
                                    UART_puts(message);
                                }
                            UART_puts("\n");
                        break;
                    }
            }
    }

/******************************************************************************
 * ISR function for UART communication
 *****************************************************************************/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
  char rx_byte;
  P1OUT|= BIT0; /// blink to show DEBUG uart
  switch(__even_in_range(UCA1IV,4))
  {
    case 0:break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG

      rx_byte = UCA1RXBUF;
          switch(rx_byte)   ////    TEST master commands with shorted Tx-Rx.
              {
                  case '0':
                    UART_puts("SND8");
                    mdb_message_send(0x08,0,0);
                  break;
                  case '1':
                    UART_puts("SND9");
                    mdb_message_send(0x09,0,0);
                  break;
                  case '2':
                    UART_puts("SNDA");
                    mdb_message_send(0x0A,0,0);
                  break;
                  case '3':
                    UART_puts("SNDB");
                    mdb_message_send(0x0B,0,0);
                  break;
                  case '4':
                    UART_puts("SNDC");
                    mdb_message_send(0x0C,0,0);
                  break;
                  case '5':
                    UART_puts("SNDD");
                    mdb_message_send(0x0D,0,0);
                  break;
              }

      break;
    case 4:                            // Vector 4 - TXIFG
        if (tx_counter) ///
           {
               --tx_counter;
               UCA1TXBUF = tx_buffer[tx_rd_index++];
            #if TX_BUFFER_SIZE != 256
               if (tx_rd_index == TX_BUFFER_SIZE) tx_rd_index=0;
            #endif
           }
        else
            {
                UCA1IE &= ~UCTXIE; //  stop interrupt
                UCA1IFG |= UCTXIFG;//  but SET interrupt flag for future transmission
            }
        break;
    default: break;
  }

  P1OUT&= ~BIT0;
}

/******************************************************************************
 * ISR function for UART communication
 *****************************************************************************/
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  char rx_byte;
  P4OUT|= BIT7; /// blink to show MDB uart
  switch(__even_in_range(UCA0IV,4))
  {
    case 0:break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG

      rx_byte = UCA0RXBUF;
      if(UCA0CTL1&UCDORM)
          {
              mdb_buf_index=0;
              if((rx_byte&ADDR_MASK)==ADDR_DEVICE)
                  {
                      mdm_buffer[mdb_buf_index++]=rx_byte;
                      UCA0CTL1&=~UCDORM;
                      set_5ms_timer();
                  }
              //// TODO RET command
              //UART_puts("C:");
              //UART_putchar(rx_byte);
          }
      else
          {
              //UART_puts("D:");
              //UART_putchar(rx_byte);

              if(mdb_buf_index>MDB_BUF_MAX_LENGTH-2)
                  {
                      stop_5ms_timer();
                      mdb_buf_index=0;///exit in case of overflow
                      UCA0CTL1|=UCDORM;
                  }
              else
              if(mdb_chk_calc(mdm_buffer,mdb_buf_index)==rx_byte)/// last CHK byte
                  {
                      stop_5ms_timer();
                      //UART_puts("RCV:");
                      mdb_process(mdm_buffer,mdb_buf_index);
                      mdb_buf_index=0;///exit
                      UCA0CTL1|=UCDORM;
                  }
              else
                  {
                      mdm_buffer[mdb_buf_index++]=rx_byte;
                      set_5ms_timer();
                  }
          }

      break;
    case 4:break;                             // Vector 4 - TXIFG
    default: break;
  }
  P4OUT&= ~BIT7;
}
