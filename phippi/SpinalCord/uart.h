
/******************************************************************************
Function Prototypes
******************************************************************************/
extern void test_uart5(void);
extern void uart8_init(void);
extern void uart5_init(void);
extern void uart7_init(void);
extern void uart7_send(unsigned char);
extern void uart8_send (unsigned char);
extern void u5_print ( const char *ptr,  ...);

#define TX_BUFF_SIZE  256 // 512
#define RX_BUFF_SIZE  1024

extern char tx5_buff[TX_BUFF_SIZE ];    // saatebuff uart5
extern unsigned short tx5_ptr;               // salvestusviit
extern volatile unsigned short tx5_ptrr;              // väljastusviit
extern char rx8_buff[RX_BUFF_SIZE ];    // saatebuff uart8
extern unsigned short rx8_ptr;               // salvestusviit
extern unsigned short rx8_ptrr;              // väljastusviit

extern volatile unsigned char U8_in,U5_in;

extern void reset_com0 ( void );
extern void send_OK ( void);

#define OK 1
#define ERROR -1
#define FULL 0x80
#define EMPTY 0


// robustne häkk
#define rd_buff     rx8_buff
#define datat_buf   rx8_ptr


