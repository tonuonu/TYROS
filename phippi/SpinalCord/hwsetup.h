void HardwareSetup(void);

#define LED1     p3_3
#define LED2     p3_5
#define KEY_1   p5_3
#define KEY_2   p5_2
#define KEY_3   p5_1

void SPI3_Init(void);
void SPI4_Init(void);
void uart7_Init(void);
struct statuses
{
    char sek_flag;
};

extern volatile struct statuses status;
