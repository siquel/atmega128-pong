#define F_CPU 1000000UL

#include <string.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include "font5x7.h"

typedef struct {
    int x;
    int y;
    int w;
    int h;
} rect_t;

typedef struct {
    int x;
    int y;
    int radius;
} circle_t;

typedef enum {
    LCD_CMD = 0,
    LCD_DATA = 1
} lcd_cmd_data_t;

typedef struct {
    rect_t paddle;
    int score;
} player_t;

typedef enum { 
    player_one, 
    player_two 
} player_index_t;

/* Lcd screen size */
#define LCD_X_RES 132
#define LCD_Y_RES 64

uint8_t cpu_framebuffer[LCD_X_RES][8];
uint8_t gpu_framebuffer[LCD_X_RES][8];

/* Pinout for LCD */
#define LCD_CLK_PIN 	(1<<PC4)
#define LCD_DATA_PIN 	(1<<PC3)
#define LCD_DC_PIN 		(1<<PC2)
#define LCD_CE_PIN 		(1<<PC1)
#define LCD_RST_PIN 	(1<<PC0)
#define LCD_PORT		PORTC
#define LCD_DDR			DDRC

void lcd_send(uint8_t data, lcd_cmd_data_t cd);

void lcd_clear()
{
    int i, j;

    for (i = 0; i < 8; i++)
    {
        lcd_send(0xB0 | i, LCD_CMD);
        lcd_send(0x10, LCD_CMD);
        lcd_send(0x00, LCD_CMD);	// column 0

        for (j = 0; j < LCD_X_RES; j++)
        {
            lcd_send(0x00, LCD_DATA);
        }
    }

    lcd_send(0xB0, LCD_CMD);	// page 0
    lcd_send(0x10, LCD_CMD);
    lcd_send(0x00, LCD_CMD);	// column 0

    memset(cpu_framebuffer, 0, sizeof cpu_framebuffer);
}

void lcd_send(uint8_t data, lcd_cmd_data_t cd)
{
    // Data/DC are outputs for the lcd (all low)
    LCD_DDR |= LCD_DATA_PIN | LCD_DC_PIN;

    // Enable display controller (active low)
    LCD_PORT &= ~LCD_CE_PIN;

    // Either command or data
    if (cd == LCD_DATA)
    {
        LCD_PORT |= LCD_DC_PIN;
    }
    else
    {
        LCD_PORT &= ~LCD_DC_PIN;
    }

    for (unsigned char i = 0; i < 8; i++)
    {
        // Set the DATA pin value
        if ((data >> (7 - i)) & 0x01)
        {
            LCD_PORT |= LCD_DATA_PIN;
        }
        else
        {
            LCD_PORT &= ~LCD_DATA_PIN;
        }

        // Toggle the clock
        LCD_PORT |= LCD_CLK_PIN;
        for (int j = 0; j < 4; j++); // delay
        LCD_PORT &= ~LCD_CLK_PIN;
    }

    // Disable display controller
    //LCD_PORT &= ~LCD_DC_PIN;
    LCD_PORT |= LCD_CE_PIN;

    // Data/DC can be used as button inputs when not sending to LCD (/w pullups)
    LCD_DDR &= ~(LCD_DATA_PIN | LCD_DC_PIN);
    LCD_PORT |= LCD_DATA_PIN | LCD_DC_PIN;
}

void lcd_init(void)
{
    memset(cpu_framebuffer, 0, sizeof cpu_framebuffer);
    memset(gpu_framebuffer, 0, sizeof gpu_framebuffer);

    //Pull-up on reset pin
    LCD_PORT |= LCD_RST_PIN;	//Reset = 1

    //Set output bits on lcd port
    LCD_DDR |= LCD_RST_PIN | LCD_CE_PIN | LCD_DC_PIN | LCD_DATA_PIN | LCD_CLK_PIN;

    //Wait after VCC high for reset (max 30ms)
    _delay_ms(15);

    //Toggle display reset pin
    LCD_PORT &= ~LCD_RST_PIN; 	//Reset = 0
    _delay_ms(15);
    LCD_PORT |= LCD_RST_PIN;	//Reset = 1

    _delay_ms(15);

    //Disable LCD controller
    LCD_PORT |= LCD_CE_PIN;

    lcd_send(0xEB, LCD_CMD);  	//LCD bias 
    lcd_send(0x23, LCD_CMD);  	//Set Lines >> 23 = 64
    lcd_send(0x81, LCD_CMD);	//Set Potentiometer
    lcd_send(0x64, LCD_CMD);	//16 >> 64 (Tummuus)
    lcd_send(0xAF, LCD_CMD);  	//Set Display ON
    lcd_send(0xCC, LCD_CMD);  	//Set LCD to RAM mapping

    // Clear lcd
    lcd_clear();
}

void lcd_char(int x, int y, int8_t c)
{
    // char is 5 pixel wide
    for (int i = 0; i < 5; ++i)
    {
        cpu_framebuffer[x + i][y] = (pgm_read_byte(&font5x7[c - 32][i]) << 1);
    }
}

void lcd_pixel(int x, int y)
{
    int page = y / 8;
    int y_line = y % 8;

    int i = y_line;

    int chr = 1;

    while (i > 0) {
        chr = chr << 1;
        --i;
    }

    chr = chr | cpu_framebuffer[x][page];

    cpu_framebuffer[x][page] = chr;
}

void lcd_submit()
{
    for (int x = 0; x < LCD_X_RES; ++x)
    {
        for (int y = 0; y < 8; ++y)
        {
            if (gpu_framebuffer[x][y] == cpu_framebuffer[x][y]) continue;

            gpu_framebuffer[x][y] = cpu_framebuffer[x][y];

            lcd_send(0xB0 | y, LCD_CMD);	// page

            lcd_send(0x00 | (x & 0x0F), LCD_CMD); // what is this?
            lcd_send(0x10 | ((x & 0xF0) >> 4), LCD_CMD);	// column


            lcd_send(gpu_framebuffer[x][y], LCD_DATA);
        }
    }
}

void rect_fill_draw(rect_t* rect) {
    int x = rect->x;
    int y = rect->y;

    int w = rect->w;
    int h = rect->h;

    for (int i = y; i < y + h; i++) {
        for (int j = x; j < x + w; j++) {
            lcd_pixel(j, i);
        }
    }
}

void circle_fill_draw(circle_t* circle) {

    int radius = circle->radius;

    while (radius >= 0)
    {
        int x = radius, y = 0;
        int xChange = 1 - (radius << 1);
        int yChange = 0;
        int radiusError = 0;
        int x0 = circle->x;
        int y0 = circle->y;


        while (x >= y)
        {
            lcd_pixel(x + x0, y + y0);
            lcd_pixel(y + x0, x + y0);
            lcd_pixel(-x + x0, y + y0);
            lcd_pixel(-y + x0, x + y0);
            lcd_pixel(-x + x0, -y + y0);
            lcd_pixel(-y + x0, -x + y0);
            lcd_pixel(x + x0, -y + y0);
            lcd_pixel(y + x0, -x + y0);

            y++;
            radiusError += yChange;
            yChange += 2;
            if (((radiusError << 1) + xChange) > 0)
            {
                x--;
                radiusError += xChange;
                xChange += 2;
            }
        }
        --radius;
    }
}

void adc_init()
{
    ADCSRA |= 1 << ADPS2;   //Esijakaja 64 -> Taajuus sopivaksi AD-muuntimelle
    ADCSRA |= 1 << ADPS1;

    ADMUX |= 1 << ADLAR;    //AD-muunnoksen tulos vasemmalle vieritetty

    ADMUX |= 1 << REFS0;
    ADMUX &= ~(1 << REFS1); //Avcc(+5v) muuntimen referenssijännitteeksi

    ADCSRA |= 1 << ADEN;    //Otetaan AD-muunnin käyttöön
}


uint16_t adc_read(uint8_t ch)
{
    uint16_t ADCresult = 0;

    ADMUX &= (~0x1F);                           //Nollataan rekisterin kanavanvalintabitit
    ADMUX |= ch;                                //otetaan haluttu kanava k?ytt??n

    ADCSRA |= 1 << ADSC;                        //Aloitetaan uusi muunnos
    while (!(ADCSRA & (1 << ADIF)));            //Odotetaan että muunnos on valmis

    uint8_t theLowADC = (ADCL >> 6);            // Luetaan AD-muuntimelta tuleva LSB ja bittien siirto
    uint8_t theHighADC = ADCH;                  // Luetaan AD-muuntimelta tuleva MSB

    ADCresult = theLowADC | (theHighADC << 2);  //Yhdistetään AD-muuntimen LSB ja MSB ja bittien siirto
    ADCresult = ADCresult & 0x03FF;             //Tuloksen maskaus	

    return ADCresult;
}

void rect_keep_in_screen(rect_t* rect) {
    if (rect->x <= 4) rect->x = 4;
    if (rect->y < 0) rect->y = 0;
    if (rect->x + rect->w >= LCD_X_RES) rect->x = LCD_X_RES - rect->w;
    if (rect->y + rect->h >= LCD_Y_RES) rect->y = LCD_Y_RES - rect->h;
}

void joystick_read(int* vx, int* vy, player_index_t pindex)
{
    *vx = 0;
    *vy = 0;

    int channelX = 0, channelY = 0;

    // ADC kanavat 0, 1
    if (pindex == player_one) {
        channelX = 0;
        channelY = 1;
    } // ADC kanavat 2, 3
    else if (pindex == player_two) {
        channelX = 2;
        channelY = 3;
    }

    float temp = adc_read(channelX); //AD-muuntimen lukeminen, kanava 0 (0b00000)
    float xVoltage = ((temp * 5.f) / 1024);

    temp = adc_read(channelY);//AD-muuntimen lukeminen, kanava 1 (0b00001)
    float yVoltage = ((temp * 5.f) / 1024);

    if (xVoltage < 2.4f)
        *vx = -1;
    else if (xVoltage > 2.9f)
        *vx = 1;

    if (yVoltage < 2.4f)
        *vy = -1;
    else if (yVoltage > 2.9f)
        *vy = 1;
}

int rect_collision_circle(rect_t* r, circle_t* circle) {

    rect_t c;
    c.x = circle->x - circle->radius;
    c.y = circle->y - circle->radius;
    c.h = circle->radius * 2;
    c.w = circle->radius * 2;

    int noOverlap = r->x > (c.x + c.w) ||
        c.x > (r->x + r->w) ||
        r->y > (c.y + c.h) ||
        c.y > (r->y + r->h);

    return !noOverlap;
}

void game_reset(player_t* p1, player_t* p2, circle_t* ball) {
    p1->score = 0;
    p1->paddle.x = 8;
    p1->paddle.y = 0;
    p1->paddle.w = 4;
    p1->paddle.h = 18;

    p2->score = 0;
    p2->paddle.w = p1->paddle.w;
    p2->paddle.h = p1->paddle.h;
    p2->paddle.y = 0;
    p2->paddle.x = LCD_X_RES - p1->paddle.x;

    ball->x = 70;
    ball->y = 30;
    ball->radius = 5;
}

int main()
{
    adc_init();
    lcd_init();

    player_t player, player2;
    circle_t ball;

    game_reset(&player, &player2, &ball);

    int velx = 1;
    int vely = 1;
    float speed = 1.f;
    float constant = 0.2f;

    char mem[10];

    for (;;)
    {
        // clear buffer
        memset(cpu_framebuffer, 0, sizeof(cpu_framebuffer));

        memset(mem, 0, sizeof mem);
        sprintf(mem, "P1: %d", player.score);

        for (int i = 0; i < strlen(mem); ++i)
            lcd_char(20 + i * 6, 1, mem[i]);

        memset(mem, 0, sizeof mem);

        sprintf(mem, "P2: %d", player2.score);

        for (int i = 0; i < strlen(mem); ++i)
            lcd_char(90 + i * 6, 1, mem[i]);

        int vx = 0, vy = 0;

        joystick_read(&vx, &vy, player_one);

        if (vx != 0) player.paddle.x += vx * 1;
        if (vy != 0) player.paddle.y += vy * 1;

        rect_keep_in_screen(&player.paddle);

        joystick_read(&vx, &vy, player_two);

        if (vx != 0) player2.paddle.x += vx * 1;
        if (vy != 0) player2.paddle.y += vy * 1;

        rect_keep_in_screen(&player2.paddle);

        if (ball.y - ball.radius <= 0) vely = 1;
        if (ball.y + ball.radius >= 64) vely = -1;

        if (rect_collision_circle(&player.paddle, &ball)) {
            ball.x = player.paddle.x + player.paddle.w + ball.radius;
            velx *= -1;
            speed += constant;
        }
        else if (rect_collision_circle(&player2.paddle, &ball)) {
            ball.x = player2.paddle.x - player2.paddle.w - ball.radius;
            velx *= -1;
            speed += constant;
        }

        ball.x += velx * speed;
        ball.y += vely * speed;

        int spawn = 0;

        if (ball.x + ball.radius >= LCD_X_RES) {
            ++player.score;
            spawn = 1;
        }
        else if (ball.x - ball.radius <= 0) {
            ++player2.score;
            spawn = 1;
        }

        if (spawn) {
            ball.x = 70;
            ball.y = 30;
            speed = 1.f;
        }

        circle_fill_draw(&ball);
        rect_fill_draw(&player.paddle);
        rect_fill_draw(&player2.paddle);

        lcd_submit();

        if (player.score >= 3 || player2.score >= 3) {

            memset(cpu_framebuffer, 0, sizeof(cpu_framebuffer));
            memset(mem, 0, sizeof mem);

            if (player.score > player2.score)
                sprintf(mem, "P1 WON!");
            else
                sprintf(mem, "P2 WON!");

            for (int i = 0; i < strlen(mem); ++i)
                lcd_char(50 + i * 6, 4, mem[i]);


            lcd_submit();

            _delay_ms(7500);

            game_reset(&player, &player2, &ball);
            // reset speed
            speed = 1.f;
        }
    }
    return 0;
}
