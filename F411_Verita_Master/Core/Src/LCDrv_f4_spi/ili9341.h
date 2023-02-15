/* Interface mode
   - 1: SPI or paralell interface mode
   - 2: RGB mode (LTDC hardware, HSYNC, VSYNC, pixel clock, RGB bits data, framebuffer memory)
*/
#define  ILI9341_INTERFACE_MODE   1

/* Orientation
   - 0: 240x320 portrait (plug in top)
   - 1: 320x240 landscape (plug in left)
   - 2: 240x320 portrait (plug in botton)
   - 3: 320x240 landscape (plug in right)
*/
#define  ILI9341_ORIENTATION      0

/* Color mode
   - 0: RGB565 (R:bit15..11, G:bit10..5, B:bit4..0)
   - 1: BRG565 (B:bit15..11, G:bit10..5, R:bit4..0)
*/
#define  ILI9341_COLORMODE        0

/* To clear the screen before display turning on ?
   - 0: does not clear
   - 1: clear
*/
#define  ILI9341_INITCLEAR        1

/* Analog touchscreen (only INTERFACE_MODE == 1, 8bit paralell IO mode)
   - 0: touchscreen disabled
   - 1: touchscreen enabled
*/
#define  ILI9341_TOUCH            0

/* Touchscreen calibration data for 4 orientations */
#define  TS_CINDEX_0        {1444723, 5348, -114234, 421806850, -131233, -975, 521525308}
#define  TS_CINDEX_1        {1444723, -131233, -975, 521525308, -5348, 114234, -76518053}
#define  TS_CINDEX_2        {1444723, -5348, 114234, -76518053, 131233, 975, -60658671}
#define  TS_CINDEX_3        {1444723, 131233, 975, -60658671, 5348, -114234, 421806850}

/* For multi-threaded or intermittent use, Lcd and Touchscreen simultaneous use can cause confusion (since it uses common I/O resources)
   Lcd functions wait for the touchscreen header, the touchscreen query is not executed when Lcd is busy.
   Note: If the priority of the Lcd is higher than that of the Touchscreen, it may end up in an infinite loop!
   - 0: multi-threaded protection disabled
   - 1: multi-threaded protection enabled
*/
#define  ILI9341_MULTITASK_MUTEX   0

#if  ILI9341_INTERFACE_MODE == 2

/* please see in the main.c what is the LTDC_HandleTypeDef name */
extern   LTDC_HandleTypeDef       hltdc;

/* Frambuffer memory alloc, free */
#define  ILI9341_MALLOC           malloc
#define  ILI9341_FREE             free

/* include for memory alloc/free */
#include <stdlib.h>

#endif  /* #if ILI9341_INTERFACE_MODE == 2 */

//-----------------------------------------------------------------------------
// ILI9341 physic resolution (in 0 orientation)
#define  ILI9341_LCD_PIXEL_WIDTH  240
#define  ILI9341_LCD_PIXEL_HEIGHT 320

#define cl_BLACK       0x0000
#define cl_NAVY        0x000F
#define cl_DARKGREEN   0x03E0
#define cl_DARKCYAN    0x03EF
#define cl_MAROON      0x7800
#define cl_PURPLE      0x780F
#define cl_OLIVE       0x7BE0
#define cl_LIGHTGREY   0xC618
#define cl_GRAY        0x5AEB
#define cl_DARKGREY    0x7BEF
#define cl_BLUE        0x001F
#define cl_GREEN       0x07E0
#define cl_CYAN        0x07FF
#define cl_RED         0xF800
#define cl_MAGENTA     0xF81F
#define cl_YELLOW      0xFFE0
#define cl_WHITE       0xFFFF
#define cl_ORANGE      0xFD20
#define cl_GREENYELLOW 0xAFE5
#define cl_PINK        0xF81F


/*****************************************************************************/
// Lcd Prototypo
/*****************************************************************************/

void     ili9341_Init(void);
uint16_t ili9341_ReadID(void);
void     ili9341_DisplayOn(void);
void     ili9341_DisplayOff(void);
void     ili9341_SetCursor(uint16_t Xpos, uint16_t Ypos);
void     ili9341_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code);
uint16_t ili9341_ReadPixel(uint16_t Xpos, uint16_t Ypos);
void     ili9341_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     ili9341_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     ili9341_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     ili9341_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t RGBCode);
uint16_t ili9341_GetLcdPixelWidth(void);
uint16_t ili9341_GetLcdPixelHeight(void);
void     ili9341_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);
void     ili9341_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData);
void     ili9341_ReadRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData);
void     ili9341_Scroll(int16_t Scroll, uint16_t TopFix, uint16_t BottonFix);
