#include "app_module_test.h"

void oled_test_run(void)
{
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "OLED TEST");
    OLED_ShowString(2, 1, "NUM:");
    OLED_ShowSignedNum(2, 6, -123, 3);
    OLED_ShowString(3, 1, "F:");
    OLED_ShowFloat(3, 3, 12.34f, 2, 2);

    while (1)
    {
        delay_ms(500);
    }
}

