#include "mybsp.h"




// 定义行和列的GPIO
#define KEYPAD_ROW_PORT GPIOF
#define KEYPAD_ROW_1_PIN GPIO_PIN_0
#define KEYPAD_ROW_2_PIN GPIO_PIN_1
#define KEYPAD_ROW_3_PIN GPIO_PIN_2
#define KEYPAD_ROW_4_PIN GPIO_PIN_3

#define KEYPAD_COL_PORT GPIOF
#define KEYPAD_COL_1_PIN GPIO_PIN_4
#define KEYPAD_COL_2_PIN GPIO_PIN_5
#define KEYPAD_COL_3_PIN GPIO_PIN_6
#define KEYPAD_COL_4_PIN GPIO_PIN_7




#define DEBOUNCE_TIME_MS 20


const unsigned char key_map[4][4] = {
    {KEY_1, KEY_2, KEY_3, KEY_A},
    {KEY_4, KEY_5, KEY_6, KEY_B},
    {KEY_7, KEY_8, KEY_9, KEY_C},
    {KEY_STAR, KEY_0, KEY_HASH, KEY_D}
};
// 列引脚数组

GPIO_TypeDef* col_ports[4] = {KEYPAD_COL_PORT, KEYPAD_COL_PORT, KEYPAD_COL_PORT, KEYPAD_COL_PORT};
uint16_t col_pins[4] = {KEYPAD_COL_1_PIN, KEYPAD_COL_2_PIN, KEYPAD_COL_3_PIN, KEYPAD_COL_4_PIN};

// 行引脚数组
GPIO_TypeDef* row_ports[4] = {KEYPAD_ROW_PORT, KEYPAD_ROW_PORT, KEYPAD_ROW_PORT, KEYPAD_ROW_PORT};
uint16_t row_pins[4] = {KEYPAD_ROW_1_PIN, KEYPAD_ROW_2_PIN, KEYPAD_ROW_3_PIN, KEYPAD_ROW_4_PIN};




// ---------------------- 状态机变量 ----------------------
// 当前扫描的列
static uint8_t current_col = 0;
// 上一次扫描到的按键
static unsigned char last_key_detected = NO_KEY;
// 存储最终按下的键值
static unsigned char key_event_value = NO_KEY;
// 存储最终按键事件类型
static Key_Event_Type key_event = KEY_EVENT_NONE;
// 用于消抖的计时器
static uint32_t debounce_timer = 0;
// --------------------------------------------------------

/**
 * @brief 在主循环中周期性调用的任务函数
 */
void Keypad_FSM_Task(void)
{
    unsigned char current_key = NO_KEY;
    int row, col;

    // 扫描所有列
    for (col = 0; col < 4; col++) {
        // 首先将所有列引脚设置为高电平
        for (int i = 0; i < 4; i++) {
            HAL_GPIO_WritePin(col_ports[i], col_pins[i], GPIO_PIN_SET);
        }
        
        // 设置当前列为低电平
        HAL_GPIO_WritePin(col_ports[col], col_pins[col], GPIO_PIN_RESET);
        
        // 短暂延时确保电平稳定
        for(volatile int i = 0; i < 100; i++);
        
        // 读取所有行，检测按键状态
        for (row = 0; row < 4; row++)
        {
            if (HAL_GPIO_ReadPin(row_ports[row], row_pins[row]) == GPIO_PIN_RESET)
            {
                current_key = key_map[row][col];
                goto key_found; // 找到按键后立即退出双重循环
            }
        }
    }
    
    key_found:
    
    // ------------------- 按键状态机逻辑 -------------------
    // 1. 如果当前没有按键被按下
    if (current_key == NO_KEY)
    {
        // 如果上一次有按键被检测到，且消抖时间已到
        if (last_key_detected != NO_KEY && HAL_GetTick() - debounce_timer > DEBOUNCE_TIME_MS)
        {
            key_event_value = last_key_detected;
            key_event = KEY_EVENT_RELEASED;
            last_key_detected = NO_KEY;
        }
    }
    // 2. 如果有按键被按下
    else
    {
        // 如果是新的按键按下
        if (current_key != last_key_detected)
        {
            debounce_timer = HAL_GetTick(); // 重置消抖计时器
            last_key_detected = current_key; // 记录新的按键
        }
        // 如果是同一个按键，并且消抖时间已到
        else if (HAL_GetTick() - debounce_timer > DEBOUNCE_TIME_MS)
        {
            key_event_value = current_key;
            key_event = KEY_EVENT_PRESSED;
        }
    }
}


/**
 * @brief 获取按键事件
 * @param key_value 用于返回按下的键值
 * @return 返回按键事件类型
 */
Key_Event_Type Keypad_Get_Event(unsigned char *key_value)
{
    Key_Event_Type event_to_return = KEY_EVENT_NONE;
    if (key_event != KEY_EVENT_NONE)
    {
        event_to_return = key_event;
        *key_value = key_event_value;
        key_event = KEY_EVENT_NONE; // 清除事件标志
    }
    return event_to_return;
}




