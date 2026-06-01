#include "gray.h"
#include "app.h"

void gray_read(Gray_TypeDef* gray)
{
    // 读取各个灰度对管（1表示检测到黑线，0表示检测到白色）
    gray->left3  = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);
    gray->left2  = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
    gray->left1  = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
    gray->middle = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
    gray->right1 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6);
    gray->right2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
    gray->right3 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);

}
float calculate_tracking_error(Gray_TypeDef* gray)
{
    float error = 0;
    uint8_t sensor_sum = 0;
    
    // 计算偏差（加权平均法）
	if (gray->left3  == 0) { error -= 3.0f; sensor_sum++; }
    if (gray->left2  == 0) { error -= 2.0f; sensor_sum++; }
    if (gray->left1  == 0) { error -= 1.0f; sensor_sum++; }
    if (gray->middle == 0) { error += 0.0f; sensor_sum++; }
    if (gray->right1 == 0) { error += 1.0f; sensor_sum++; }
    if (gray->right2 == 0) { error += 2.0f; sensor_sum++; }
	if (gray->right3 == 0) { error += 3.0f; sensor_sum++; }
    
    if (sensor_sum == 0)  { 
        return error;  // 返回上次的偏差
    }
    // 归一化处理 
    error /= sensor_sum;
    return error;
} 


bool gray_is_lost(Gray_TypeDef *gray)
{
    // 7个传感器全检测到黑线(1) → T字路口
    if ( gray->left3 && gray->left2 && gray->left1 && gray->middle && gray->right1 && gray->right2 && gray->right3 ) {
       return true; 
    }
    else {
       return false; 
    }
}

bool gray_is_on_line(Gray_TypeDef *gray)
{
    if (gray->left3 || gray->left2 || gray->left1 || gray->middle || gray->right1 || gray->right2|| gray->right3) {
        return true; 
    }
    else {
        return false; 
    }
}

static uint32_t last_T_line_time = 0; 
bool gray_is_T_cross(Gray_TypeDef *gray)
{
    // 检测到所有传感器为黑色
    if (gray->left2 == 1 && gray->left1 == 1 && gray->middle == 1 && gray->right1 == 1 && gray->right2 == 1) {
        // 当前出线状态是“白线”，判断是否持续了300ms
        if (HAL_GetTick() - last_T_line_time >= 200) { // 300ms 判断
            return true; // 持续出线，返回 true
        }
    }
    else {
        // 只要有线（任何传感器检测到黑线），更新出线时间
        last_T_line_time = HAL_GetTick();
    }
    return false; // 不满足 500ms 的持续出线，返回 false
}



