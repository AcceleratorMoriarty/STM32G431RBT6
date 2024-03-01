# 一、时钟树

| 时钟源名称     | 频率      | 材料      | 用途       |
| -------------- | --------- | --------- | ---------- |
| 高速外部振荡器 | 4-48MHz() | 晶体/陶瓷 | SYSCLK/RTC |
| 低速外部振荡器 | 32.768KHz | 晶体/陶瓷 | RTC        |
| 高速内部振荡器 | 16MHz     | RC        | SYSCLK     |
| 低速内部振荡器 | 32KHz     | RC        | RTC/IWDG   |

![image-20240301153029963](Pic\image-20240301153029963.png)

RCC：复位与时钟控制（reset clock control）

![时钟树](Pic\snipaste20240301_153849.jpg)

![clockTree](Pic\clockTree.png)

# 二、 GPIO 寄存器（参考手册9.4）

+ GPIO port mode register (GPIOx_MODER)
+ GPIO port output type register (GPIOx_OTYPER)
+ GPIO port output speed register (GPIOx_OSPEEDR)
+ GPIO port pull-up/pull-down register (GPIOx_PUPDR)
+ GPIO port input data register (GPIOx_IDR) 端口输入数据寄存器
+ GPIO port output data register (GPIOx_ODR)端口输出数据寄存器
+ GPIO port bit set/reset register (GPIOx_BSRR)端口位设置/清除寄存器
+ GPIO port configuration lock register (GPIOx_LCKR) 端口位锁定寄存器
+ GPIO alternate function low register (GPIOx_AFRL) 
+ GPIO alternate function high register (GPIOx_AFRH) 
+ GPIO port bit reset register (GPIOx_BRR)端口位清除寄存器

