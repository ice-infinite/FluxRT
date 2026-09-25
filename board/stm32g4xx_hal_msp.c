/*
 * FluxRT —— STM32G4 HAL 的 MSP（MCU Support Package）初始化钩子。
 * FluxRT - STM32G4 HAL MSP (MCU Support Package) init hooks.
 *
 * 职责 / Responsibility:
 *   - 提供 HAL 要求应用实现的底层初始化：HAL_MspInit() 的全局时钟与 PWR 设置，
 *     以及每个外设的 HAL_xxx_MspInit()；
 *   - 本工程只用到 LPUART1（板载控制台），HAL 的 UART 句柄由 RT-Thread 的串口驱动
 *     创建并调用这里的钩子，因此不需要 CubeMX 生成的 MX_xxx_Init() 文件。
 *   - Supplies the low-level init the HAL expects the application to implement: the global
 *     clock and PWR setup in HAL_MspInit() plus one HAL_xxx_MspInit() per peripheral. This
 *     project only uses LPUART1 (the on-board console); the HAL UART handle is created by
 *     the RT-Thread serial driver, which calls the hook here, so no CubeMX-generated
 *     MX_xxx_Init() file is needed.
 *
 * 边界 / Boundary:
 *   - 功率级外设（TIM1 / 注入组 ADC / 栅极驱动）**不**经过这里，它们由
 *     foc/platform/stm32g431/ 直接配置；
 *   - 本文件在调度器启动前或串口初始化时执行，不是实时路径。
 *   - The power-stage peripherals (TIM1, injected-group ADC, gate driver) never pass
 *     through here; foc/platform/stm32g431/ configures them directly. This file runs before
 *     the scheduler starts or during serial init, so it is not a realtime path.
 */

#include <drv_common.h>
#include <stm32g4xx_ll_pwr.h>

/*
 * HAL 全局初始化：由 HAL_Init() 调用一次，在 RT-Thread 启动流程的最前面。
 * Global HAL initialisation: called once by HAL_Init() at the very start of the RT-Thread
 * boot path.
 *
 * 为什么需要 / Why:
 *   - SYSCFG 时钟：用于外部中断线映射与部分模拟开关；
 *   - PWR 时钟 + 关闭死区电池充电：G431 的 VBAT 域带一个死区电池充电电路，
 *     NUCLEO 板上没有纽扣电池，使能它会在 PC13/PC14/PC15 相关引脚上叠加不需要的
 *     电压，因此显式关闭。
 *   - The SYSCFG clock is needed for EXTI line mapping and some analog switches. The PWR
 *     clock plus disabling the dead-battery charge is needed because the G431 VBAT domain
 *     has a dead-battery charging circuit, and the NUCLEO has no coin cell: leaving it on
 *     would put unwanted voltage on the PC13/PC14/PC15 pins.
 */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    LL_PWR_DisableDeadBatteryPD();
}

/*
 * LPUART1 的外设与引脚初始化：开启时钟并把 PA2/PA3 复用为 LPUART1_TX/RX。
 * Peripheral and pin init for LPUART1: enables the clocks and muxes PA2/PA3 as
 * LPUART1_TX/LPUART1_RX.
 *
 * 参数 / Parameters:
 *   uart - HAL 句柄；本工程只处理 Instance == LPUART1，其它实例直接返回。
 *          the HAL handle; only Instance == LPUART1 is handled and any other instance
 *          returns immediately.
 *
 * 上下文 / Context: 由 RT-Thread 串口驱动在打开设备时调用，非实时路径。
 * Called by the RT-Thread serial driver when the device is opened; not a realtime path.
 *
 * 注意 / Caveat:
 *   PA2/PA3 是**临时 bring-up 映射**（见 board/Kconfig 的 BSP_USING_LPUART1），与 NUCLEO
 *   板载 ST-LINK 虚拟串口不是同一对引脚；硬件定版后必须同步更新这里、Kconfig 与文档。
 *   PA2/PA3 is a temporary bring-up mapping (see BSP_USING_LPUART1 in board/Kconfig) and is
 *   not the NUCLEO's on-board ST-LINK virtual COM port pair; when the hardware is frozen
 *   this function, the Kconfig option and the documentation must change together.
 */
void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef gpio = {0};

    if (uart->Instance != LPUART1)
    {
        return;
    }

    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 临时 bring-up 控制台：PA2=LPUART1_TX, PA3=LPUART1_RX.
     * Temporary bring-up console: PA2=LPUART1_TX, PA3=LPUART1_RX.
     *
     * 上拉与低速的取舍 / Pull-up and low speed:
     *   RX/TX 共用一个初始化结构体，因此两脚都开内部上拉：空闲的 TX 线在悬空时可能被
     *   误认为起始位。控制台只有 115200，低速档足够且能减少边沿干扰；AF12 是 LPUART1
     *   在本封装上的唯一复用号。
     *   RX and TX share one init struct, so both pins get internal pull-ups: an idle,
     *   floating TX line can be mistaken for a start bit. The console runs at 115200 only,
     *   where the low-speed setting is sufficient and reduces edge noise; AF12 is the only
     *   LPUART1 alternate function on this package. */
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF12_LPUART1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/*
 * LPUART1 的反初始化：关时钟并把 PA2/PA3 还原为普通 GPIO。
 * De-initialisation for LPUART1: disables the clock and returns PA2/PA3 to plain GPIO.
 *
 * 与 HAL_UART_MspInit() 对称，只在串口设备被关闭时调用；此函数不涉及功率级，
 * 因此不存在"关掉控制台会留下已 arm 驱动"的问题（功率级的关断在平台层的 emergency
 * stop 路径里，与本文件无关）。
 * Symmetric with HAL_UART_MspInit() and only called when the serial device is closed. It
 * never touches the power stage, so losing the console cannot leave an armed driver behind;
 * the power-stage shutdown lives in the platform emergency-stop path, not here.
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *uart)
{
    if (uart->Instance != LPUART1)
    {
        return;
    }

    __HAL_RCC_LPUART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
}
