

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#include <stm32f1xx.h>
#pragma GCC diagnostic pop

#include <system_stm32f1xx.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

/* Part number: STM32F103C8T6 */

static volatile unsigned int sys_time;
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

/* 16px by 115px */

#define NBLOCKS 8
#define NCOLS   115
uint8_t module_sizes[5] = {25, 20, 20, 25, 25};
union framebuf_t {
    struct {
        uint16_t cols[8];
    } blocks[NBLOCKS];
    uint16_t cols[NCOLS];
};
union framebuf_t framebuf;

uint32_t get_tick() { return SysTick->VAL; }

enum Command {
    NOCMD_IDLE=0,
    CMD_PING=0x01,
    CMD_PIXEL_BLOCK=0x23,
    CMD_STROBE=0x42,
} cmd = NOCMD_IDLE;

enum GlobalOp {
    OP_IDLE,
    OP_UPDATE,
} global_op = OP_IDLE;

void USART1_IRQHandler() {
    int isr = USART1->SR;
    int data = USART1->DR;

    /* Overrun detected? */
    if (isr & USART_SR_ORE) {
        return;
    }

    if (cmd == NOCMD_IDLE) {
        switch (data) {
        case CMD_PIXEL_BLOCK:
            cmd = data;
            break;
        case CMD_STROBE:
            global_op = OP_UPDATE;
        default:
            cmd = NOCMD_IDLE;
            break;
        }
    } else {
        switch (cmd) {
        case CMD_PIXEL_BLOCK:
            if (data >= COUNT_OF(framebuf.blocks))
                break;
            /* Disable this RX interrupt for duration of DMA transfer */
            USART1->CR1 &= ~USART_CR1_RXNEIE_Msk;
            /* Enable DMA transfer to write buffer */
            DMA1->IFCR |= DMA_IFCR_CGIF3;
            DMA1_Channel5->CMAR = (unsigned int)(framebuf.blocks[data].cols); /* FIXME */
            DMA1_Channel5->CCR |= DMA_CCR_EN;
            USART1->CR3 |= USART_CR3_DMAR;
            break;
        default:
            cmd = NOCMD_IDLE;
        }
    }
}

void DMA1_Channel5_IRQHandler() {
    /* DMA Transfer complete */
    /* ...and disable this DMA channel */
    USART1->CR3 &= ~USART_CR3_DMAR_Msk;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN_Msk;
    /* Kick off formatting code in main loop outside interrupt context */
    DMA1->IFCR |= DMA_IFCR_CGIF5;
    /* re-enable receive interrupt */
    USART1->SR &= ~USART_SR_RXNE_Msk;
    USART1->CR1 |= USART_CR1_RXNEIE;
    cmd = NOCMD_IDLE;
}

void uart_config(void) {
    USART1->CR1 = /* 8-bit -> M clear */
          USART_CR1_RXNEIE
        | USART_CR1_TE
        | USART_CR1_RE;
    USART1->BRR = 0x1A0; /* 115.2kBd @48.0Mhz */
    USART1->CR1 |= USART_CR1_UE;

    /* Configure DMA for USART frame data reception */
    DMA1_Channel5->CPAR = (unsigned int)&USART1->DR;
    DMA1_Channel5->CNDTR = sizeof(framebuf.blocks[0]);
    DMA1_Channel5->CCR = (0<<DMA_CCR_PL_Pos);
    DMA1_Channel5->CCR |=
          (0<<DMA_CCR_MSIZE_Pos) /* 8 bit */
        | (0<<DMA_CCR_PSIZE_Pos) /* 8 bit */
        | DMA_CCR_MINC
        | DMA_CCR_TCIE
        | DMA_CCR_CIRC;

    DMA1_Channel4->CPAR = (unsigned int)&CRC->DR;
    //DMA1_Channel4->CMAR = (unsigned int)rx_buf; FIXME
    DMA1_Channel4->CCR = (0<<DMA_CCR_PL_Pos);
    DMA1_Channel4->CCR |=
          DMA_CCR_MEM2MEM /* Software trigger (precludes CIRC) */
        | DMA_CCR_DIR /* Read from memory */
        | (0<<DMA_CCR_MSIZE_Pos) /* 8 bit */
        | (0<<DMA_CCR_PSIZE_Pos) /* 8 bit */
        | DMA_CCR_MINC;

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 4);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    NVIC_SetPriority(DMA1_Channel5_IRQn, 3);
}

void set_col(uint16_t val) {
    GPIOA->ODR = (GPIOA->ODR & ~0x00FF) | (val&0xFF);
    GPIOB->ODR = (GPIOB->ODR & ~0xFF00) | (val>>8);
}

/* not very accurate */
void delay_us(int val) {
    while (val--) {
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    }
}

void clr(bool val) {
    if (val)
        GPIOB->BSRR |= 1<<3;
    else
        GPIOB->BRR |= 1<<3;
}

void clk(bool val) {
    if (val)
        GPIOA->BSRR |= 1<<8;
    else
        GPIOA->BRR |= 1<<8;
}

void rst(bool val) {
    if (val)
        GPIOA->BSRR |= 1<<9;
    else
        GPIOA->BRR |= 1<<9;
}

void sel(int val) {
    GPIOA->ODR = (GPIOA->ODR & ~(0x1F<<10)) | (val<<10);
}

void module_init(int module) {
    set_col(0x0000); clr(0); clk(0); rst(0); sel(1<<module);
    delay_us(10);  /*******************************/

    set_col(0xFFFF);
    delay_us(200); /*******************************/

    set_col(0x0000); clr(1); clk(0);
    delay_us(100); /*******************************/

                                     rst(1); sel(0);
    delay_us(200); /*******************************/

                     clr(0); clk(1);
    delay_us(20);  /*******************************/

                             clk(0);         sel(1<<module);
    delay_us(30);  /*******************************/

                     clr(1);
}

#define FLIP_TIME_US 800
void tx_row(uint16_t data, int delay, int module) {
    delay_us(delay); /*******************************/

                        clr(0); clk(0); rst(1); sel(1<<module);
    delay_us(10);    /*******************************/

                                clk(1);
    delay_us(10);    /*******************************/

    set_col(data);              clk(0);
    delay_us(delay); /*******************************/

    set_col(0x0000);    clr(1);        
    delay_us(10);    /*******************************/

                        clr(0); clk(1);
    delay_us(10);    /*******************************/

                                clk(0);
    delay_us(10);    /*******************************/

                        clr(1);        
}

int main(void) {
    /* External crystal: 8MHz */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR&RCC_CR_HSERDY));

    /* Sysclk = HCLK = 48MHz */
    RCC->CFGR = (4<<RCC_CFGR_PLLMULL_Pos) | RCC_CFGR_PLLSRC | (4<<RCC_CFGR_PPRE1_Pos) | (0<<RCC_CFGR_PPRE2_Pos) | (0<<RCC_CFGR_HPRE_Pos);
    //RCC->CFGR = (0<<RCC_CFGR_PPRE1_Pos) | (0<<RCC_CFGR_PPRE2_Pos) | (0<<RCC_CFGR_HPRE_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR&RCC_CR_PLLRDY));

    /* Switch to HSE */
    RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);
    //RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE1_Msk & ~RCC_CFGR_PPRE2_Msk))
    //    | (4<<RCC_CFGR_PPRE1_Pos) | (0<<RCC_CFGR_PPRE2_Pos);
    SystemCoreClockUpdate();

    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;

    inline void config_pin_output(GPIO_TypeDef *gpio, int pin) {
        if (pin < 8) {
            gpio->CRL &= ~(0xf<<(pin*4));
            gpio->CRL |= (0x1<<(pin*4)); /* push/pull general purpose output */
        } else {
            gpio->CRH &= ~(0xf<<((pin-8)*4));
            gpio->CRH |= (0x1<<((pin-8)*4)); /* push/pull general purpose output */
        }
    }

    /* UART IOs. TX -> PB6, RX -> PB7 */
    GPIOB->CRL = (GPIOB->CRL & 0x00FFFFFF) | 0x89000000;
    GPIOB->ODR |= (1<<7);
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

    config_pin_output(GPIOC, 13); /* LED */
//  config_pin_output(GPIOB, 3); /* CLEAR MOSFET */
//  /* Display IOs */
//  for (int pin=0; pin<8; pin++) /* Y0-7 -> PA0-7 */
//      config_pin_output(GPIOA, pin);
//  for (int pin=8; pin<16; pin++) /* Y8-15 -> PB8-15 */
//      config_pin_output(GPIOB, pin);
//  for (int pin=8; pin<15; pin++) /* aux -> PA8-14 */
//      config_pin_output(GPIOA, pin);

    uart_config();

    SysTick_Config(SystemCoreClock/1000); /* 1ms interval */
    while (42) {
        if (global_op == OP_UPDATE) {
            global_op = OP_IDLE;
            uint16_t *dp = framebuf.cols;
            for (int module=0; module<COUNT_OF(module_sizes); module++) {
                module_init(module);
                for (int col=0; col<module_sizes[module]; col++) {
                    tx_row(*dp++, FLIP_TIME_US, module);
                }
            }
        }
    }
}

void NMI_Handler(void) {
}

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler() {
    asm volatile ("bkpt");
}

void SVC_Handler(void) {
}


void PendSV_Handler(void) {
}

void SysTick_Handler(void) {
    sys_time++;
}

void _init(void) {
}

void MemManage_Handler(void) __attribute__((naked));
void MemManage_Handler() {
    asm volatile ("bkpt");
}

void BusFault_Handler(void) __attribute__((naked));
void BusFault_Handler() {
    asm volatile ("bkpt");
}