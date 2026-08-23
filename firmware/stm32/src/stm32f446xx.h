#ifndef STM32F446XX_H
#define STM32F446XX_H

#include <stdint.h>

#define __IO volatile

typedef struct {
  __IO uint32_t MODER;
  __IO uint32_t OTYPER;
  __IO uint32_t OSPEEDR;
  __IO uint32_t PUPDR;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t LCKR;
  __IO uint32_t AFR[2];
} GPIO_TypeDef;

typedef struct {
  __IO uint32_t CR;
  __IO uint32_t PLLCFGR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t AHB1RSTR;
  __IO uint32_t AHB2RSTR;
  __IO uint32_t AHB3RSTR;
  uint32_t RESERVED0;
  __IO uint32_t APB1RSTR;
  __IO uint32_t APB2RSTR;
  uint32_t RESERVED1[2];
  __IO uint32_t AHB1ENR;
  __IO uint32_t AHB2ENR;
  __IO uint32_t AHB3ENR;
  uint32_t RESERVED2;
  __IO uint32_t APB1ENR;
  __IO uint32_t APB2ENR;
  uint32_t RESERVED3[2];
  __IO uint32_t AHB1LPENR;
  __IO uint32_t AHB2LPENR;
  __IO uint32_t AHB3LPENR;
  uint32_t RESERVED4;
  __IO uint32_t APB1LPENR;
  __IO uint32_t APB2LPENR;
  uint32_t RESERVED5[2];
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
  __IO uint32_t SSCGR;
  __IO uint32_t PLLI2SCFGR;
  __IO uint32_t PLLSAICFGR;
  __IO uint32_t DCKCFGR;
  __IO uint32_t CKGATENR;
  __IO uint32_t DCKCFGR2;
} RCC_TypeDef;

typedef struct {
  __IO uint32_t SR;
  __IO uint32_t DR;
  __IO uint32_t BRR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t CR3;
  __IO uint32_t GTPR;
} USART_TypeDef;

#define GPIOA ((GPIO_TypeDef *)0x40020000UL)
#define RCC ((RCC_TypeDef *)0x40023800UL)
#define USART2 ((USART_TypeDef *)0x40004400UL)
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100UL)
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104UL)
#define NVIC_IPR ((volatile uint8_t *)0xE000E400UL)

#define RCC_AHB1ENR_GPIOAEN (1UL << 0U)
#define RCC_APB1ENR_USART2EN (1UL << 17U)
#define USART_SR_ORE (1UL << 3U)
#define USART_SR_RXNE (1UL << 5U)
#define USART_SR_TXE (1UL << 7U)
#define USART_CR1_RE (1UL << 2U)
#define USART_CR1_TE (1UL << 3U)
#define USART_CR1_RXNEIE (1UL << 5U)
#define USART_CR1_UE (1UL << 13U)

#endif
