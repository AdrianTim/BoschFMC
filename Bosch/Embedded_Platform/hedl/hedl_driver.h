#ifndef HEDL_DRIVER_H
#define HEDL_DRIVER_H

#include <mbed.h>
#include <TaskManager.h>


// HEDL 5540 driver 
//  using   - PA0 and PA1 pins
//          - TIM2 timer
class CHEDLDriver{
    public:
        CHEDLDriver(){
            initialize();
        }
        void resetCounter(){
            TIM4->CNT=0;
        }
        
        int32_t getPosition(){
            int32_t l_position=TIM4->CNT;
            return l_position;
        }
    private:
        void initialize(){
            // configure GPIO PB
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable clock for GPIOB
                            
            GPIOB->MODER   |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 ;           //PB6 & PB7 as Alternate Function   /*!< GPIO port mode register,              */
            GPIOB->OTYPER  |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 ;                 //PB6 & PB7 as Inputs               /*!< GPIO port output type register,       */
            GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7 ;     // Low speed                        /*!< GPIO port output speed register,      */
            GPIOB->PUPDR   |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0 ;           // Pull Down                        /*!< GPIO port pull-up/pull-down register, */
            GPIOB->AFR[0]  |= 0x22000000 ;                                          //  AF01 for PB6 & PB7              /*!< GPIO alternate function registers,    */
            GPIOB->AFR[1]  |= 0x00000000 ;                                          //                                  /*!< GPIO alternate function registers,    */
        
            // configure TIM4 as Encoder input
            RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  // Enable clock for TIM4
            TIM4->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
            TIM4->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
            // TIM4->CCMR1 = 0xF1F1;     
            TIM4->CCMR1 |= TIM_CCMR1_CC1S_1 | TIM_CCMR1_CC2S_1;
            // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
            TIM4->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
            TIM4->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
            TIM4->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
            TIM4->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register
            
            TIM4->CNT = 0x0000;  //reset the counter before we use it  
        }
    
};


class CHEDLTask:public CTask{
    public:
        CHEDLTask(  uint32_t                 f_period
                    ,float                   f_period_s
                    ,uint                    f_ppr
                    ):CTask(f_period)
                    ,m_period_s(f_period_s)
                    ,m_ppr(f_ppr)
                    ,m_hedl()
                    ,m_rps(0)
                    {}
        float getRPS() {return m_rps;}

    private:
        virtual void _run(){
            int16_t l_relativposition=m_hedl.getPosition();
            float l_rps=static_cast<float>(l_relativposition)/m_ppr/m_period_s;
            m_rps=l_rps;
            m_hedl.resetCounter();
        }

        float        m_period_s;
        uint32_t     m_ppr;//pulse per revolution
        CHEDLDriver  m_hedl;
        float        m_rps;//rotation per second
        
};



#endif