#line 1 "..\\flasher.c"
#line 1 "..\\flasher.h"











#line 1 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
 











 






































 









 
 
 



 



 
typedef enum IRQn
{
     

    NonMaskableInt_IRQn   = -14,     
    HardFault_IRQn        = -13,     
    SVCall_IRQn           = -5,      
    PendSV_IRQn           = -2,      
    SysTick_IRQn          = -1,      

     

    BOD_IRQn              = 0,       
    WDT_IRQn              = 1,       
    EINT0_IRQn            = 2,       
    EINT1_IRQn            = 3,       
    GPIO01_IRQn           = 4,       
    GPIO234_IRQn          = 5,       
    PWM_IRQn              = 6,       
    FB_IRQn               = 7,       
    TMR0_IRQn             = 8,       
    TMR1_IRQn             = 9,       
    UART_IRQn             = 12,      
    SPI_IRQn              = 14,      
    GPIO5_IRQn            = 16,      
    HIRC_IRQn             = 17,      
    I2C_IRQn              = 18,      
    ACMP_IRQn             = 25,      
    PDWU_IRQn             = 28,      
    ADC_IRQn              = 29       

} IRQn_Type;






 


 






   


#line 1 "..\\Library\\CMSIS\\Include\\core_cm0.h"
 




 

























 











#line 1 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdint.h"






 
#line 45 "..\\Library\\CMSIS\\Include\\core_cm0.h"

















 




 



 

 













#line 120 "..\\Library\\CMSIS\\Include\\core_cm0.h"



 







#line 162 "..\\Library\\CMSIS\\Include\\core_cm0.h"

#line 1 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 263 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"


#line 297 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"



 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 



__attribute__((always_inline)) static __inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;
  int32_t s = 4   * 8 - 1;  

  result = value;                       
  for (value >>= 1U; value; value >>= 1U)
  {
    result <<= 1U;
    result |= value & 1U;
    s--;
  }
  result <<= s;                         
  return(result);
}








 



#line 649 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"

   


 



 

#line 731 "..\\Library\\CMSIS\\Include\\cmsis_armcc.h"
 


#line 54 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"

 
#line 84 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"

   

#line 164 "..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 
#line 84 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 

#line 165 "..\\Library\\CMSIS\\Include\\core_cm0.h"
















 
#line 198 "..\\Library\\CMSIS\\Include\\core_cm0.h"

 






 
#line 214 "..\\Library\\CMSIS\\Include\\core_cm0.h"

 




 










 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:28;               
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:15;               
    uint32_t T:1;                         
    uint32_t _reserved1:3;                
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t _reserved0:1;                
    uint32_t SPSEL:1;                     
    uint32_t _reserved1:30;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 



 







 



 
typedef struct
{
  volatile uint32_t ISER[1U];                
        uint32_t RESERVED0[31U];
  volatile uint32_t ICER[1U];                
        uint32_t RSERVED1[31U];
  volatile uint32_t ISPR[1U];                
        uint32_t RESERVED2[31U];
  volatile uint32_t ICPR[1U];                
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  volatile uint32_t IP[8U];                  
}  NVIC_Type;

 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
        uint32_t RESERVED0;
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
        uint32_t RESERVED1;
  volatile uint32_t SHP[2U];                 
  volatile uint32_t SHCSR;                   
} SCB_Type;

 















 



























 















 









 






 



 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 








 
 







 






 







 


 







 

 










 









 


 



 





 

 
 









 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] = ((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  = ((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
  else
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FAUL << 16U) |
                 (1UL << 2U));
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 2) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 










#line 125 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\system_NUC029FAE.h"
 









 









 
 
 



 

#line 35 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\system_NUC029FAE.h"


extern uint32_t SystemCoreClock;         
extern uint32_t CyclesPerUs;             









 

extern void SystemCoreClockUpdate (void);








 
#line 126 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 127 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

 
 
 



 


#pragma anon_unions



 



 
typedef struct
{
    volatile uint32_t  CMPCR[2];       
    volatile uint32_t  CMPSR;          
    volatile uint32_t  CMPRVCR;        
} ACMP_T;

 





















 


















   


 




 
typedef struct
{
    


















































 
    volatile uint32_t  PWRCON;

    









 
    volatile uint32_t  AHBCLK;

    













































 
    volatile uint32_t  APBCLK;

    



















 
    volatile uint32_t  CLKSTATUS;

    



























 
    volatile uint32_t  CLKSEL0;

    








































 
    volatile uint32_t  CLKSEL1;

    












 
    volatile uint32_t  CLKDIV;

    














 
    volatile uint32_t  CLKSEL2;
    

 
    uint32_t  RESERVED0;

    














 
    volatile uint32_t  FRQDIV;
} CLK_T;

 















#line 498 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

#line 505 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"




 



 




































 



#line 560 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

#line 567 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

#line 574 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

 






 























 






 









 









   


 




 
typedef struct
{
    volatile uint32_t  ADDR;            
    uint32_t  RESERVED0[7];    
    volatile uint32_t  ADCR;            
    volatile uint32_t  ADCHER;          
    volatile uint32_t  ADCMPR[2];       
    volatile uint32_t  ADSR;            
    volatile uint32_t  ADTDCR;          
    volatile uint32_t  ADSAMP;          
} ADC_T;

 









 


















 






 


















 





















 



 



   


 




 
typedef struct
{
    




















































 
    volatile uint32_t ISPCON;

    









 
    volatile uint32_t ISPADR;

    









 
    volatile uint32_t ISPDAT;

    















 
    volatile uint32_t ISPCMD;

    











 
    volatile uint32_t ISPTRG;

    













 
    volatile const  uint32_t DFBADR;

} FMC_T;


 


















 









 




   

 



 

typedef struct
{
    volatile uint32_t  PMD;                         
    volatile uint32_t  OFFD;                        
    volatile uint32_t  DOUT;                        
    volatile uint32_t  DMASK;                       
    volatile const  uint32_t  PIN;                         
    volatile uint32_t  DBEN;                        
    volatile uint32_t  IMD;                         
    volatile uint32_t  IEN;                         
    volatile uint32_t  ISRC;                        
} GPIO_T;


typedef struct
{
    volatile uint32_t  DBNCECON;              
} GPIO_DBNCECON_T;

 
























 



 



 



 



 



 



 






 



 










typedef struct
{
    volatile uint32_t  GP_BIT0;       
    volatile uint32_t  GP_BIT1;       
    volatile uint32_t  GP_BIT2;       
    volatile uint32_t  GP_BIT3;       
    volatile uint32_t  GP_BIT4;       
    volatile uint32_t  GP_BIT5;       
    volatile uint32_t  GP_BIT6;       
    volatile uint32_t  GP_BIT7;       
} GPIOBIT_T;

   


 




 

typedef struct
{
    volatile uint32_t  I2CON;             
    volatile uint32_t  I2CADDR0;          
    volatile uint32_t  I2CDAT;            
    volatile uint32_t  I2CSTATUS;         
    volatile uint32_t  I2CLK;             
    volatile uint32_t  I2CTOC;            
    volatile uint32_t  I2CADDR1;          
    volatile uint32_t  I2CADDR2;          
    volatile uint32_t  I2CADDR3;          
    volatile uint32_t  I2CADM0;           
    volatile uint32_t  I2CADM1;           
    volatile uint32_t  I2CADM2;           
    volatile uint32_t  I2CADM3;           
    uint32_t  RESERVED0;         
    uint32_t  RESERVED1;         
    volatile uint32_t  I2CON2;            
    volatile uint32_t  I2CSTATUS2;        
} I2C_T;


 


















 






 



 



 



 









 



 















 















   

 




 

typedef struct
{
    volatile const  uint32_t IRQSRC[32];  
    volatile uint32_t NMICNO;      
    volatile uint32_t MCUIRQ;      

} INT_T;
   

 




 

typedef struct
{
    volatile uint32_t  PPR;                   
    volatile uint32_t  CSR;                   
    volatile uint32_t  PCR;                   
    volatile uint32_t  CNR[6];                
    volatile uint32_t  CMR[6];                
    uint32_t  RESERVED0[6];          
    volatile uint32_t  PIER;                  
    volatile uint32_t  PIIR;                  
    volatile uint32_t  POE;                   
    volatile uint32_t  PFBCON;                
    volatile uint32_t  PDZIR;                 
    volatile uint32_t  TRGCON0;               
    volatile uint32_t  TRGCON1;               
    volatile uint32_t  TRGSTS0;               
    volatile uint32_t  TRGSTS1;               
    volatile uint32_t  PHCHG;                 
    volatile uint32_t  PHCHGNXT;              
    volatile uint32_t  PHCHGMASK;             
    volatile uint32_t  INTACCUCTL;            
} PWM_T;

 









 


















 














































































 



 




 










































 










































 


















 




































 









 
















































 
























 
















































 
























 


























































































 


























































































 









 







   


 




 

typedef struct
{
    volatile uint32_t CNTRL;           
    volatile uint32_t DIVIDER;         
    volatile uint32_t SSR;             
    uint32_t RESERVED0;       
    volatile const  uint32_t RX;              
    uint32_t RESERVED1[3];    
    volatile  uint32_t TX;              
    uint32_t RESERVED2[6];    
    volatile uint32_t CNTRL2;          
    volatile uint32_t FIFO_CTL;        
    volatile uint32_t STATUS;          
} SPI_T;

 
















































 



 















 


















 
























 




































   



 




 
typedef struct
{
    








 
    volatile const  uint32_t  PDID;

    

































 
    volatile uint32_t  RSTSRC;

    
























 
    volatile uint32_t  IPRSTC1;

    

































 
    volatile uint32_t  IPRSTC2;

    




 
    uint32_t  RESERVED0[2];

    


































 
    volatile uint32_t  BODCTL;

    




 
    uint32_t  RESERVED1[5];

    





















































 
    volatile uint32_t  P0_MFP;

    















































 
    volatile uint32_t  P1_MFP;

    















































 
    volatile uint32_t  P2_MFP;

    

























































 
    volatile uint32_t  P3_MFP;

    
























 
    volatile uint32_t  P4_MFP;

    
















































 
    volatile uint32_t  P5_MFP;

    




 
    uint32_t  RESERVED3[14];

    

























 
    volatile uint32_t  IRCTRIMCTL;

    




















 
    volatile uint32_t  IRCTRIMIER;

    























 
    volatile uint32_t  IRCTRIMISR;

    




 
    uint32_t  RESERVED4[29];

    





















 
    volatile uint32_t  RegLockAddr;
} SYS_T;

 


















 






 



























 


















 









 









 









 









 









 









 






 






 









 



   



 



 

typedef struct
{
    volatile uint32_t  TCSR;        
    volatile uint32_t  TCMPR;       
    volatile uint32_t  TISR;        
    volatile const  uint32_t  TDR;         
    volatile const  uint32_t  TCAP;        
    volatile uint32_t  TEXCON;      
    volatile uint32_t  TEXISR;      
} TIMER_T;

 







































 



 






 



 



 
























 



   


 




 
typedef struct
{
    union
    {
        volatile const   uint32_t  RBR;          
        volatile   uint32_t  THR;          
    };
    volatile uint32_t  IER;           
    volatile uint32_t  FCR;           
    volatile uint32_t  LCR;           
    volatile uint32_t  MCR;           
    volatile uint32_t  MSR;           
    volatile uint32_t  FSR;           
    volatile uint32_t  ISR;           
    volatile uint32_t  TOR;           
    volatile uint32_t  BAUD;          
    volatile uint32_t  IRCR;          
    volatile uint32_t  ALT_CSR;       
    volatile uint32_t  FUN_SEL;       
} UART_T;

 



 



 






























 















 


















 










 










 







































 





































 






 












 









 















 




   

 




 


 
typedef struct
{
    volatile uint32_t  WTCR;           
} WDT_T;

 






























   



#pragma no_anon_unions


   




 
 






 
#line 3190 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

#line 3209 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

   





 
#line 3226 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

#line 3244 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

   




 

typedef volatile unsigned char  vu8;        
typedef volatile unsigned short vu16;       
typedef volatile unsigned long  vu32;       





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 



   

 
 
 



 











 
#line 3431 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"

 










   






 
 
 
#line 1 "..\\Library\\StdDriver\\inc\\sys.h"
 









 










 



 



 
 
 
 
#line 45 "..\\Library\\StdDriver\\inc\\sys.h"


 
 
 
#line 57 "..\\Library\\StdDriver\\inc\\sys.h"


 
 
 


























































































#line 158 "..\\Library\\StdDriver\\inc\\sys.h"



















































   



 
#line 226 "..\\Library\\StdDriver\\inc\\sys.h"

#line 234 "..\\Library\\StdDriver\\inc\\sys.h"


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
void SYS_LockReg(void);
void SYS_UnlockReg(void);
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);

   

   

   





#line 3454 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\clk.h"
 









 











 



 





 

 
 
 





 
 
 
#line 56 "..\\Library\\StdDriver\\inc\\clk.h"


 
 
 
#line 88 "..\\Library\\StdDriver\\inc\\clk.h"


 
 
 
#line 100 "..\\Library\\StdDriver\\inc\\clk.h"


 
 
 




 
 
 
#line 121 "..\\Library\\StdDriver\\inc\\clk.h"
 
 
 
#line 136 "..\\Library\\StdDriver\\inc\\clk.h"

   




 

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_SysTickDelay(uint32_t us);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);



   

   

   







 
#line 3455 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\acmp.h"
 









 











 



 



 

 
 
 
#line 64 "..\\Library\\StdDriver\\inc\\acmp.h"


   




 




























 
#line 109 "..\\Library\\StdDriver\\inc\\acmp.h"







 








 








 








 









 








 








 








 








 













 























 






 






 








 








 








 







 


void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum);

   

   

   







 
#line 3456 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\adc.h"
 









 











 



 



 

#line 56 "..\\Library\\StdDriver\\inc\\adc.h"

   




 











 








 











 











 











 










 










 







 







 
















 
#line 184 "..\\Library\\StdDriver\\inc\\adc.h"





 
















 
#line 216 "..\\Library\\StdDriver\\inc\\adc.h"





 










 







 







 


void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_SetExtraSampleTime(ADC_T *adc,
                            uint32_t u32ChNum,
                            uint32_t u32SampleTime);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);



   

   

   







 
#line 3457 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\fmc.h"
 









 










 



 



 
 
 
 









 
 
 
#line 53 "..\\Library\\StdDriver\\inc\\fmc.h"


   



 

#line 72 "..\\Library\\StdDriver\\inc\\fmc.h"


extern void FMC_SetBootSource(int32_t i32BootSrc);
extern void FMC_Close(void);
extern void FMC_DisableAPUpdate(void);
extern void FMC_DisableConfigUpdate(void);
extern void FMC_DisableLDUpdate(void);
extern void FMC_EnableAPUpdate(void);
extern void FMC_EnableConfigUpdate(void);
extern void FMC_EnableLDUpdate(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read (uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadPID(void);
extern uint32_t FMC_ReadUCID(uint32_t u32Index);
extern uint32_t FMC_ReadUID(uint32_t u32Index);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);


   

   

   








#line 3458 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\gpio.h"
 









 











 



 



 


 
 
 





 
 
 






 
 
 



 
 
 






#line 82 "..\\Library\\StdDriver\\inc\\gpio.h"














 
#line 144 "..\\Library\\StdDriver\\inc\\gpio.h"

   



 










 











 











 











 











 











 











 












 



















 










 











 










 













 












 














 












 



void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin);



   

   

   







 
#line 3459 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\i2c.h"
 









 











 



 




 






   




 






 






 






 






 






 






 






 






 






 






 






 






 






 







 






 








 








 






 


uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
void I2C_Close(I2C_T *i2c);
void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_GetData(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);

   

   

   







 
#line 3460 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\pwm.h"
 









 











 



 



 
#line 50 "..\\Library\\StdDriver\\inc\\pwm.h"

   




 






 







 







 







 







 







 









 
#line 122 "..\\Library\\StdDriver\\inc\\pwm.h"










 















 











 











 











 




uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequency,
                                 uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrake(PWM_T *pwm,
                          uint32_t u32ChannelMask,
                          uint32_t u32LevelMask,
                          uint32_t u32BrakeSource);
void PWM_ClearFaultBrakeFlag(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_DisableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_ClearFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
uint32_t PWM_GetFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);



   

   

   







 
#line 3461 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\spi.h"
 









 











 



 



 













#line 51 "..\\Library\\StdDriver\\inc\\spi.h"


   




 





 






 






 






 






 






 








 








 






 







 






 
static __inline void SPI_SET_SS_HIGH(SPI_T *spi)
{
    spi->SSR &= ~(1ul << 3);
    spi->SSR |= ((1ul << 5) | (1ul << 2) | (1ul << 0));
}





 
static __inline void SPI_SET_SS_LOW(SPI_T *spi)
{
    spi->SSR &= ~(1ul << 3);
    spi->SSR |= (1ul << 5);
    spi->SSR &= ~(1ul << 2);
    spi->SSR |= (1ul << 0);
}





 






 







 






 






 







 
static __inline void SPI_SET_DATA_WIDTH(SPI_T *spi, uint32_t u32Width)
{
    if(u32Width == 32)
        u32Width = 0;

    spi->CNTRL = (spi->CNTRL & ~(0x1Ful << 3)) | (u32Width << 3);
}







 






 


uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_EnableFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
void SPI_DisableFIFO(SPI_T *spi);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);

   

   

   







 
#line 3462 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\timer.h"
 









 











 



 



 

#line 48 "..\\Library\\StdDriver\\inc\\timer.h"

   




 







 









 









 










 








 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 30);
}





 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 30);
}






 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 23);
}





 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 23);
}






 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 6);
}





 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 6);
}






 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 7);
}





 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 7);
}





 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 29);
}





 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 29);
}





 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 5);
}





 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 5);
}







 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return(timer->TISR & (1ul << 0) ? 1 : 0);
}





 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 0);
}







 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->TEXISR;
}





 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->TEXISR = (1ul << 0);
}







 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 1) ? 1 : 0);
}





 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 1);
}





 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->TCAP;
}





 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->TDR;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);


   

   

   







 
#line 3463 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\uart.h"
 









 











 



 



 

 
 
 











 
 
 
















 
 
 



 
 
 





   




 








 









 











 









 










 








 









 









 








 









 










 











 











 









 









 


















 

















 




















 








 
__inline void UART_CLEAR_RTS(UART_T* uart)
{
    uart->MCR |= (1ul << 9);
    uart->MCR &= (1ul << 1);
}






 
__inline void UART_SET_RTS(UART_T* uart)
{
    uart->MCR |= (1ul << 9) | (1ul << 1);
}






 








 



void UART_ClearIntFlag(UART_T* uart, uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart );
void UART_DisableFlowCtrl(UART_T* uart );
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_EnableFlowCtrl(UART_T* uart );
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


   

   

   







 








#line 3464 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"
#line 1 "..\\Library\\StdDriver\\inc\\wdt.h"
 









 











 



 



 
#line 40 "..\\Library\\StdDriver\\inc\\wdt.h"

   




 




 





 





 








 








 








 







 



void  WDT_Open(uint32_t u32TimeoutInterval,
               uint32_t u32ResetDelay,
               uint32_t u32EnableReset,
               uint32_t u32EnableWakeup);
void WDT_Close(void);

void WDT_EnableInt(void);
void WDT_DisableInt(void);

   

   

   







 
#line 3465 "..\\Library\\Device\\Nuvoton\\NUC029FAE\\Include\\NUC029FAE.h"



#line 13 "..\\flasher.h"
#line 14 "..\\flasher.h"
#line 1 "..\\Library\\Sinyalizasyon\\inc\\buttonLib.h"








#line 10 "..\\Library\\Sinyalizasyon\\inc\\buttonLib.h"
#line 1 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdbool.h"



#line 11 "..\\Library\\Sinyalizasyon\\inc\\buttonLib.h"




	typedef enum
	{
		PULL_DOWN,
		PULL_UP,
	} pushPullMode_t;

	typedef enum
	{
		BUTTON_DEFAULT_STATE,
		BUTTON_CLICK,
		BUTTON_LONG_PRESS,
	} buttonState;

	typedef struct _button Button;
	struct _button
	{
		GPIO_T *_port;
		uint32_t _bit, _timePressed, _initialPressTime;
		volatile uint32_t *_pinPointer;
		_Bool _state, _longPressed, _sampled, _debounceTimeInMs, _longPressedOnPrevCall;
		uint8_t _ppMode;
		uint32_t (*readPin)(Button *);
		_Bool (*isPressed)(Button *);
		buttonState (*getButtonState)(Button *);
	};

	buttonState _getButtonState(Button *me);
	Button *attachButton(Button *me, GPIO_T *port, uint32_t bit, volatile uint32_t *pinPointer, pushPullMode_t ppMode, uint32_t debounceTimeInMs);
	uint32_t _readPin(Button *me);
	_Bool _isPressed(Button *me);

#line 15 "..\\flasher.h"
#line 1 "..\\Library\\Sinyalizasyon\\inc\\ticker.h"








#line 10 "..\\Library\\Sinyalizasyon\\inc\\ticker.h"
#line 1 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdio.h"



 

#line 11 "..\\Library\\Sinyalizasyon\\inc\\ticker.h"
#line 1 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"
 
 
 




 
 



 






   














  


 








#line 54 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"


  



    typedef unsigned int size_t;    
#line 70 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"






    



    typedef unsigned short wchar_t;  
#line 91 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { long long quot, rem; } lldiv_t;
    


#line 112 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"
   



 

   




 
#line 131 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 436 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 524 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 553 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
   











 
#line 634 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 892 "C:\\Keil_v5\\ARM\\ARM_Compiler_5.06u7\\Bin\\..\\include\\stdlib.h"





 
#line 12 "..\\Library\\Sinyalizasyon\\inc\\ticker.h"

    void startTicker(TIMER_T *timer);
    void stopTicker(TIMER_T *timer);
    uint32_t getTick(void);
    void tickerDelay(uint32_t delayMs);

#line 16 "..\\flasher.h"







#line 29 "..\\flasher.h"

	void flasherInit(void), stateHandler(void), blinkLed(uint32_t blinkIntervalinMs), buttonStateHandler(buttonState bState), configHandler(void);
	void stopFlash(void), flashLed(uint32_t flashDurationInSeconds), _readConfig(void), _clickHandler(void), _saveConfig(void);
	static int _set_data_flash_base(uint32_t u32DFBA);

	typedef enum _deviceState
	{
		MAIN,
		CONFIG,
	} DeviceState;

	typedef enum
	{
		DURATION_20S,
		DURATION_30S,
		DURATION_60S,
	} flashDuration_t;

#line 2 "..\\flasher.c"

const uint16_t flashDuration[] = {20000, 30000, 60000};
const uint8_t nrOfElements = sizeof(flashDuration) / sizeof(flashDuration[0]);

 
uint8_t flashDurationIndex = DURATION_30S, prevFlashDurationIndex = DURATION_30S;
_Bool flash = 0, flashed = 0;

Button button;
DeviceState deviceState = MAIN;




 
void flasherInit()
{





	attachButton(&button, ((GPIO_T *) (((uint32_t)0x50000000) + 0x04040)), (0x00000004), &(*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(1))) + ((2)<<2)))), PULL_DOWN, 150);
	GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04000)), (0x00000010), 0x1UL);
}




 
void stateHandler()
{
	switch (deviceState)
	{
	case MAIN:
		buttonStateHandler(button.getButtonState(&button));
		break;

	case CONFIG:
		configHandler();
		break;
	}
}





 
void blinkLed(uint32_t blinkIntervalinMs)
{
	static uint32_t ledPrevTick = 0;

	if (getTick() - ledPrevTick >= blinkIntervalinMs)
	{
		(*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(0))) + ((4)<<2)))) ^= 1;
		ledPrevTick = getTick();
	}
}





 
void buttonStateHandler(buttonState bState)
{
	switch (bState)
	{
	case BUTTON_CLICK:
		flash = 1;
		break;

	case BUTTON_LONG_PRESS:



			flash = 1;

		break;

	default:
		break;
	}

	if (flash)
		flashLed(flashDuration[flashDurationIndex]);
	else
		stopFlash();
}




 
void configHandler()
{
	(*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(0))) + ((4)<<2)))) = 1;

	switch (button.getButtonState(&button))
	{
	case BUTTON_CLICK:
		_clickHandler();
		break;

	case BUTTON_LONG_PRESS:
		_saveConfig();
		break;

	default:
		break;
	}

	if (prevFlashDurationIndex != flashDurationIndex)
	{
		for (uint8_t i = 0; i < (flashDurationIndex + 1); i++)
		{
			(*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(0))) + ((4)<<2)))) = 0;
			CLK_SysTickDelay(1000000);
			(*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(0))) + ((4)<<2)))) = 1;
			CLK_SysTickDelay(1000000);
		}
		prevFlashDurationIndex = flashDurationIndex;
	}
}





 
void flashLed(uint32_t flashDurationInSeconds)
{
	static uint32_t initialFlashTime = 0;

	if (!flashed)
	{
		(*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(0))) + ((4)<<2)))) = 1;
		initialFlashTime = getTick();
		flashed = 1;
	}

	if (getTick() - initialFlashTime >= flashDurationInSeconds)
	{
		stopFlash();
	}
}




 
void stopFlash()
{
	(*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(0))) + ((4)<<2)))) = 0;
	flash = 0;
	flashed = 0;
}

void _clickHandler()
{
	if (++flashDurationIndex == 3)
		flashDurationIndex = 0;



}

void _readConfig()
{
	SYS_UnlockReg();
	FMC_Open();
	_set_data_flash_base(0X3FFC);
	uint32_t temp = FMC_Read(0X3FFC);

	if (temp <= nrOfElements)
	{



		flashDurationIndex = temp;
		prevFlashDurationIndex = flashDurationIndex;
	}
	else
	{



	}
	FMC_Close();
	SYS_LockReg();
}

static int _set_data_flash_base(uint32_t u32DFBA)
{
	uint32_t au32Config[2];

	if (FMC_ReadConfig(au32Config, 2) < 0)
	{



		return -1;
	}

	if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
		return 0;

	FMC_EnableConfigUpdate();

	au32Config[0] &= ~0x1;
	au32Config[1] = u32DFBA;

	if (FMC_WriteConfig(au32Config, 2) < 0)
		return -1;




	
	((SYS_T *) (((uint32_t)0x50000000) + 0x00000))->IPRSTC1 = (1ul << 0);
	return 0;
}

void _saveConfig()
{
	stopFlash();
	SYS_UnlockReg();
	FMC_Open();
	for (uint32_t currentAddr = 0X3FFC; currentAddr < (0X3FFC + 4); currentAddr += 4)
	{
		FMC_Erase(0X3FFC);
		FMC_Write(0X3FFC, flashDurationIndex);
	}



	FMC_Close();
	SYS_LockReg();
	deviceState = MAIN;
}
