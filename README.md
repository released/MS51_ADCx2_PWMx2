# MS51_ADCx2_PWMx2
 MS51_ADCx2_PWMx2

update @ 2020/04/15

update MS51 driver to Keil_V1.00.003 , by test in MS51 TSSOP20 EVM

- initial 2xPWM with freq : 1K , initial as 30% , 65% duty
	
	- PWM1 : PWM0_CH4 , P0.1
	
	- PWM2 : PWM0_CH3 , P0.0

	- capture waveform as below 

![image](https://github.com/released/MS51_ADCx2_PWMx2/blob/master/PWM_CH4_CH3.jpg)
	
- initial 2xTIMER , with 1ms

	- use GPIO P1.3 , P1.4 with 500ms and 1000ms
	
	- capture waveform as below 

![image](https://github.com/released/MS51_ADCx2_PWMx2/blob/master/TIMER0_TIMER1.jpg)
	
- initial 2xADC , use drop and average method , display log message as below 

	- ADC1 : ADC_CH6 , P0.3
	
	- ADC2 : ADC_CH5 , P0.4

![image](https://github.com/released/MS51_ADCx2_PWMx2/blob/master/ADC_CH6_CH5.jpg)

