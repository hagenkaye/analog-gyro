;;	.device "ATtiny214" 
	.cseg
	.org	0x00
	
	
;; registers used by the program	
	.def    r_acc = r0
	.def    r_acc_b = r1
	.def    r_zero = r2             ; always zero
	.def    r_unprotect = r3        ; set to unprotect signature
	.def	r_status = r4			; holds status register during interupt
	
	.def    r_tmp = r16             ; temporary register
	.def    r_tmp_l = r16
	.def    r_tmp_h = r17
	.def	r_accz = r18			; accelerometer z value
	.def	r_accz_l = r18
	.def    r_accz_h = r19
	.def    r_counter = r20
	.def	r_twi_tmp = r21			; twi interface temp register
	.def	r_twi_addr = r22		; twi address 
	.def    r_twi_reg = r23			; twi register to read/write
	.def	r_twi_data = r24		; twi data read/write
	.def	r_twi_data_h = r25
	
	.def    rx = r26                ; temporary register
	.def    rx_l = r26
	.def    rx_h = r27
	
;; macros
    .macro  unlock
    out     CPU_CCP,r_unprotect
    .endmacro

	.macro	led_on
	sbi		VPORTA_DIR,1
	.endmacro

	.macro	led_off
	cbi		VPORTA_DIR,1
	.endmacro

	.macro	gyro_on
	sbi		VPORTA_DIR,5
	.endmacro

	.macro gyro_off
	cbi		VPORTA_DIR,5
	.endmacro
    

; reset and interupt vector table
    rjmp    power_on_reset          ; RESET 
    rjmp    not_implemented         ; CRCSCAN_NMI     
    rjmp    low_voltage             ; BOD_VLM
    rjmp    not_implemented         ; PORTA_PORT
    rjmp    not_implemented         ; PORTB_PORT
    rjmp    not_implemented         ; not used???
    rjmp    not_implemented         ; RTC_CNT
    rjmp    not_implemented         ; RTC_PIT
    rjmp    not_implemented         ; TCA0_LUNF / TCA0_OVF
    rjmp    not_implemented         ; TCA_HUNF
    rjmp    not_implemented         ; TCA0_LCMP0 / TCA_CMP0
    rjmp    not_implemented         ; TCA0_LCMP1 / TCA_CMP1
    rjmp    not_implemented         ; TCA0_LCMP2 / TCA_CMP2
    rjmp    not_implemented         ; TCB0_INT
    rjmp    not_implemented         ; TCD0_OVF
    rjmp    not_implemented         ; TCD0_TRIG
    rjmp    not_implemented         ; AC0_AC
    rjmp    adc_result_ready        ; ADC0_RESRDY
    rjmp    not_implemented         ; ADC0_WCOMP
    rjmp    not_implemented         ; TWI0_TWIS
    rjmp    not_implemented         ; TWI0_TWIM
    rjmp    not_implemented         ; SPI0_INT
    rjmp    not_implemented         ; USART0_RXC
    rjmp    not_implemented         ; USART0_DRE
    rjmp    not_implemented         ; USART0_TXC
    rjmp    not_implemented         ; NVMCTRL_EE
    
low_voltage:
    ldi     r_tmp_l,BOD_VLMIF_bm
    sts     BOD_INTFLAGS,r_tmp_l        ; clear bod interupt flag
    reti

adc_result_ready:
	in		r_status,CPU_SREG			; save status register
	lds		r_accz_l,ADC0_RESL			; get result of ADC conversion
	lds		r_accz_h,ADC0_RESH

	cpi		r_accz_h,0x44
	brge	tilt
	cpi		r_counter,0
	breq	counter_min
	dec		r_counter
	rjmp	adc_return
counter_min:
	led_off
	rjmp	adc_return

tilt:
	cpi		r_counter,0x7F
	breq	counter_max
	inc		r_counter
	rjmp	adc_return
counter_max:
	led_on

adc_return:
	ldi		r_tmp,0x01				   ; clear result ready flag
	sts		ADC0_INTFLAGS,r_tmp
    out     CPU_SREG,r_status          ; restore status register
	reti
    
not_implemented:
power_on_reset:
    eor     r_zero,r_zero 
    out     CPU_SREG,r_zero             ; clear status register
    ldi     r_tmp,0xD8                  ; for ccp unprotect registers
    mov     r_unprotect,r_tmp
	mov		r_counter,r_zero
    
    ldi     r_tmp_l,LOW(INTERNAL_SRAM_END)  ; set stack pointer to top of memory
    ldi     r_tmp_h,HIGH(INTERNAL_SRAM_END)
    out     CPU_SPL,r_tmp_l
    out     CPU_SPH,r_tmp_h
    
;; configure clock
;; The clock is set to use the internal 20Mhz clock, with a divide by 2
;; prescaler, so the CPU will be running at 10Mhz

    unlock
    sts     CLKCTRL_MCLKCTRLA,r_zero
    unlock
    ldi     r_tmp,0b00000001            ; clock prescaler enabled, div by 2
    sts     CLKCTRL_MCLKCTRLB,r_tmp   
    
;; brown out detection
;; generally disabled for now, we do set some values

    unlock
    sts     BOD_CTRLA,r_zero            ; disable bod detection
    unlock
    ldi     r_tmp,0x04
    sts     BOD_CTRLB,r_tmp             ; brown out level = 3.3V
    sts     BOD_VLMCTRLA,r_zero         ; bod level 5% above level
    sts     BOD_INTCTRL,r_zero          ; level goes below, interupts disabled

;; sleep control - disabled
    sts     SLPCTRL_CTRLA,r_zero
 
;; disable watch dog timer
    unlock
    sts     WDT_CTRLA,r_zero    

;; setup realtime clock 
;; 1Khz, no interupts
	ldi		r_tmp,0x01
	sts		RTC_CLKSEL,r_tmp
	sts		RTC_DBGCTRL,r_zero
	sts		RTC_INTCTRL,r_zero
wait_rtc:
	lds		r_tmp,RTC_STATUS
	and		r_tmp,r_tmp
	brne	wait_rtc
	ldi		r_tmp,0x01
	sts		RTC_CTRLA,r_tmp

;; Port A configuration
;; PA0 - not used (used as UPDI)
;; PA1 - Tilt LED output
;; PA2 - not used
;; PA3 - not used
;; PA4 - analog input z acceleromter
;; PA5 - Gyro power
;; PA6 - DAC output - analog gyro
;; PA7 - not used

    sts     PORTA_DIR,r_zero			; set all pins to input
    sts     PORTA_OUT,r_zero            ; outputs are set low, we set the direction to make it go low
    
    ldi     rx_l,LOW(PORTA_PIN0CTRL)
    ldi     rx_h,HIGH(PORTA_PIN0CTRL)
    st      x+,r_zero                   ; Pin A0 NO ISR
    st      x+,r_zero                   ; Pin A1 NO ISR
    st      x+,r_zero                   ; Pin A2 NO ISR
    st      x+,r_zero                   ; Pin A3 NO ISR
    st      x+,r_zero                   ; Pin A4 NO ISR
    st      x+,r_zero                   ; Pin A5 NO ISR
    st      x+,r_zero                   ; Pin A6 NO ISR
    st      x+,r_zero                   ; Pin A7 NO ISR
    
;; Port B configuration
;; PB0 - not used - TW0 SCL
;; PB1 - not used - TW0 SDA
;; PB2 - INT1 input from Gyro
;; PB3 - DRDY input from Gyro

    sts     PORTB_DIR,r_zero			; B0-B3 all inputs
    sts     PORTB_OUT,r_zero            ; set B0-B3 to low
    
    ldi     rx_l,LOW(PORTB_PIN0CTRL)
    ldi     rx_h,HIGH(PORTB_PIN0CTRL)
    st      x+,r_zero                   ; Pin B0 NO ISR
    st      x+,r_zero                   ; Pin B1 NO ISR
    st      x+,r_zero                   ; Pin B2 NO ISR
    st      x+,r_zero                   ; Pin B3 NO ISR
    
;; Port Multiplexer configuration
;; No alternate assignments, set everything to 0
    ldi     rx_l,LOW(PORTMUX_CTRLA)    
    ldi     rx_h,HIGH(PORTMUX_CTRLA)
    st      x+,r_zero                   ; PORTMUX CTRLA = 0
    st      x+,r_zero                   ; PORTMUX CTRLB = 0
    st      x+,r_zero                   ; PORTMUX CTRLC = 0
    st      x+,r_zero                   ; PORTMUX CTRLD = 0
    
;; Event Gen/User configuration
;; no events configured
    ldi     rx_l,LOW(EVSYS_ASYNCCH0)     
    ldi     rx_h,HIGH(EVSYS_ASYNCCH0)
    ldi     r_tmp,0x10                  ; async gen0 - Port A6
    st      x+,r_tmp
    st      x+,r_zero                   ; async gen1 - off
    st      x+,r_zero                   ; async gen2 - off
    st      x+,r_zero                   ; async gen3 - off
    st      x+,r_zero                   ; sync gen0 - off
    st      x+,r_zero                   ; sync gen1 - off
    ldi     rx_l,LOW(EVSYS_ASYNCUSER0)
    ldi     rx_h,HIGH(EVSYS_ASYNCUSER0)
    ldi     r_tmp,0x03                  ; TCB0 - use async gen0
    st      x+,r_tmp                    ; (this connects PA6 to TCB0)
    st      x+,r_zero                   ; ADC0 - no connection
    st      x+,r_zero                   ; CCL_LUT0EV0 - no connection
    st      x+,r_zero                   ; CCL_LUT1EV0 - no connection
    st      x+,r_zero                   ; CCL_LUT0EV1 - no connection
    st      x+,r_zero                   ; CCL_LUT1EV1 - no connection
    st      x+,r_zero                   ; TCD0_EV0 - no connection
    st      x+,r_zero                   ; TCD0_EV1 - no connection
    st      x+,r_zero                   ; EVOUT0 - no connection
    st      x+,r_zero                   ; EVOUT1 - no connection
    st      x+,r_zero                   ; EVOUT2 - no connection
    ldi     rx_l,LOW(EVSYS_SYNCUSER0)
    ldi     rx_h,HIGH(EVSYS_SYNCUSER0)
    st      x+,r_zero                   ; TCA0 - no connection
    st      x+,r_zero                   ; USART0 - no connection
    
;; set up internal voltage reference
	ldi		r_tmp,0x33					; ADC 4.3V, DAC 4.3V
	sts		VREF_CTRLA,r_tmp
    
;; set up ADC
	ldi		r_tmp,0x06					; accumulate 64 results
	sts		ADC0_CTRLB,r_tmp
	ldi		r_tmp,0b01000101			; reduced capcitance, internal reference, clock/64
	sts		ADC0_CTRLC,r_tmp
	sts		ADC0_CTRLD,r_zero			; no delays
	sts		ADC0_CTRLE,r_zero			; no comparator
	sts		ADC0_SAMPCTRL,r_zero		; no extension is sampling
	ldi		r_tmp,0x04					; mux = AIN4 (pin 2)
	sts		ADC0_MUXPOS,r_tmp
	sts		ADC0_EVCTRL,r_zero			; no event control
	ldi		r_tmp,0x01					; interrupt on result ready
	sts		ADC0_INTCTRL,r_tmp
	sts		ADC0_DBGCTRL,r_zero			; do not run in debug mode
	ldi		r_tmp,0b00000011			; free run mode, enable ADC
	sts		ADC0_CTRLA,r_tmp
	ldi		r_tmp,0x01					; start conversion
	sts		ADC0_COMMAND,r_tmp

;; set up DAC
	ldi		r_tmp,0xC1
	sts		DAC0_CTRLA,r_tmp
	ldi		r_tmp,0x80
	sts		DAC0_DATA,r_tmp

	rcall	gyro_reset
    
;; Interupt configuration
    out     CPU_CCP,r18
    sts     CPUINT_CTRLA,r_zero
    sts     CPUINT_LVL0PRI,r_zero
    sts     CPUINT_LVL1VEC,r_zero
    sei                                 ; enable interupts

loop:
	lds		r_twi_reg,PORTB_IN			; test for data ready on gyro
	andi	r_twi_reg,0x04
	breq	loop

	ldi		r_twi_reg,0xAA				; read in y axis 
	ldi		r_twi_data,0
	ldi		r_twi_data_h,0
	rcall	twi_read_register

	subi	r_twi_data_h,0x80			; normalize reading
	
	sts		DAC0_DATA,r_twi_data_h

    rjmp     loop                       ; wait for next gyro data

;; since the gyro doesn't have a reset pin, we'll power it off/on
gyro_reset:
	gyro_off
reset_rtc:
	lds		r_twi_tmp,RTC_STATUS		; rtc counts 1ms periods
	and		r_twi_tmp,r_twi_tmp
	brne	reset_rtc
	sts		RTC_CNTL,r_zero
	sts		RTC_CNTH,r_zero

	;; TWI reset
	sts		TWI0_MCTRLA,r_zero

	;; TWI configurre
	sts		TWI0_CTRLA,r_zero			; no setup, disable fsm+
	ldi		r_twi_tmp,0x01
	sts		TWI0_DBGCTRL,r_twi_tmp		; run in debug mode
	sts		TWI0_MCTRLB,r_zero
	ldi		r_twi_tmp,44				; set clock to 100khz
	sts		TWI0_MBAUD,r_twi_tmp
	ldi		r_twi_tmp,0b00000001		; turn on master
	sts		TWI0_MCTRLA,r_twi_tmp
	ldi		r_twi_tmp,0b00000001		; enter idle mode
	sts		TWI0_MSTATUS,r_twi_tmp	

wait_5_ms:
	lds		r_twi_tmp,RTC_CNTL			; sit here waiting 5 ms before powering gyro
	cpi		r_twi_tmp,5
	brne	wait_5_ms

	gyro_on

wait_20_ms:								; wait another 15ms before trying to connect via TWI
	lds		r_twi_tmp,RTC_CNTL
	cpi		r_twi_tmp,20
	brne	wait_20_ms

wait_for_gyro:
;; gyro address
	ldi		r_twi_addr,0xD4
;; read who ami register
	ldi		r_twi_reg,0x0F
	ldi		r_twi_data,0x00
	rcall	twi_read_register			; sanity check - should be 0xD7 for gyro
	cpi	    r_twi_data,0xD7
	brne	wait_for_gyro

	ldi		r_twi_reg,0x20
	ldi		r_twi_data,0xBA				; enable y gyro only - 400 smp/s 110hz filter
	rcall	twi_write_register
	ldi		r_twi_reg,0x21
	ldi		r_twi_data,0x00				; no trigger, no high pass filter
	rcall	twi_write_register
	ldi		r_twi_reg,0x22
	ldi		r_twi_data,0x08				; output data ready on int2
	rcall	twi_write_register
	ldi		r_twi_reg,0x23
	ldi		r_twi_data,0x00				; normal sensitivity 245dps
	rcall	twi_write_register
	ldi		r_twi_reg,0x24
	ldi		r_twi_data,0x00				; no fifo, no high pass
	rcall	twi_write_register
	ldi		r_twi_reg,0x2E
	ldi		r_twi_data,0x00				; fifo in bypass mode
	rcall	twi_write_register
	ldi		r_twi_reg,0x30		
	ldi		r_twi_data,0x00				; no interupt generation
	rcall	twi_write_register
	ldi		r_twi_reg,0x39
	ldi		r_twi_data,0x00				; low speed ddr disabled
	rcall	twi_write_register

	ret									; gyro has now been reset

    
;; twi functions
;; might be specific to the l3gd20h gyro
;; can write a 8 bit value to a register
;; can read a 8 bit or 16 bit value from a register

;; Write a byte value into a register on a twi device
;; r_twi_addr - contains twi address of device
;; r_twi_reg - contains the register id
;; r_twi_data - 8bit data to write
twi_write_register:
	andi	r_twi_addr,0xFE				; make sure lsb of addr is 0 for write operation
	sts		TWI0_MADDR,r_twi_addr		; start transaction
	rcall	twi_wait_for_hold			; wait for hold
	breq	twi_write_no_slave			; negative acknowledgement, no slave
	sts		TWI0_MDATA,r_twi_reg		; send register value
	rcall	twi_wait_for_hold
	breq	twi_write_slave_nack
	sts		TWI0_MDATA,r_twi_data		; send data
	rcall	twi_wait_for_hold

twi_write_slave_nack:
twi_write_no_slave:
	ldi		r_twi_tmp,0x03				; stop transaction
	sts		TWI0_MCTRLB,r_twi_tmp
	ret

;; Read a byte value from a register on a twi device
;; r_twi_addr - contains twi address of device
;; r_twi_reg - contains the register id
;; r_twi_data - 8bit data read
;; r_twi_data_h - optional high 8bits of 16bit read
twi_read_register:
	andi	r_twi_addr,0xFE				; make sure lsb of addr is 0 for write operation
	sts		TWI0_MADDR,r_twi_addr		; start transaction
	rcall	twi_wait_for_hold			; wait for hold
	breq	twi_read_no_slave			; negative acknowledgement, no slave
	sts		TWI0_MDATA,r_twi_reg
	rcall	twi_wait_for_hold
	breq	twi_read_neg_ack
	ori		r_twi_addr,0x01				; change direction of twi, now reading
	sts		TWI0_MADDR,r_twi_addr
	rcall	twi_wait_for_hold
	breq	twi_read_no_slave
	lds		r_twi_data,TWI0_MDATA		; read register 
	ldi		r_twi_tmp,0x02				; acknowledge and read next byte
	sts		TWI0_MCTRLB,r_twi_tmp
	rcall	twi_wait_for_hold
	lds		r_twi_data_h,TWI0_MDATA

twi_read_neg_ack:
twi_read_no_slave:
	ldi		r_twi_tmp,0x07				; stop transaction
	sts		TWI0_MCTRLB,r_twi_tmp
	ret

twi_wait_for_hold:
	lds		r_twi_tmp,TWI0_MSTATUS		; get status of bus
	andi	r_twi_tmp,0x03
	cpi		r_twi_tmp,0x03
	breq	twi_bus_busy
	lds		r_twi_tmp,TWI0_MSTATUS
	andi	r_twi_tmp,0x20
	breq	twi_wait_for_hold			; hold = 0, loop
	lds		r_twi_tmp,TWI0_MSTATUS
	andi	r_twi_tmp,0x10				
	cpi		r_twi_tmp,0x10              ; zero flag is set if
twi_bus_busy:							;  twi device responded with NACK
	ret									;  twi bus was busy
	

