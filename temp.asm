;
; temp_meas.asm
;
; Created: 11/18/2020 9:14:23 PM
; Author : Frankie
;


.nolist
.include "m4809def.inc"
.list

.dseg

cel_temp: .byte 2
far_temp: .byte 2

bcd_entries: .byte 4
hex_results: .byte 4
led_display: .byte 4

digit_num: .byte 1
.cseg					;start of code segment

.equ PERIOD_EXAMPLE_VALUE = 50 ;325
reset:
 	jmp start			;reset vector executed a power on
.org TCA0_OVF_VECT
	jmp toggle_pin_ISR

.org ADC0_RESRDY_vect
	jmp adc0_resrdy_ISR


start:
	;ldi r16, 0x20
	ldi r16,0x10
	sts VREF_CTRLA, r16

	;ldi XH, HIGH(PORTE_PIN1CTRL) ;X points to PORTA_PIN0CTRL
	;ldi XL, LOW(PORTE_PIN1CTRL)
	
	ldi r16,0x04
	sts PORTE_PIN3CTRL,r16

	ld r16,X
	ori r16,0x00

	ldi r16,0x45
	sts ADC0_CTRLC,r16

	;ldi r16,0x09 ;PE1 Analog Input
	ldi r16,0x0B; PE3 Analog Input
	sts ADC0_MUXPOS,r16

	ldi r16,0x01
	sts ADC0_CTRLA,r16

	sts ADC0_INTCTRL,r16
	
	;sts ADC0_INTFLAGS,r16
	
	;configure TCA0
	ldi r16, TCA_SINGLE_WGMODE_NORMAL_gc ;WGMODE normal
	sts TCA0_SINGLE_CTRLB, r16

	ldi r16, TCA_SINGLE_OVF_bm    ;enable overflow interrupt 
	sts TCA0_SINGLE_INTCTRL,r16

	ldi r16, LOW(PERIOD_EXAMPLE_VALUE) ;enable overflow interupt
	sts TCA0_SINGLE_PER, r16
	ldi r16, HIGH(PERIOD_EXAMPLE_VALUE)
	sts TCA0_SINGLE_PER + 1 , r16

	ldi r16, TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm
	sts TCA0_SINGLE_CTRLA,r16

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	ldi r16, 0xFF  ;Loads all ones into r16
	out VPORTC_DIR, r16 ;Sets direction of c to output
	out VPORTD_DIR,  r16 ;Sets direction of d to output

	ldi r16,0x00 ;Loads all zeros into r16
	out VPORTA_DIR,r16 ;Sets direciton of e to input
	ldi r16,0x00 ;Loads all zeros into r16
	out VPORTA_DIR,r16 ;Sets direciton of A to input
	;out VPORTE_DIR,r16 ;Sets direciton of E to input


	ldi r17,0x00; Loads 0 into bcd_entries
		
	ldi YL,LOW(bcd_entries)
	ldi YH,HIGH(bcd_entries)

	sts bcd_entries+0,r17
	sts bcd_entries+1,r17
	sts bcd_entries+2,r17
	sts bcd_entries+3,r17

	
	ldi XL, LOW(digit_num)
	ldi XH, HIGH(digit_num)
	ldi r16,0x01
	sts digit_num,r16

	ldi YL, LOW(led_display)  ;Sets pointers to byte led_display
	ldi YH, HIGH(led_display)

	;ldi r16,0xFF; ;OAVR
	ldi r16,0x00 ;SAVR
	sts led_display+0,r16
	sts led_display+1,r16
	sts led_display+2,r16
	sts led_display+3,r16
	
	sei

	rcall post_display
		
	ldi r16,0x01
	sts ADC0_COMMAND,r16
					
main_loop:
	rjmp main_loop

post_display:
	;ldi r16, 0xFF ;SAVR
	ldi r16,0x00 ;OAVR

	sts led_display+0,r16
	sts led_display+1,r16
	sts led_display+2,r16
	sts led_display+3,r16
	rcall one_sec_delay
	rcall turn_off_display

	;ldi r16, 0x00 ;SAVR
	ldi r16,0xFF ;OAVR
	sts led_display+0,r16
	sts led_display+1,r16
	sts led_display+2,r16
	sts led_display+3,r16
	ret




adc0_resrdy_ISR:	
	rcall poll_analog_input
	rcall update_hex_values
	rcall update_led_display_values

	lds r16, ADC0_RESL
	lds r17, ADC0_RESH
	ldi r16,0x01
	sts ADC0_COMMAND,r16
	reti

multiply_by_1100:
	;ldi r18,0xC4
	;ldi r19,0x09
	ldi r18,0x4C
	ldi r19,0x04
	
	rcall mpy16u
	ret

divide_by_1024:
	lsl r20 ;reg b
	lsl r20
	lsr r19 ;reg c
	lsr r19

	mov r21,r20
	andi r21,0b00001111
	lsl r21
	lsl r21
	lsl r21
	lsl r21

	add r19,r21

	lsr r20
	lsr r20
	lsr r20
	lsr r20

	ret

convert_cels_to_faren:
	lds r16,cel_temp+0
	lds r17,cel_temp+1

	ldi r18,0x09
	ldi r19,0x00

	rcall mpy16u

	mov r17,r19
	mov r16,r18
	ldi r19,0x00
	ldi r18,0x05

	rcall div16u

	ldi r22,0x20

	add r16,r22

	sts far_temp+0,r16
	sts far_temp+1,r17

	ret

cel_to_hex_results:

	lds r19,cel_temp+0
	lds r20,cel_temp+1

	sts hex_results+2,r20
	
	mov r16,r19
	andi r16,0b11110000
	lsr r16
	lsr r16
	lsr r16
	lsr r16

	sts hex_results+1,r16

	mov r16,r19
	andi r16,0b00001111

	sts hex_results+0,r16

	ldi r16,0x00
	sts hex_results+3,r16

	mov r16,r19
	mov r17,r20
	rcall bin16_to_BCD	
	mov r16,r22
	andi r16,0b00001111
	sts bcd_entries+0,r16
	mov r16,r22
	andi r16,0b11110000
	lsr r16
	lsr r16
	lsr r16
	lsr r16
	sts bcd_entries+1,r16
	mov r16,r23
	andi r16,0b00001111
	sts bcd_entries+2,r16
	mov r16,r23
	andi r16,0b11110000
	lsr r16
	lsr r16
	lsr r16
	lsr r16
	sts bcd_entries+3,r16

	ret

far_to_hex_results:

	lds r19,far_temp+0
	lds r20,far_temp+1

	sts hex_results+2,r20
	
	mov r16,r19
	andi r16,0b11110000
	lsr r16
	lsr r16
	lsr r16
	lsr r16

	sts hex_results+1,r16

	mov r16,r19
	andi r16,0b00001111

	sts hex_results+0,r16

	ldi r16,0x00
	sts hex_results+3,r16

	mov r16,r19
	mov r17,r20
	rcall bin16_to_BCD	
	mov r16,r22
	andi r16,0b00001111
	sts bcd_entries+0,r16
	mov r16,r22
	andi r16,0b11110000
	lsr r16
	lsr r16
	lsr r16
	lsr r16
	sts bcd_entries+1,r16
	mov r16,r23
	andi r16,0b00001111
	sts bcd_entries+2,r16
	mov r16,r23
	andi r16,0b11110000
	lsr r16
	lsr r16
	lsr r16
	lsr r16
	sts bcd_entries+3,r16

	ret


;is_higher_then_01:
	;cpi r20,0x01
	;brsh increment
	;ret

is_higher_then_F4:
	cpi r19,0xF4
	brsh increment1	
	ret

increment1:
	inc r24
	ret

increment2:
	inc r25
	lsl r25
	ret

subtract_500:
	ldi r17,0x0E
	ldi r16,0x0C
	;ldi r17,0x0D
	;ldi r16,0x4C
	
	add r20, r17
	andi r20,0b00001111

	add r19, r16

	ret	

n500_from_subtract:
	ldi r17,0x01
	ldi r16,0xF4

	sub r17,r20
	sub r16,r19

	mov r19,r16
	mov r20,r17
	ret

what_to_subtract:
	cpi r24,0x03
	brsh subtract_500  
		
exit:
 ret

poll_analog_input:	
	lds r16,ADC0_INTFLAGS
	cpi r16, 0x01
	brne exit

	lds r16, ADC0_RESL
	lds r17, ADC0_RESH	

	rcall multiply_by_1100
	rcall divide_by_1024
	rcall subtract_500

	;rcall is_higher_then_F4
	;rcall is_higher_then_01
	;add r24,r25
	;rcall what_to_subtract
	
	sts cel_temp+0,r19
	sts cel_temp+1,r20	
	rcall convert_cels_to_faren

	rcall cel_to_hex_results
	;rcall far_to_hex_results
	ret

update_hex_values:
	lds r17, bcd_entries+0
	lds r19, bcd_entries+1
	lds r20, bcd_entries+2
	lds r21, bcd_entries+3
	ret
	
multiplex_display:	
	ldi YL, LOW(led_display)  ;Sets pointers to byte led_display
	ldi YH, HIGH(led_display)		
	lds r16, digit_num		  ;loads the value of digitnum into r16
	add YL,r16		  ;adds the the value of digit num onto the position of of ledsdisplay
	ldi r17,0
	adc YH, r17;overflow lower lower addition
	lds r17,led_display	      ;loads value from led_display into r17
	cpi r16 , 0 
	breq drive_0 //turn on digit 0
	cpi r16, 1
	breq drive_1

	cpi r16,2
	breq drive_2

	cpi r16,3
	breq drive_3
	ret	

drive_0:
	;sbi VPORTC_OUT,4 ;Change OAVR
	cbi VPORTC_OUT,4 ;SAVR

	lds r17,led_display+0

	;com r17 ;OAVR
	out VPORTD_OUT,r17
	;sbi VPORTC_OUT,4
	;com r17 ;OAVR
	lds r16, digit_num
	inc r16
	sts digit_num,r16
	ret

drive_1:
	;sbi VPORTC_OUT,5 ;Change OAVR
	cbi VPORTC_OUT,5 ;SAVR

	lds r17,led_display+1

	;com r17 ;OAVR
	out VPORTD_OUT,r17
	;sbi VPORTC_OUT,5
	;com r17 ;OAVR
	lds r16, digit_num
	inc r16
	sts digit_num,r16

	ret
drive_2:
	;sbi VPORTC_OUT,6 ;Change OAVR
	cbi VPORTC_OUT,6 ;SAVR
	
	lds r17,led_display+2

	;com r17 ;OAVR
	out VPORTD_OUT,r17
	;com r17 ;OAVR
	;sbi VPORTC_OUT,6
	lds r16, digit_num
	inc r16
	sts digit_num,r16
	ret

drive_3:
	;sbi VPORTC_OUT,7 ; Change OAVR
	cbi VPORTC_OUT,7 ;SAVR

	lds r17,led_display+3

	;com r17 ;OAVR
	out VPORTD_OUT,r17
	;com r17 ;OAVR
	
	ldi YL, LOW(led_display)  ;Sets pointers to byte led_display
	ldi YH, HIGH(led_display)

	ldi XL, LOW(digit_num)
	ldi XH, HIGH(digit_num)

	ldi r16,0x00
	sts digit_num,r16
	ret
	
update_led_display_values:

	ldi r16,0x00
	ldi r18,0x00

	ldi YL,LOW(led_display) ;Sets pointer to led_display
	ldi YH,HIGH(led_display)
			
		
	mov r18,r17
	rcall hex_to_7seg  ;calls subroutine
	ldi r16,0b10000000
	add r18,r16
	sts led_display+0,r18;stores new value in led_display
	;mov r18,r22 ;moves value 1 to r18 to be used in hex_to_7seg
	
	mov r18,r19
	rcall hex_to_7seg ;calls subroutine
	sts led_display+1,r18;stores new value in led_display
	;mov r18,r17  ;moves value 2 to r18 to be used in hex_to_7seg

	mov r18,r20
	rcall hex_to_7seg ;calls subroutine
	ldi r16,0b10000000
	add r18,r16
	sts led_display+2,r18;stores new value in led_display ;stores in led_display
	;mov r18,r20 ;moves value 3 to r18 to be used in hex_to_7seg
	;mov r18,r21
	
	mov r18,r21
	rcall hex_to_7seg ;calls subroutin
	ldi r16,0b10000000
	add r18,r16
	sts led_display+3,r18       ;stores new value in led_display ;stores in led_display

	ldi r21,0x00	
		
	ret

turn_off_display:
	;ldi r16,0x00 ;OAVR
	ldi r16, 0XFF ;SAVR ;Turns off All Transistors
	out VPORTC_OUT, r16
	ret

shift_values:
	sts bcd_entries+0,r20
	sts bcd_entries+1,r16
	sts bcd_entries+2,r17 ;shifting values up
	sts bcd_entries+3,r18 ;shifitng values up

	mov r21,r18 ;Digit 4
	mov r18,r17 ;Temp for digit 3
	mov r17,r20 ;Digit 1
	mov r19,r16 ;Digit 2
	mov r20,r18 ;Digit 3

	ret 


hex_to_7seg:
	ldi ZH, HIGH(hextable * 2) ;set Z to point to start of table
	ldi ZL, LOW(hextable * 2)
	ldi r16, $00			   ;add offset to Z pointer
	add ZL, r18
	add ZH, r16
	lpm r18, Z				   ;load byte from table pointed to by Z
	ret
	;Table of segment vlaues to display digits 0 - F
	;!!! seven vlaues must be added
hextable: 
	.db $01,$4F,$12,$06,$4C,$24,$60,$0F,$00, $0C, $08,$60,$31,$42,$30,$38

toggle_pin_ISR:
	push r16
	in r16, CPU_SREG
	push r16
	push r17
	
	ldi r16, TCA_SINGLE_OVF_bm ;clear OVF flag
	sts TCA0_SINGLE_INTFLAGS, r16

	pop r17
	pop r16
	out CPU_SREG,r16
	pop r16	

	rcall turn_off_display	
	rcall multiplex_display
	
	reti


;***************************************************************************
;*
;* "BCD2bin16" - BCD to 16-Bit Binary Conversion
;*
;* This subroutine converts a 5-digit packed BCD number represented by
;* 3 bytes (fBCD2:fBCD1:fBCD0) to a 16-bit number (tbinH:tbinL).
;* MSD of the 5-digit number must be placed in the lowermost nibble of fBCD2.
;*
;* Let "abcde" denote the 5-digit number. The conversion is done by
;* computing the formula: 10(10(10(10a+b)+c)+d)+e.
;* The subroutine "mul10a"/"mul10b" does the multiply-and-add operation
;* which is repeated four times during the computation.
;*
;* Number of words	:30
;* Number of cycles	:108
;* Low registers used	:4 (copyL,copyH,mp10L/tbinL,mp10H/tbinH)
;* High registers used  :4 (fBCD0,fBCD1,fBCD2,adder)	
;*
;***************************************************************************

;***** "mul10a"/"mul10b" Subroutine Register Variables

.def	copyL	=r12		;temporary register
.def	copyH	=r13		;temporary register
.def	mp10L	=r14		;Low byte of number to be multiplied by 10
.def	mp10H	=r15		;High byte of number to be multiplied by 10
.def	adder	=r19		;value to add after multiplication	

;***** Code

mul10a:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" high nibble
	swap	adder
mul10b:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" low nibble
	mov	copyL,mp10L	;make copy
	mov	copyH,mp10H
	lsl	mp10L		;multiply original by 2
	rol	mp10H
	lsl	copyL		;multiply copy by 2
	rol	copyH		
	lsl	copyL		;multiply copy by 2 (4)
	rol	copyH		
	lsl	copyL		;multiply copy by 2 (8)
	rol	copyH		
	add	mp10L,copyL	;add copy to original
	adc	mp10H,copyH	
	andi	adder,0x0f	;mask away upper nibble of adder
	add	mp10L,adder	;add lower nibble of adder
	brcc	m10_1		;if carry not cleared
	inc	mp10H		;	inc high byte
m10_1:	ret	

;***** Main Routine Register Variables

.def	tbinL	=r14		;Low byte of binary result (same as mp10L)
.def	tbinH	=r15		;High byte of binary result (same as mp10H)
.def	fBCD0	=r16		;BCD value digits 1 and 0
.def	fBCD1	=r17		;BCD value digits 2 and 3
.def	fBCD2	=r18		;BCD value digit 5

;***** Code

BCD2bin16:
	andi	fBCD2,0x0f	;mask away upper nibble of fBCD2
	clr	mp10H		
	mov	mp10L,fBCD2	;mp10H:mp10L = a
	mov	adder,fBCD1
	rcall	mul10a		;mp10H:mp10L = 10a+b
	mov	adder,fBCD1
	rcall	mul10b		;mp10H:mp10L = 10(10a+b)+c
	mov	adder,fBCD0		
	rcall	mul10a		;mp10H:mp10L = 10(10(10a+b)+c)+d
	mov	adder,fBCD0
	rcall	mul10b		;mp10H:mp10L = 10(10(10(10a+b)+c)+d)+e
	ret

one_sec_delay:
	ldi r16, 0xFF ;loads full set of 1 to be used in Var delay
	rcall var_delay ;runs var_delay
	inc r18; increments count 

	cpi r18, 0xC3 ;0x28 ;Cycle was found to run .0255s for 84924cycles 1/0.255 is 39times for one second org;0x27
	brne one_sec_delay
	ldi r18,0x00 ;resests count for next run
	ret

var_delay: ;delay ATmega4809 @ 3.33 MHz, approx.= r16 * 0.1 ms
outer_loop:
 ldi r17, 110
inner_loop:
 dec r17
 brne inner_loop
 dec r16
 brne outer_loop
ret 

; ***************************************************************************
;* 
;* "bin16_to_BCD" - 16-bit Binary to BCD Conversion
;*
;* Description: Converts a 16-bit unsigned binary number to a five digit
;* packed BCD number. Uses subroutine div16u from Atmel application note AVR200
;*
;* Author:					Ken Short
;* Version:					0.0
;* Last updated:			111320
;* Target:					ATmega4809
;* Number of words:
;* Number of cycles:
;* Low registers modified:	r14, r15
;* High registers modified: r16, r17, r18, r19, r20, r22, r23, r24
;*
;* Parameters: r17:r16 16-bit unsigned right justified number to be converted.
;* Returns:		r24:r23:r22 five digit packed BCD result.
;*
;* Notes: 
;* Subroutine uses repeated division by 10 to perform conversion.
;***************************************************************************
bin16_to_BCD:
	ldi r19, 0			;high byte of divisor for div16u
	ldi r18, 10			;low byte of the divisor for div16u

	rcall div16u		;divide original binary number by 10
	mov r22, r14		;result is BCD digit 0 (least significant digit)
	rcall div16u		;divide result from first division by 10, gives digit 1 
	swap r14			;swap digit 1 for packing
	or r22, r14			;pack

	rcall div16u		;divide result from second division by 10, gives digit 2
	mov r23, r14		;place in r23
	rcall div16u		;divide result from third division by 10, gives digit 3 
	swap r14			;swap digit 3 for packing
	or r23, r14			;pack

	rcall div16u		;divide result from fourth division by 10, gives digit 4
	mov r24, r14		;place in r24

	ret


;Subroutine div16u is from Atmel application note AVR200

;***************************************************************************
;*
;* "div16u" - 16/16 Bit Unsigned Division
;*
;* This subroutine divides the two 16-bit numbers 
;*# "dd16uH:dd16uL" (dividend) and "dv16uH:dv16uL" (divisor). 
;* The result is placed in "dres16uH:dres16uL" and the remainder in
;* "drem16uH:drem16uL".
;*  
;* Number of words	:19
;* Number of cycles	:235/251 (Min/Max)
;* Low registers used	:2 (drem16uL,drem16uH)
;* High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem16uL=r14
.def	drem16uH=r15
.def	dres16uL=r16
.def	dres16uH=r17
.def	dd16uL	=r16
.def	dd16uH	=r17
.def	dv16uL	=r18
.def	dv16uH	=r19
.def	dcnt16u	=r20

;***** Code

div16u:	clr	drem16uL	;clear remainder Low byte
	sub	drem16uH,drem16uH;clear remainder High byte and carry
	ldi	dcnt16u,17	;init loop counter
d16u_1:	rol	dd16uL		;shift left dividend
	rol	dd16uH
	dec	dcnt16u		;decrement counter
	brne	d16u_2		;if done
	ret			;    return
d16u_2:	rol	drem16uL	;shift dividend into remainder
	rol	drem16uH
	sub	drem16uL,dv16uL	;remainder = remainder - divisor
	sbc	drem16uH,dv16uH	;
	brcc	d16u_3		;if result negative
	add	drem16uL,dv16uL	;    restore remainder
	adc	drem16uH,dv16uH
	clc			;    clear carry to be shifted into result
	rjmp	d16u_1		;else
d16u_3:	sec			;    set carry to be shifted into result
	rjmp	d16u_1


;***************************************************************************
;*
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	mc16uL	=r16		;multiplicand low byte
.def	mc16uH	=r17		;multiplicand high byte
.def	mp16uL	=r18		;multiplier low byte
.def	mp16uH	=r19		;multiplier high byte
.def	m16u0	=r18		;result byte 0 (LSB)
.def	m16u1	=r19		;result byte 1
.def	m16u2	=r20		;result byte 2
.def	m16u3	=r21		;result byte 3 (MSB)
.def	mcnt16u	=r22		;loop counter

;***** Code

mpy16u:	clr	m16u3		;clear 2 highest bytes of result
	clr	m16u2
	ldi	mcnt16u,16	;init loop counter
	lsr	mp16uH
	ror	mp16uL

m16u_1:	brcc	noad8		;if bit 0 of multiplier set
	add	m16u2,mc16uL	;add multiplicand Low to byte 2 of res
	adc	m16u3,mc16uH	;add multiplicand high to byte 3 of res
noad8:	ror	m16u3		;shift right result byte 3
	ror	m16u2		;rotate right result byte 2
	ror	m16u1		;rotate result byte 1 and multiplier High
	ror	m16u0		;rotate result byte 0 and multiplier Low
	dec	mcnt16u		;decrement loop counter
	brne	m16u_1		;if not done, loop more
	ret