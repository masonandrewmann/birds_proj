EESchema Schematic File Version 4
LIBS:lil_duder-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Microchip_ATmega:ATmega328-PU U1
U 1 1 61854B2B
P 2000 3150
F 0 "U1" H 2100 3150 50  0000 R CNN
F 1 "ATmega328-PU" H 2150 3050 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm_Socket" H 2000 3150 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 2000 3150 50  0001 C CNN
	1    2000 3150
	1    0    0    -1  
$EndComp
$Comp
L modular_synth:NJM2073S U4
U 1 1 61856AB0
P 7550 2250
F 0 "U4" H 7600 2617 50  0000 C CNN
F 1 "NJM2073S" H 7600 2526 50  0000 C CNN
F 2 "Package_SIP:SIP-9_21.54x3mm_P2.54mm" H 7650 2150 50  0001 C CNN
F 3 "" H 7550 2150 50  0001 C CNN
	1    7550 2250
	1    0    0    -1  
$EndComp
$Comp
L modular_synth:NJM2073S U4
U 2 1 618570E2
P 7550 3750
F 0 "U4" H 7650 4050 50  0000 C CNN
F 1 "NJM2073S" H 7700 3950 50  0000 C CNN
F 2 "Package_SIP:SIP-9_21.54x3mm_P2.54mm" H 7650 3650 50  0001 C CNN
F 3 "" H 7550 3650 50  0001 C CNN
	2    7550 3750
	1    0    0    -1  
$EndComp
$Comp
L modular_synth:NJM2073S U4
U 3 1 61857256
P 9750 5900
F 0 "U4" H 9928 5946 50  0000 L CNN
F 1 "NJM2073S" H 9928 5855 50  0000 L CNN
F 2 "Package_SIP:SIP-9_21.54x3mm_P2.54mm" H 9850 5800 50  0001 C CNN
F 3 "" H 9750 5800 50  0001 C CNN
	3    9750 5900
	1    0    0    -1  
$EndComp
$Comp
L Analog_DAC:MCP4901 U3
U 1 1 61857CF3
P 5050 2150
F 0 "U3" H 5650 2000 50  0000 L CNN
F 1 "MCP4901" H 5500 1900 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm_Socket" H 6050 2050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/22248a.pdf" H 6050 2050 50  0001 C CNN
	1    5050 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 2250 3000 2450
Wire Wire Line
	3000 2450 2600 2450
Wire Wire Line
	2600 2250 2900 2250
Wire Wire Line
	2900 2250 2900 2350
$Comp
L power:Earth #PWR09
U 1 1 6185E5ED
P 4650 2050
F 0 "#PWR09" H 4650 1800 50  0001 C CNN
F 1 "Earth" H 4650 1900 50  0001 C CNN
F 2 "" H 4650 2050 50  0001 C CNN
F 3 "~" H 4650 2050 50  0001 C CNN
	1    4650 2050
	0    1    1    0   
$EndComp
$Comp
L power:Earth #PWR011
U 1 1 6185ED0E
P 5050 2550
F 0 "#PWR011" H 5050 2300 50  0001 C CNN
F 1 "Earth" H 5050 2400 50  0001 C CNN
F 2 "" H 5050 2550 50  0001 C CNN
F 3 "~" H 5050 2550 50  0001 C CNN
	1    5050 2550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR010
U 1 1 6185F53E
P 5050 1750
F 0 "#PWR010" H 5050 1600 50  0001 C CNN
F 1 "+5V" H 5065 1923 50  0000 C CNN
F 2 "" H 5050 1750 50  0001 C CNN
F 3 "" H 5050 1750 50  0001 C CNN
	1    5050 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR012
U 1 1 6185FD2E
P 5250 1750
F 0 "#PWR012" H 5250 1600 50  0001 C CNN
F 1 "+5V" H 5265 1923 50  0000 C CNN
F 2 "" H 5250 1750 50  0001 C CNN
F 3 "" H 5250 1750 50  0001 C CNN
	1    5250 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR03
U 1 1 61860116
P 2000 1600
F 0 "#PWR03" H 2000 1450 50  0001 C CNN
F 1 "+5V" H 2015 1773 50  0000 C CNN
F 2 "" H 2000 1600 50  0001 C CNN
F 3 "" H 2000 1600 50  0001 C CNN
	1    2000 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 1600 2100 1600
Wire Wire Line
	2100 1600 2100 1650
Wire Wire Line
	2000 1600 2000 1650
Connection ~ 2000 1600
$Comp
L power:Earth #PWR04
U 1 1 618610CB
P 2000 4700
F 0 "#PWR04" H 2000 4450 50  0001 C CNN
F 1 "Earth" H 2000 4550 50  0001 C CNN
F 2 "" H 2000 4700 50  0001 C CNN
F 3 "~" H 2000 4700 50  0001 C CNN
	1    2000 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 1600 1350 1600
Wire Wire Line
	1350 1600 1350 1950
Wire Wire Line
	1350 1950 1400 1950
$Comp
L Device:CP1 C6
U 1 1 6181A563
P 7250 2650
F 0 "C6" H 7135 2604 50  0000 R CNN
F 1 "1u" H 7135 2695 50  0000 R CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.0mm_Bigger_Holes" H 7250 2650 50  0001 C CNN
F 3 "~" H 7250 2650 50  0001 C CNN
	1    7250 2650
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R6
U 1 1 6181B4FB
P 7050 2850
F 0 "R6" V 6845 2850 50  0000 C CNN
F 1 "220" V 6936 2850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7090 2840 50  0001 C CNN
F 3 "~" H 7050 2850 50  0001 C CNN
	1    7050 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	7300 3650 7250 3650
Wire Wire Line
	7250 3650 7250 3500
Wire Wire Line
	7250 3200 7250 3150
Wire Wire Line
	6850 3150 6850 2850
Wire Wire Line
	6850 2850 6900 2850
Wire Wire Line
	7200 2850 7250 2850
Wire Wire Line
	7250 2850 7250 2800
Wire Wire Line
	7250 2500 7250 2350
Wire Wire Line
	7250 2350 7300 2350
$Comp
L Device:R_US R5
U 1 1 61821FEC
P 6250 2500
F 0 "R5" H 6182 2454 50  0000 R CNN
F 1 "10k" H 6182 2545 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6290 2490 50  0001 C CNN
F 3 "~" H 6250 2500 50  0001 C CNN
	1    6250 2500
	-1   0    0    1   
$EndComp
$Comp
L power:Earth #PWR013
U 1 1 61823B2F
P 6450 2700
F 0 "#PWR013" H 6450 2450 50  0001 C CNN
F 1 "Earth" H 6450 2550 50  0001 C CNN
F 2 "" H 6450 2700 50  0001 C CNN
F 3 "~" H 6450 2700 50  0001 C CNN
	1    6450 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R8
U 1 1 61824359
P 7650 2850
F 0 "R8" V 7445 2850 50  0000 C CNN
F 1 "5k" V 7536 2850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7690 2840 50  0001 C CNN
F 3 "~" H 7650 2850 50  0001 C CNN
	1    7650 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	7500 2850 7250 2850
Connection ~ 7250 2850
Wire Wire Line
	7800 2850 7950 2850
Wire Wire Line
	7950 2850 7950 2250
Wire Wire Line
	7950 2250 7900 2250
$Comp
L Device:R_US R9
U 1 1 61825451
P 7650 3150
F 0 "R9" V 7445 3150 50  0000 C CNN
F 1 "5k" V 7536 3150 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7690 3140 50  0001 C CNN
F 3 "~" H 7650 3150 50  0001 C CNN
	1    7650 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	7500 3150 7250 3150
Connection ~ 7250 3150
Wire Wire Line
	7800 3150 7950 3150
Wire Wire Line
	7950 3150 7950 3750
Wire Wire Line
	7950 3750 7900 3750
$Comp
L Device:C C3
U 1 1 618267D6
P 6700 3800
F 0 "C3" H 6815 3846 50  0000 L CNN
F 1 "22n" H 6815 3755 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 6738 3650 50  0001 C CNN
F 3 "~" H 6700 3800 50  0001 C CNN
	1    6700 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3650 6700 3650
Connection ~ 7250 3650
$Comp
L power:Earth #PWR014
U 1 1 61827748
P 6700 3950
F 0 "#PWR014" H 6700 3700 50  0001 C CNN
F 1 "Earth" H 6700 3800 50  0001 C CNN
F 2 "" H 6700 3950 50  0001 C CNN
F 3 "~" H 6700 3950 50  0001 C CNN
	1    6700 3950
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR015
U 1 1 61827C08
P 7250 3950
F 0 "#PWR015" H 7250 3700 50  0001 C CNN
F 1 "Earth" H 7250 3800 50  0001 C CNN
F 2 "" H 7250 3950 50  0001 C CNN
F 3 "~" H 7250 3950 50  0001 C CNN
	1    7250 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 3850 7250 3850
Wire Wire Line
	7250 3850 7250 3950
$Comp
L Device:R_US R12
U 1 1 61828E64
P 8550 2800
F 0 "R12" H 8482 2754 50  0000 R CNN
F 1 "16" H 8482 2845 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8590 2790 50  0001 C CNN
F 3 "~" H 8550 2800 50  0001 C CNN
	1    8550 2800
	-1   0    0    1   
$EndComp
$Comp
L Device:C C8
U 1 1 61829301
P 8550 3100
F 0 "C8" H 8665 3146 50  0000 L CNN
F 1 "0.22u" H 8665 3055 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 8588 2950 50  0001 C CNN
F 3 "~" H 8550 3100 50  0001 C CNN
	1    8550 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 3250 8550 3750
Wire Wire Line
	8550 3750 7950 3750
Connection ~ 7950 3750
Wire Wire Line
	8550 2650 8550 2250
Wire Wire Line
	8550 2250 7950 2250
Connection ~ 7950 2250
$Comp
L Device:C C2
U 1 1 6182C8D5
P 6650 2500
F 0 "C2" H 6765 2546 50  0000 L CNN
F 1 "22n" H 6765 2455 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 6688 2350 50  0001 C CNN
F 3 "~" H 6650 2500 50  0001 C CNN
	1    6650 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 2350 6650 2350
Connection ~ 7250 2350
Wire Wire Line
	6450 2700 6450 2650
Wire Wire Line
	6450 2650 6250 2650
Wire Wire Line
	6450 2650 6650 2650
Connection ~ 6450 2650
Wire Wire Line
	6250 2350 6250 2150
Wire Wire Line
	6250 2150 7300 2150
$Comp
L power:Earth #PWR017
U 1 1 61832C3D
P 9750 6250
F 0 "#PWR017" H 9750 6000 50  0001 C CNN
F 1 "Earth" H 9750 6100 50  0001 C CNN
F 2 "" H 9750 6250 50  0001 C CNN
F 3 "~" H 9750 6250 50  0001 C CNN
	1    9750 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 6150 9650 6150
Wire Wire Line
	9850 6150 9750 6150
Connection ~ 9750 6150
$Comp
L Device:C C5
U 1 1 618353F6
P 9350 5900
F 0 "C5" H 9465 5946 50  0000 L CNN
F 1 "0.1u" H 9465 5855 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 9388 5750 50  0001 C CNN
F 3 "~" H 9350 5900 50  0001 C CNN
	1    9350 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 6150 9750 6200
Wire Wire Line
	9750 6200 9350 6200
Connection ~ 9750 6200
Wire Wire Line
	9750 6200 9750 6250
Wire Wire Line
	9350 6050 9350 6200
Wire Wire Line
	9750 5650 9750 5600
Wire Wire Line
	9350 5750 9350 5600
Wire Wire Line
	9350 5600 9750 5600
$Comp
L Device:CP1 C7
U 1 1 6183DA3A
P 7250 3350
F 0 "C7" H 7365 3396 50  0000 L CNN
F 1 "1u" H 7365 3305 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.0mm_Bigger_Holes" H 7250 3350 50  0001 C CNN
F 3 "~" H 7250 3350 50  0001 C CNN
	1    7250 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J5
U 1 1 6183E54B
P 8750 2250
F 0 "J5" H 8778 2276 50  0000 L CNN
F 1 "SPK_A" H 8778 2185 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 8750 2250 50  0001 C CNN
F 3 "~" H 8750 2250 50  0001 C CNN
	1    8750 2250
	1    0    0    -1  
$EndComp
Connection ~ 8550 2250
$Comp
L Connector:Conn_01x01_Female J6
U 1 1 6183EEE7
P 8750 3750
F 0 "J6" H 8778 3776 50  0000 L CNN
F 1 "SPK_B" H 8778 3685 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 8750 3750 50  0001 C CNN
F 3 "~" H 8750 3750 50  0001 C CNN
	1    8750 3750
	1    0    0    -1  
$EndComp
Connection ~ 8550 3750
Wire Wire Line
	5750 2150 6250 2150
Connection ~ 6250 2150
Text Notes 8800 2900 0    50   ~ 0
Choose equal to\nresistive load\nof speaker\n
$Comp
L Device:Microphone_Condenser MK1
U 1 1 61819B5C
P 2550 6250
F 0 "MK1" H 2420 6296 50  0000 R CNN
F 1 "Mic" H 2420 6205 50  0000 R CNN
F 2 "" V 2550 6350 50  0001 C CNN
F 3 "~" V 2550 6350 50  0001 C CNN
	1    2550 6250
	-1   0    0    -1  
$EndComp
$Comp
L Device:CP1 C1
U 1 1 6181CB0B
P 2800 5850
F 0 "C1" V 2550 5800 50  0000 L CNN
F 1 "1u" V 2650 5800 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.0mm_Bigger_Holes" H 2800 5850 50  0001 C CNN
F 3 "~" H 2800 5850 50  0001 C CNN
	1    2800 5850
	0    -1   1    0   
$EndComp
Wire Wire Line
	2550 5850 2650 5850
$Comp
L power:+5V #PWR01
U 1 1 6181FF1E
P 2200 5850
F 0 "#PWR01" H 2200 5700 50  0001 C CNN
F 1 "+5V" H 2215 6023 50  0000 C CNN
F 2 "" H 2200 5850 50  0001 C CNN
F 3 "" H 2200 5850 50  0001 C CNN
	1    2200 5850
	-1   0    0    -1  
$EndComp
Connection ~ 2550 5850
$Comp
L power:Earth #PWR02
U 1 1 618205F3
P 2550 6600
F 0 "#PWR02" H 2550 6350 50  0001 C CNN
F 1 "Earth" H 2550 6450 50  0001 C CNN
F 2 "" H 2550 6600 50  0001 C CNN
F 3 "~" H 2550 6600 50  0001 C CNN
	1    2550 6600
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_US R1
U 1 1 618216FB
P 2400 5850
F 0 "R1" V 2200 5850 50  0000 R CNN
F 1 "2k2" V 2300 5900 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2440 5840 50  0001 C CNN
F 3 "~" H 2400 5850 50  0001 C CNN
	1    2400 5850
	0    -1   1    0   
$EndComp
Wire Wire Line
	2200 5850 2250 5850
$Comp
L Amplifier_Operational:TL072 U2
U 1 1 6182858C
P 3750 5950
F 0 "U2" H 3800 5700 50  0000 C CNN
F 1 "TL072" H 3850 5800 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_Socket" H 3750 5950 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 3750 5950 50  0001 C CNN
	1    3750 5950
	1    0    0    1   
$EndComp
$Comp
L Amplifier_Operational:TL072 U2
U 2 1 61829A70
P 4700 6050
F 0 "U2" H 4700 6300 50  0000 C CNN
F 1 "TL072" H 4800 6200 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_Socket" H 4700 6050 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 4700 6050 50  0001 C CNN
	2    4700 6050
	1    0    0    1   
$EndComp
$Comp
L Amplifier_Operational:TL072 U2
U 3 1 6182ABC5
P 3750 5950
F 0 "U2" H 3750 5800 50  0000 L CNN
F 1 "TL072" H 3700 5700 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm_Socket" H 3750 5950 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 3750 5950 50  0001 C CNN
	3    3750 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 6184BCCF
P 3200 5850
F 0 "R2" V 2995 5850 50  0000 C CNN
F 1 "1k" V 3086 5850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3240 5840 50  0001 C CNN
F 3 "~" H 3200 5850 50  0001 C CNN
	1    3200 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	3050 5850 2950 5850
Wire Wire Line
	3350 5850 3400 5850
Wire Wire Line
	3450 6050 3400 6050
$Comp
L Device:R_US R4
U 1 1 6185784E
P 3750 5400
F 0 "R4" V 3545 5400 50  0000 C CNN
F 1 "100k" V 3636 5400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3790 5390 50  0001 C CNN
F 3 "~" H 3750 5400 50  0001 C CNN
	1    3750 5400
	0    1    1    0   
$EndComp
Connection ~ 3400 5850
Wire Wire Line
	3400 5850 3450 5850
Wire Wire Line
	4100 5950 4050 5950
Wire Wire Line
	3400 5400 3600 5400
Wire Wire Line
	3400 5400 3400 5850
Wire Wire Line
	3900 5400 4100 5400
Wire Wire Line
	4100 5400 4100 5950
$Comp
L power:Earth #PWR07
U 1 1 618756DA
P 3650 6250
F 0 "#PWR07" H 3650 6000 50  0001 C CNN
F 1 "Earth" H 3650 6100 50  0001 C CNN
F 2 "" H 3650 6250 50  0001 C CNN
F 3 "~" H 3650 6250 50  0001 C CNN
	1    3650 6250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2600 2850 2900 2850
Wire Wire Line
	2550 5850 2550 6000
Wire Wire Line
	2550 6450 2550 6550
$Comp
L Connector:Conn_01x01_Female J2
U 1 1 618834E2
P 2350 6550
F 0 "J2" H 2400 6500 50  0000 C CNN
F 1 "Mic-" H 2400 6600 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 2350 6550 50  0001 C CNN
F 3 "~" H 2350 6550 50  0001 C CNN
	1    2350 6550
	-1   0    0    1   
$EndComp
Connection ~ 2550 6550
Wire Wire Line
	2550 6550 2550 6600
$Comp
L Connector:Conn_01x01_Female J1
U 1 1 61884B90
P 2350 6000
F 0 "J1" H 2400 5950 50  0000 C CNN
F 1 "Mic+" H 2400 6050 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 2350 6000 50  0001 C CNN
F 3 "~" H 2350 6000 50  0001 C CNN
	1    2350 6000
	-1   0    0    1   
$EndComp
Connection ~ 2550 6000
Wire Wire Line
	2550 6000 2550 6050
$Comp
L power:+2V5 #PWR05
U 1 1 61885E9F
P 3400 6750
F 0 "#PWR05" H 3400 6600 50  0001 C CNN
F 1 "+2V5" H 3415 6923 50  0000 C CNN
F 2 "" H 3400 6750 50  0001 C CNN
F 3 "" H 3400 6750 50  0001 C CNN
	1    3400 6750
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R3
U 1 1 61887185
P 3400 6600
F 0 "R3" V 3195 6600 50  0000 C CNN
F 1 "5k" V 3286 6600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3440 6590 50  0001 C CNN
F 3 "~" H 3400 6600 50  0001 C CNN
	1    3400 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R10
U 1 1 6188DB24
P 10650 5450
F 0 "R10" H 10500 5400 50  0000 C CNN
F 1 "100k" H 10500 5550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 10690 5440 50  0001 C CNN
F 3 "~" H 10650 5450 50  0001 C CNN
	1    10650 5450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R11
U 1 1 6188EB11
P 10450 6200
F 0 "R11" V 10245 6200 50  0000 C CNN
F 1 "100k" V 10336 6200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 10490 6190 50  0001 C CNN
F 3 "~" H 10450 6200 50  0001 C CNN
	1    10450 6200
	0    1    1    0   
$EndComp
Wire Wire Line
	10600 6200 10650 6200
Wire Wire Line
	10650 6200 10650 5900
Wire Wire Line
	10300 6200 9750 6200
$Comp
L power:+2V5 #PWR018
U 1 1 6189A661
P 10750 5900
F 0 "#PWR018" H 10750 5750 50  0001 C CNN
F 1 "+2V5" H 10765 6073 50  0000 C CNN
F 2 "" H 10750 5900 50  0001 C CNN
F 3 "" H 10750 5900 50  0001 C CNN
	1    10750 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10750 5900 10650 5900
Connection ~ 10650 5900
Wire Wire Line
	10650 5900 10650 5600
Wire Wire Line
	9300 5000 9300 4950
Wire Wire Line
	9300 5350 9300 5300
$Comp
L power:+5V #PWR0101
U 1 1 618F125D
P 9300 4950
F 0 "#PWR0101" H 9300 4800 50  0001 C CNN
F 1 "+5V" H 9315 5123 50  0000 C CNN
F 2 "" H 9300 4950 50  0001 C CNN
F 3 "" H 9300 4950 50  0001 C CNN
	1    9300 4950
	1    0    0    -1  
$EndComp
Connection ~ 9300 4950
Connection ~ 8600 4950
Wire Wire Line
	7950 4950 8300 4950
Wire Wire Line
	8950 5350 8950 5450
$Comp
L power:Earth #PWR0102
U 1 1 618E584B
P 8950 5450
F 0 "#PWR0102" H 8950 5200 50  0001 C CNN
F 1 "Earth" H 8950 5300 50  0001 C CNN
F 2 "" H 8950 5450 50  0001 C CNN
F 3 "~" H 8950 5450 50  0001 C CNN
	1    8950 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 5350 9300 5350
Connection ~ 8950 5350
Wire Wire Line
	8950 5250 8950 5350
Wire Wire Line
	8600 5350 8950 5350
Wire Wire Line
	8600 5300 8600 5350
Wire Wire Line
	9300 4950 9250 4950
Wire Wire Line
	8600 4950 8650 4950
Wire Wire Line
	8600 5000 8600 4950
$Comp
L Regulator_Linear:L7805 U5
U 1 1 618C37A9
P 8950 4950
F 0 "U5" H 8950 5192 50  0000 C CNN
F 1 "L7805" H 8950 5101 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 8975 4800 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 8950 4900 50  0001 C CNN
	1    8950 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C10
U 1 1 618D1955
P 9300 5150
F 0 "C10" H 9400 5200 50  0000 L CNN
F 1 "100u" H 9400 5100 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.0mm_Bigger_Holes" H 9300 5150 50  0001 C CNN
F 3 "~" H 9300 5150 50  0001 C CNN
	1    9300 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C9
U 1 1 618D103D
P 8600 5150
F 0 "C9" H 8400 5200 50  0000 L CNN
F 1 "100u" H 8300 5100 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.0mm_Bigger_Holes" H 8600 5150 50  0001 C CNN
F 3 "~" H 8600 5150 50  0001 C CNN
	1    8600 5150
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR0103
U 1 1 618CBD34
P 7950 6100
F 0 "#PWR0103" H 7950 5850 50  0001 C CNN
F 1 "Earth" H 7950 5950 50  0001 C CNN
F 2 "" H 7950 6100 50  0001 C CNN
F 3 "~" H 7950 6100 50  0001 C CNN
	1    7950 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 5400 7950 5350
$Comp
L Device:Battery BT1
U 1 1 618C19BB
P 7950 5600
F 0 "BT1" H 8058 5646 50  0000 L CNN
F 1 "Battery" H 8058 5555 50  0000 L CNN
F 2 "" V 7950 5660 50  0001 C CNN
F 3 "~" V 7950 5660 50  0001 C CNN
	1    7950 5600
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW1
U 1 1 618A220C
P 7950 5150
F 0 "SW1" V 7904 5248 50  0000 L CNN
F 1 "9v_SW" V 7995 5248 50  0000 L CNN
F 2 "" H 7950 5150 50  0001 C CNN
F 3 "~" H 7950 5150 50  0001 C CNN
	1    7950 5150
	0    1    1    0   
$EndComp
Wire Notes Line
	10950 4650 10950 6400
Text Notes 10600 4800 0    50   ~ 0
POWER
Wire Notes Line
	6100 1800 9600 1800
Wire Notes Line
	9600 1800 9600 4250
Wire Notes Line
	9600 4250 6100 4250
Wire Notes Line
	6100 4250 6100 1800
Text Notes 9150 1950 0    50   ~ 0
AMPLIFIER
Wire Notes Line
	1950 5100 5300 5100
Text Notes 4750 5250 0    50   ~ 0
MICROPHONE
$Comp
L power:PWR_FLAG #FLG01
U 1 1 6181D1B4
P 7950 5800
F 0 "#FLG01" H 7950 5875 50  0001 C CNN
F 1 "PWR_FLAG" V 7950 5928 50  0000 L CNN
F 2 "" H 7950 5800 50  0001 C CNN
F 3 "~" H 7950 5800 50  0001 C CNN
	1    7950 5800
	0    1    1    0   
$EndComp
Connection ~ 7950 5800
$Comp
L power:PWR_FLAG #FLG02
U 1 1 61821C28
P 8300 4950
F 0 "#FLG02" H 8300 5025 50  0001 C CNN
F 1 "PWR_FLAG" H 8300 5123 50  0000 C CNN
F 2 "" H 8300 4950 50  0001 C CNN
F 3 "~" H 8300 4950 50  0001 C CNN
	1    8300 4950
	1    0    0    -1  
$EndComp
Connection ~ 8300 4950
Wire Wire Line
	8300 4950 8600 4950
NoConn ~ 2600 2050
NoConn ~ 2600 2350
NoConn ~ 2600 3050
NoConn ~ 2600 3150
NoConn ~ 2600 3250
NoConn ~ 2600 3350
NoConn ~ 2600 4050
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 61862DC8
P 10650 6200
F 0 "#FLG0101" H 10650 6275 50  0001 C CNN
F 1 "PWR_FLAG" V 10750 6100 50  0000 L CNN
F 2 "" H 10650 6200 50  0001 C CNN
F 3 "~" H 10650 6200 50  0001 C CNN
	1    10650 6200
	0    1    1    0   
$EndComp
Connection ~ 10650 6200
$Comp
L Device:LED D4
U 1 1 61863B53
P 7650 5200
F 0 "D4" V 7700 5350 50  0000 R CNN
F 1 "PWLED" V 7600 5500 50  0000 R CNN
F 2 "" H 7650 5200 50  0001 C CNN
F 3 "~" H 7650 5200 50  0001 C CNN
	1    7650 5200
	0    -1   -1   0   
$EndComp
$Comp
L power:Earth #PWR022
U 1 1 61865E69
P 7650 5850
F 0 "#PWR022" H 7650 5600 50  0001 C CNN
F 1 "Earth" H 7650 5700 50  0001 C CNN
F 2 "" H 7650 5850 50  0001 C CNN
F 3 "~" H 7650 5850 50  0001 C CNN
	1    7650 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 5050 7650 4950
Wire Wire Line
	7650 4950 7950 4950
Connection ~ 7950 4950
$Comp
L Connector:Conn_01x02_Female J8
U 1 1 6186CDA7
P 7450 4950
F 0 "J8" H 7342 4725 50  0000 C CNN
F 1 "9V_Conn" H 7342 4816 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x02_P3.81mm_Drill1mm" H 7450 4950 50  0001 C CNN
F 3 "~" H 7450 4950 50  0001 C CNN
	1    7450 4950
	-1   0    0    1   
$EndComp
Connection ~ 7650 4950
$Comp
L Connector:Conn_01x01_Female J9
U 1 1 6186DA5D
P 7450 5850
F 0 "J9" H 7342 5625 50  0000 C CNN
F 1 "LED_GND" H 7450 5750 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 7450 5850 50  0001 C CNN
F 3 "~" H 7450 5850 50  0001 C CNN
	1    7450 5850
	-1   0    0    1   
$EndComp
Wire Notes Line
	7300 4650 7300 6400
Wire Notes Line
	7300 4650 10950 4650
Wire Notes Line
	7300 6400 10950 6400
$Comp
L Device:R_US R20
U 1 1 61871F27
P 7650 5500
F 0 "R20" V 7445 5500 50  0000 C CNN
F 1 "5k" V 7536 5500 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7690 5490 50  0001 C CNN
F 3 "~" H 7650 5500 50  0001 C CNN
	1    7650 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 5650 7650 5850
Connection ~ 7650 5850
Text Notes 5300 1800 0    50   ~ 0
DIGITAL/ANALOG \nCONVERTER\n
Wire Notes Line
	5950 1500 5950 2750
Wire Notes Line
	4400 1500 5950 1500
Wire Notes Line
	1150 4850 1150 1350
Text Notes 2200 1500 0    50   ~ 0
MICROCONTROLLER
$Comp
L Device:LED D1
U 1 1 6188BB17
P 4600 4450
F 0 "D1" V 4650 4600 50  0000 R CNN
F 1 "PWLED" V 4550 4750 50  0000 R CNN
F 2 "" H 4600 4450 50  0001 C CNN
F 3 "~" H 4600 4450 50  0001 C CNN
	1    4600 4450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R16
U 1 1 6188BB1D
P 4600 4750
F 0 "R16" V 4395 4750 50  0000 C CNN
F 1 "5k" V 4486 4750 50  0000 C CNN
F 2 "" V 4640 4740 50  0001 C CNN
F 3 "~" H 4600 4750 50  0001 C CNN
	1    4600 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 6188F7B2
P 5000 4450
F 0 "D2" V 5050 4600 50  0000 R CNN
F 1 "PWLED" V 4950 4750 50  0000 R CNN
F 2 "" H 5000 4450 50  0001 C CNN
F 3 "~" H 5000 4450 50  0001 C CNN
	1    5000 4450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R18
U 1 1 6188F7B8
P 5000 4750
F 0 "R18" V 4795 4750 50  0000 C CNN
F 1 "5k" V 4886 4750 50  0000 C CNN
F 2 "" V 5040 4740 50  0001 C CNN
F 3 "~" H 5000 4750 50  0001 C CNN
	1    5000 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 618933DD
P 5350 4450
F 0 "D3" V 5400 4600 50  0000 R CNN
F 1 "PWLED" V 5300 4750 50  0000 R CNN
F 2 "" H 5350 4450 50  0001 C CNN
F 3 "~" H 5350 4450 50  0001 C CNN
	1    5350 4450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R19
U 1 1 618933E3
P 5350 4750
F 0 "R19" V 5145 4750 50  0000 C CNN
F 1 "5k" V 5236 4750 50  0000 C CNN
F 2 "" V 5390 4740 50  0001 C CNN
F 3 "~" H 5350 4750 50  0001 C CNN
	1    5350 4750
	1    0    0    -1  
$EndComp
Wire Notes Line
	4250 4200 4250 5000
Wire Notes Line
	4250 5000 5950 5000
Wire Notes Line
	5950 5000 5950 4200
Wire Notes Line
	4250 4200 5950 4200
Text Notes 5550 4500 0    50   ~ 0
PANEL\nMOUNTED\nLEDS
$Comp
L Connector:Conn_01x01_Female J7
U 1 1 6181DAB0
P 5600 4100
F 0 "J7" H 5550 4250 50  0000 L CNN
F 1 "LED3" H 5550 4150 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 5600 4100 50  0001 C CNN
F 3 "~" H 5600 4100 50  0001 C CNN
	1    5600 4100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J4
U 1 1 6182089F
P 5200 3900
F 0 "J4" H 5150 4050 50  0000 L CNN
F 1 "LED2" H 5150 3950 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 5200 3900 50  0001 C CNN
F 3 "~" H 5200 3900 50  0001 C CNN
	1    5200 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J3
U 1 1 618218AD
P 4800 3700
F 0 "J3" H 4750 3850 50  0000 L CNN
F 1 "LED1" H 4750 3750 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 4800 3700 50  0001 C CNN
F 3 "~" H 4800 3700 50  0001 C CNN
	1    4800 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 4300 5350 4100
Wire Wire Line
	5350 4100 5400 4100
$Comp
L Device:Crystal Y1
U 1 1 618468FD
P 2850 2650
F 0 "Y1" V 2700 2650 50  0000 L CNN
F 1 "16Hz" V 3000 2650 50  0000 L CNN
F 2 "Crystal:Crystal_HC49-4H_Vertical" H 2850 2650 50  0001 C CNN
F 3 "~" H 2850 2650 50  0001 C CNN
	1    2850 2650
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 6184824E
P 3300 3450
F 0 "SW2" H 3300 3735 50  0000 C CNN
F 1 "RST_Butt" H 3300 3644 50  0000 C CNN
F 2 "" H 3300 3650 50  0001 C CNN
F 3 "~" H 3300 3650 50  0001 C CNN
	1    3300 3450
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR021
U 1 1 61848F51
P 3500 3650
F 0 "#PWR021" H 3500 3400 50  0001 C CNN
F 1 "Earth" H 3500 3500 50  0001 C CNN
F 2 "" H 3500 3650 50  0001 C CNN
F 3 "~" H 3500 3650 50  0001 C CNN
	1    3500 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 2500 2700 2500
Wire Wire Line
	2700 2500 2700 2550
Wire Wire Line
	2700 2550 2600 2550
Wire Wire Line
	2600 2650 2700 2650
Wire Wire Line
	2700 2650 2700 2800
Wire Wire Line
	2700 2800 2850 2800
$Comp
L Device:C C11
U 1 1 61857A36
P 3150 2500
F 0 "C11" V 2898 2500 50  0000 C CNN
F 1 "22p" V 2989 2500 50  0000 C CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 3188 2350 50  0001 C CNN
F 3 "~" H 3150 2500 50  0001 C CNN
	1    3150 2500
	0    1    1    0   
$EndComp
$Comp
L Device:C C12
U 1 1 61858818
P 3150 2800
F 0 "C12" V 2898 2800 50  0000 C CNN
F 1 "22p" V 2989 2800 50  0000 C CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 3188 2650 50  0001 C CNN
F 3 "~" H 3150 2800 50  0001 C CNN
	1    3150 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	2850 2500 3000 2500
Connection ~ 2850 2500
Wire Wire Line
	3000 2800 2850 2800
Connection ~ 2850 2800
$Comp
L power:Earth #PWR020
U 1 1 61862B0F
P 3300 2650
F 0 "#PWR020" H 3300 2400 50  0001 C CNN
F 1 "Earth" H 3300 2500 50  0001 C CNN
F 2 "" H 3300 2650 50  0001 C CNN
F 3 "~" H 3300 2650 50  0001 C CNN
	1    3300 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3300 2800 3300 2650
Wire Wire Line
	3300 2500 3300 2650
Connection ~ 3300 2650
Wire Wire Line
	5000 3900 5000 4300
Wire Wire Line
	5000 3850 5000 3900
Connection ~ 5000 3900
Wire Wire Line
	5350 3950 5350 4100
Connection ~ 5350 4100
$Comp
L Device:R_US R13
U 1 1 61895EAE
P 2750 3650
F 0 "R13" V 2650 3550 50  0000 C CNN
F 1 "10k" V 2650 3700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2790 3640 50  0001 C CNN
F 3 "~" H 2750 3650 50  0001 C CNN
	1    2750 3650
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR019
U 1 1 61896CA4
P 2900 3650
F 0 "#PWR019" H 2900 3500 50  0001 C CNN
F 1 "+5V" H 3000 3700 50  0000 C CNN
F 2 "" H 2900 3650 50  0001 C CNN
F 3 "" H 2900 3650 50  0001 C CNN
	1    2900 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R15
U 1 1 618AB20A
P 4250 5950
F 0 "R15" V 4045 5950 50  0000 C CNN
F 1 "10k" V 4136 5950 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4290 5940 50  0001 C CNN
F 3 "~" H 4250 5950 50  0001 C CNN
	1    4250 5950
	0    1    1    0   
$EndComp
Connection ~ 4100 5950
$Comp
L Device:R_US R17
U 1 1 618AB96A
P 4700 5700
F 0 "R17" V 4495 5700 50  0000 C CNN
F 1 "100k" V 4586 5700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4740 5690 50  0001 C CNN
F 3 "~" H 4700 5700 50  0001 C CNN
	1    4700 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	4550 5700 4400 5700
Wire Wire Line
	4400 5700 4400 5950
Connection ~ 4400 5950
Wire Wire Line
	4850 5700 5000 5700
Wire Wire Line
	5000 5700 5000 6050
Wire Wire Line
	4100 5250 5000 5250
Wire Wire Line
	5000 5250 5000 5700
Connection ~ 5000 5700
Wire Wire Line
	4600 3700 4600 3750
Wire Wire Line
	2600 2150 4650 2150
Wire Wire Line
	3000 2250 4650 2250
Wire Wire Line
	2900 2350 4650 2350
Connection ~ 4600 3750
Wire Wire Line
	4600 3750 4600 4300
Wire Wire Line
	2900 2850 2900 2900
Wire Wire Line
	2900 2900 4100 2900
Wire Wire Line
	4100 2900 4100 5250
$Comp
L Device:R_US R14
U 1 1 61914B2C
P 2800 3300
F 0 "R14" V 2700 3200 50  0000 C CNN
F 1 "10k" V 2700 3350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2840 3290 50  0001 C CNN
F 3 "~" H 2800 3300 50  0001 C CNN
	1    2800 3300
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR08
U 1 1 6191508F
P 2800 3150
F 0 "#PWR08" H 2800 3000 50  0001 C CNN
F 1 "+5V" H 2900 3200 50  0000 C CNN
F 2 "" H 2800 3150 50  0001 C CNN
F 3 "" H 2800 3150 50  0001 C CNN
	1    2800 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3450 2800 3450
Wire Wire Line
	2800 3450 3100 3450
Connection ~ 2800 3450
$Comp
L Device:CP1 C13
U 1 1 61921065
P 3300 3650
F 0 "C13" V 3350 3750 50  0000 C CNN
F 1 "1u" V 3350 3550 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.0mm_Bigger_Holes" H 3300 3650 50  0001 C CNN
F 3 "~" H 3300 3650 50  0001 C CNN
	1    3300 3650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3500 3650 3450 3650
Wire Wire Line
	3500 3650 3500 3450
Connection ~ 3500 3650
Wire Wire Line
	3150 3650 3100 3650
Wire Wire Line
	3100 3650 3100 3450
Connection ~ 3100 3450
Wire Wire Line
	2000 4700 1850 4700
Wire Wire Line
	1850 4700 1850 4650
Wire Wire Line
	2000 4700 2000 4650
Connection ~ 2000 4700
Wire Notes Line
	3600 4850 3600 1350
Wire Notes Line
	1150 1350 3600 1350
Wire Notes Line
	1150 4850 3600 4850
Wire Notes Line
	4400 1500 4400 2750
Wire Notes Line
	4400 2750 5950 2750
Wire Wire Line
	3400 6050 3400 6450
Wire Wire Line
	3400 6450 4400 6450
Wire Wire Line
	4400 6450 4400 6150
Connection ~ 3400 6450
Wire Notes Line
	5300 7050 1950 7050
Wire Notes Line
	5300 5100 5300 7050
Wire Notes Line
	1950 5100 1950 7050
$Comp
L power:Earth #PWR025
U 1 1 6197DD6E
P 5350 4900
F 0 "#PWR025" H 5350 4650 50  0001 C CNN
F 1 "Earth" H 5350 4750 50  0001 C CNN
F 2 "" H 5350 4900 50  0001 C CNN
F 3 "~" H 5350 4900 50  0001 C CNN
	1    5350 4900
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR024
U 1 1 6197E514
P 5000 4900
F 0 "#PWR024" H 5000 4650 50  0001 C CNN
F 1 "Earth" H 5000 4750 50  0001 C CNN
F 2 "" H 5000 4900 50  0001 C CNN
F 3 "~" H 5000 4900 50  0001 C CNN
	1    5000 4900
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR023
U 1 1 6197E8AF
P 4600 4900
F 0 "#PWR023" H 4600 4650 50  0001 C CNN
F 1 "Earth" H 4600 4750 50  0001 C CNN
F 2 "" H 4600 4900 50  0001 C CNN
F 3 "~" H 4600 4900 50  0001 C CNN
	1    4600 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C14
U 1 1 619946E5
P 2400 1600
F 0 "C14" V 2450 1700 50  0000 C CNN
F 1 "10u" V 2450 1500 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.0mm_Bigger_Holes" H 2400 1600 50  0001 C CNN
F 3 "~" H 2400 1600 50  0001 C CNN
	1    2400 1600
	0    -1   -1   0   
$EndComp
$Comp
L power:Earth #PWR026
U 1 1 619955CA
P 2550 1600
F 0 "#PWR026" H 2550 1350 50  0001 C CNN
F 1 "Earth" H 2550 1450 50  0001 C CNN
F 2 "" H 2550 1600 50  0001 C CNN
F 3 "~" H 2550 1600 50  0001 C CNN
	1    2550 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 1600 2100 1600
Connection ~ 2100 1600
$Comp
L power:+5V #PWR06
U 1 1 61875BD5
P 3650 5650
F 0 "#PWR06" H 3650 5500 50  0001 C CNN
F 1 "+5V" H 3665 5823 50  0000 C CNN
F 2 "" H 3650 5650 50  0001 C CNN
F 3 "" H 3650 5650 50  0001 C CNN
	1    3650 5650
	1    0    0    -1  
$EndComp
$Comp
L power:+9V #PWR016
U 1 1 619C1B93
P 7950 4950
F 0 "#PWR016" H 7950 4800 50  0001 C CNN
F 1 "+9V" H 7965 5123 50  0000 C CNN
F 2 "" H 7950 4950 50  0001 C CNN
F 3 "" H 7950 4950 50  0001 C CNN
	1    7950 4950
	1    0    0    -1  
$EndComp
$Comp
L power:+9V #PWR027
U 1 1 619C2C38
P 9750 5600
F 0 "#PWR027" H 9750 5450 50  0001 C CNN
F 1 "+9V" H 9765 5773 50  0000 C CNN
F 2 "" H 9750 5600 50  0001 C CNN
F 3 "" H 9750 5600 50  0001 C CNN
	1    9750 5600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR028
U 1 1 619C9731
P 10650 5300
F 0 "#PWR028" H 10650 5150 50  0001 C CNN
F 1 "+5V" H 10665 5473 50  0000 C CNN
F 2 "" H 10650 5300 50  0001 C CNN
F 3 "" H 10650 5300 50  0001 C CNN
	1    10650 5300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J10
U 1 1 619CC825
P 8250 5950
F 0 "J10" H 8142 5725 50  0000 C CNN
F 1 "GND_Conn" H 8142 5816 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x02_P3.81mm_Drill1mm" H 8250 5950 50  0001 C CNN
F 3 "~" H 8250 5950 50  0001 C CNN
	1    8250 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 4950 7650 4850
Wire Wire Line
	7950 5800 7950 5950
Wire Wire Line
	8050 5950 7950 5950
Connection ~ 7950 5950
Wire Wire Line
	7950 5950 7950 6050
Wire Wire Line
	8050 6050 7950 6050
Connection ~ 7950 6050
Wire Wire Line
	7950 6050 7950 6100
$Comp
L Connector:Conn_01x01_Female J11
U 1 1 61A11151
P 3100 3250
F 0 "J11" H 3050 3400 50  0000 L CNN
F 1 "RST+" H 3050 3300 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 3100 3250 50  0001 C CNN
F 3 "~" H 3100 3250 50  0001 C CNN
	1    3100 3250
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x01_Female J12
U 1 1 61A11A5D
P 3700 3450
F 0 "J12" H 3650 3600 50  0000 L CNN
F 1 "RST-" H 3650 3500 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 3700 3450 50  0001 C CNN
F 3 "~" H 3700 3450 50  0001 C CNN
	1    3700 3450
	1    0    0    -1  
$EndComp
Connection ~ 3500 3450
$Comp
L Device:C C15
U 1 1 61A28CC6
P 9650 5150
F 0 "C15" H 9765 5196 50  0000 L CNN
F 1 "100n" H 9765 5105 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 9688 5000 50  0001 C CNN
F 3 "~" H 9650 5150 50  0001 C CNN
	1    9650 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 5300 9650 5350
Wire Wire Line
	9650 5350 9300 5350
Connection ~ 9300 5350
Wire Wire Line
	9300 4950 9650 4950
Wire Wire Line
	9650 4950 9650 5000
Connection ~ 9750 5600
Wire Wire Line
	6850 3150 7250 3150
Wire Wire Line
	2600 4350 3350 4350
Wire Wire Line
	3350 4350 3350 3950
Wire Wire Line
	3350 3950 5350 3950
Wire Wire Line
	2600 4250 3250 4250
Wire Wire Line
	3250 4250 3250 3850
Wire Wire Line
	3250 3850 5000 3850
Wire Wire Line
	2600 4150 3150 4150
Wire Wire Line
	3150 4150 3150 3750
Wire Wire Line
	3150 3750 4600 3750
NoConn ~ 2600 3950
NoConn ~ 2600 3850
NoConn ~ 2600 3750
$Comp
L Connector:Conn_01x01_Female J14
U 1 1 618B1B0E
P 2800 2950
F 0 "J14" H 2850 2950 50  0000 L CNN
F 1 "A1" H 2750 3000 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 2800 2950 50  0001 C CNN
F 3 "~" H 2800 2950 50  0001 C CNN
	1    2800 2950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J13
U 1 1 61AFD642
P 2800 1950
F 0 "J13" H 2750 2100 50  0000 L CNN
F 1 "LED4" H 2750 2000 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1mm" H 2800 1950 50  0001 C CNN
F 3 "~" H 2800 1950 50  0001 C CNN
	1    2800 1950
	1    0    0    -1  
$EndComp
$EndSCHEMATC
