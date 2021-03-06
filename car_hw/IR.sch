EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 10 10
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
L Amplifier_Operational:LM358 U7
U 1 1 605B1E76
P 3050 2850
AR Path="/605B9A30/605B1E76" Ref="U7"  Part="1" 
AR Path="/605E3556/605B1E76" Ref="U8"  Part="1" 
AR Path="/605E3914/605B1E76" Ref="U9"  Part="1" 
AR Path="/605E3E72/605B1E76" Ref="U10"  Part="1" 
AR Path="/605E4182/605B1E76" Ref="U11"  Part="1" 
AR Path="/605F647C/605B1E76" Ref="U12"  Part="1" 
F 0 "U7" H 3050 3217 50  0000 C CNN
F 1 "LM358" H 3050 3126 50  0000 C CNN
F 2 "Package_SO:SOP-8_3.9x4.9mm_P1.27mm" H 3050 2850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 3050 2850 50  0001 C CNN
	1    3050 2850
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM358 U7
U 2 1 605B1E7C
P 4250 2950
AR Path="/605B9A30/605B1E7C" Ref="U7"  Part="2" 
AR Path="/605E3556/605B1E7C" Ref="U8"  Part="2" 
AR Path="/605E3914/605B1E7C" Ref="U9"  Part="2" 
AR Path="/605E3E72/605B1E7C" Ref="U10"  Part="2" 
AR Path="/605E4182/605B1E7C" Ref="U11"  Part="2" 
AR Path="/605F647C/605B1E7C" Ref="U12"  Part="2" 
F 0 "U7" H 4250 3317 50  0000 C CNN
F 1 "LM358" H 4250 3226 50  0000 C CNN
F 2 "Package_SO:SOP-8_3.9x4.9mm_P1.27mm" H 4250 2950 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 4250 2950 50  0001 C CNN
	2    4250 2950
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM358 U7
U 3 1 605B1E82
P 2050 3950
AR Path="/605B9A30/605B1E82" Ref="U7"  Part="3" 
AR Path="/605E3556/605B1E82" Ref="U8"  Part="3" 
AR Path="/605E3914/605B1E82" Ref="U9"  Part="3" 
AR Path="/605E3E72/605B1E82" Ref="U10"  Part="3" 
AR Path="/605E4182/605B1E82" Ref="U11"  Part="3" 
AR Path="/605F647C/605B1E82" Ref="U12"  Part="3" 
F 0 "U7" H 2008 3996 50  0000 L CNN
F 1 "LM358" H 2008 3905 50  0000 L CNN
F 2 "Package_SO:SOP-8_3.9x4.9mm_P1.27mm" H 2050 3950 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 2050 3950 50  0001 C CNN
	3    2050 3950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C28
U 1 1 605B1E88
P 3500 2850
AR Path="/605B9A30/605B1E88" Ref="C28"  Part="1" 
AR Path="/605E3556/605B1E88" Ref="C32"  Part="1" 
AR Path="/605E3914/605B1E88" Ref="C36"  Part="1" 
AR Path="/605E3E72/605B1E88" Ref="C40"  Part="1" 
AR Path="/605E4182/605B1E88" Ref="C44"  Part="1" 
AR Path="/605F647C/605B1E88" Ref="C48"  Part="1" 
F 0 "C28" V 3248 2850 50  0000 C CNN
F 1 "1u" V 3339 2850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3538 2700 50  0001 C CNN
F 3 "~" H 3500 2850 50  0001 C CNN
	1    3500 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R R32
U 1 1 605B1E8E
P 3650 3000
AR Path="/605B9A30/605B1E8E" Ref="R32"  Part="1" 
AR Path="/605E3556/605B1E8E" Ref="R40"  Part="1" 
AR Path="/605E3914/605B1E8E" Ref="R48"  Part="1" 
AR Path="/605E3E72/605B1E8E" Ref="R56"  Part="1" 
AR Path="/605E4182/605B1E8E" Ref="R64"  Part="1" 
AR Path="/605F647C/605B1E8E" Ref="R72"  Part="1" 
F 0 "R32" H 3500 2950 50  0000 C CNN
F 1 "68k" H 3500 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3580 3000 50  0001 C CNN
F 3 "~" H 3650 3000 50  0001 C CNN
	1    3650 3000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0159
U 1 1 605B1E94
P 3650 3150
AR Path="/605B9A30/605B1E94" Ref="#PWR0159"  Part="1" 
AR Path="/605E3556/605B1E94" Ref="#PWR0165"  Part="1" 
AR Path="/605E3914/605B1E94" Ref="#PWR0171"  Part="1" 
AR Path="/605E3E72/605B1E94" Ref="#PWR0177"  Part="1" 
AR Path="/605E4182/605B1E94" Ref="#PWR0183"  Part="1" 
AR Path="/605F647C/605B1E94" Ref="#PWR0189"  Part="1" 
F 0 "#PWR0159" H 3650 2900 50  0001 C CNN
F 1 "GND" H 3655 2977 50  0000 C CNN
F 2 "" H 3650 3150 50  0001 C CNN
F 3 "" H 3650 3150 50  0001 C CNN
	1    3650 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R31
U 1 1 605B1E9A
P 2750 3350
AR Path="/605B9A30/605B1E9A" Ref="R31"  Part="1" 
AR Path="/605E3556/605B1E9A" Ref="R39"  Part="1" 
AR Path="/605E3914/605B1E9A" Ref="R47"  Part="1" 
AR Path="/605E3E72/605B1E9A" Ref="R55"  Part="1" 
AR Path="/605E4182/605B1E9A" Ref="R63"  Part="1" 
AR Path="/605F647C/605B1E9A" Ref="R71"  Part="1" 
F 0 "R31" H 2600 3300 50  0000 C CNN
F 1 "8k2" H 2600 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2680 3350 50  0001 C CNN
F 3 "~" H 2750 3350 50  0001 C CNN
	1    2750 3350
	-1   0    0    1   
$EndComp
$Comp
L Device:C C27
U 1 1 605B1EA0
P 3200 3200
AR Path="/605B9A30/605B1EA0" Ref="C27"  Part="1" 
AR Path="/605E3556/605B1EA0" Ref="C31"  Part="1" 
AR Path="/605E3914/605B1EA0" Ref="C35"  Part="1" 
AR Path="/605E3E72/605B1EA0" Ref="C39"  Part="1" 
AR Path="/605E4182/605B1EA0" Ref="C43"  Part="1" 
AR Path="/605F647C/605B1EA0" Ref="C47"  Part="1" 
F 0 "C27" V 2948 3200 50  0000 C CNN
F 1 "100n" V 3039 3200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3238 3050 50  0001 C CNN
F 3 "~" H 3200 3200 50  0001 C CNN
	1    3200 3200
	0    1    1    0   
$EndComp
Connection ~ 3350 2850
Wire Wire Line
	3350 2850 3350 3200
Wire Wire Line
	2750 2950 2750 3200
Wire Wire Line
	2750 3200 3050 3200
Connection ~ 2750 3200
$Comp
L Device:R R30
U 1 1 605B1EAB
P 2450 3350
AR Path="/605B9A30/605B1EAB" Ref="R30"  Part="1" 
AR Path="/605E3556/605B1EAB" Ref="R38"  Part="1" 
AR Path="/605E3914/605B1EAB" Ref="R46"  Part="1" 
AR Path="/605E3E72/605B1EAB" Ref="R54"  Part="1" 
AR Path="/605E4182/605B1EAB" Ref="R62"  Part="1" 
AR Path="/605F647C/605B1EAB" Ref="R70"  Part="1" 
F 0 "R30" H 2300 3300 50  0000 C CNN
F 1 "68k" H 2300 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2380 3350 50  0001 C CNN
F 3 "~" H 2450 3350 50  0001 C CNN
	1    2450 3350
	-1   0    0    1   
$EndComp
Wire Wire Line
	2750 2750 2450 2750
Wire Wire Line
	2450 2750 2450 3200
$Comp
L power:GND #PWR0160
U 1 1 605B1EB3
P 2750 3500
AR Path="/605B9A30/605B1EB3" Ref="#PWR0160"  Part="1" 
AR Path="/605E3556/605B1EB3" Ref="#PWR0166"  Part="1" 
AR Path="/605E3914/605B1EB3" Ref="#PWR0172"  Part="1" 
AR Path="/605E3E72/605B1EB3" Ref="#PWR0178"  Part="1" 
AR Path="/605E4182/605B1EB3" Ref="#PWR0184"  Part="1" 
AR Path="/605F647C/605B1EB3" Ref="#PWR0190"  Part="1" 
F 0 "#PWR0160" H 2750 3250 50  0001 C CNN
F 1 "GND" H 2755 3327 50  0000 C CNN
F 2 "" H 2750 3500 50  0001 C CNN
F 3 "" H 2750 3500 50  0001 C CNN
	1    2750 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3500 2750 3500
Connection ~ 2750 3500
Wire Wire Line
	3650 2850 3950 2850
Connection ~ 3650 2850
$Comp
L Device:R R34
U 1 1 605B1EBD
P 4100 3450
AR Path="/605B9A30/605B1EBD" Ref="R34"  Part="1" 
AR Path="/605E3556/605B1EBD" Ref="R42"  Part="1" 
AR Path="/605E3914/605B1EBD" Ref="R50"  Part="1" 
AR Path="/605E3E72/605B1EBD" Ref="R58"  Part="1" 
AR Path="/605E4182/605B1EBD" Ref="R66"  Part="1" 
AR Path="/605F647C/605B1EBD" Ref="R74"  Part="1" 
F 0 "R34" V 3900 3400 50  0000 L CNN
F 1 "470k" V 4000 3400 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4030 3450 50  0001 C CNN
F 3 "~" H 4100 3450 50  0001 C CNN
	1    4100 3450
	0    1    1    0   
$EndComp
$Comp
L Device:C C29
U 1 1 605B1EC3
P 4100 3800
AR Path="/605B9A30/605B1EC3" Ref="C29"  Part="1" 
AR Path="/605E3556/605B1EC3" Ref="C33"  Part="1" 
AR Path="/605E3914/605B1EC3" Ref="C37"  Part="1" 
AR Path="/605E3E72/605B1EC3" Ref="C41"  Part="1" 
AR Path="/605E4182/605B1EC3" Ref="C45"  Part="1" 
AR Path="/605F647C/605B1EC3" Ref="C49"  Part="1" 
F 0 "C29" V 3848 3800 50  0000 C CNN
F 1 "100n" V 3939 3800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4138 3650 50  0001 C CNN
F 3 "~" H 4100 3800 50  0001 C CNN
	1    4100 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	3950 3050 3950 3450
Connection ~ 3950 3450
Wire Wire Line
	3950 3450 3950 3800
Wire Wire Line
	4550 2950 4550 3450
Wire Wire Line
	4550 3800 4250 3800
Wire Wire Line
	4250 3450 4550 3450
Connection ~ 4550 3450
Wire Wire Line
	4550 3450 4550 3800
$Comp
L Device:R R33
U 1 1 605B1ED1
P 3800 3800
AR Path="/605B9A30/605B1ED1" Ref="R33"  Part="1" 
AR Path="/605E3556/605B1ED1" Ref="R41"  Part="1" 
AR Path="/605E3914/605B1ED1" Ref="R49"  Part="1" 
AR Path="/605E3E72/605B1ED1" Ref="R57"  Part="1" 
AR Path="/605E4182/605B1ED1" Ref="R65"  Part="1" 
AR Path="/605F647C/605B1ED1" Ref="R73"  Part="1" 
F 0 "R33" V 3600 3750 50  0000 L CNN
F 1 "1k" V 3700 3750 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3730 3800 50  0001 C CNN
F 3 "~" H 3800 3800 50  0001 C CNN
	1    3800 3800
	0    1    1    0   
$EndComp
Connection ~ 3950 3800
$Comp
L power:GND #PWR0161
U 1 1 605B1ED8
P 3650 3800
AR Path="/605B9A30/605B1ED8" Ref="#PWR0161"  Part="1" 
AR Path="/605E3556/605B1ED8" Ref="#PWR0167"  Part="1" 
AR Path="/605E3914/605B1ED8" Ref="#PWR0173"  Part="1" 
AR Path="/605E3E72/605B1ED8" Ref="#PWR0179"  Part="1" 
AR Path="/605E4182/605B1ED8" Ref="#PWR0185"  Part="1" 
AR Path="/605F647C/605B1ED8" Ref="#PWR0191"  Part="1" 
F 0 "#PWR0161" H 3650 3550 50  0001 C CNN
F 1 "GND" H 3655 3627 50  0000 C CNN
F 2 "" H 3650 3800 50  0001 C CNN
F 3 "" H 3650 3800 50  0001 C CNN
	1    3650 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C26
U 1 1 605B1EDE
P 2300 2750
AR Path="/605B9A30/605B1EDE" Ref="C26"  Part="1" 
AR Path="/605E3556/605B1EDE" Ref="C30"  Part="1" 
AR Path="/605E3914/605B1EDE" Ref="C34"  Part="1" 
AR Path="/605E3E72/605B1EDE" Ref="C38"  Part="1" 
AR Path="/605E4182/605B1EDE" Ref="C42"  Part="1" 
AR Path="/605F647C/605B1EDE" Ref="C46"  Part="1" 
F 0 "C26" V 2048 2750 50  0000 C CNN
F 1 "100n" V 2139 2750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2338 2600 50  0001 C CNN
F 3 "~" H 2300 2750 50  0001 C CNN
	1    2300 2750
	0    1    1    0   
$EndComp
Connection ~ 2450 2750
$Comp
L Device:R R35
U 1 1 605B1EE5
P 4700 2950
AR Path="/605B9A30/605B1EE5" Ref="R35"  Part="1" 
AR Path="/605E3556/605B1EE5" Ref="R43"  Part="1" 
AR Path="/605E3914/605B1EE5" Ref="R51"  Part="1" 
AR Path="/605E3E72/605B1EE5" Ref="R59"  Part="1" 
AR Path="/605E4182/605B1EE5" Ref="R67"  Part="1" 
AR Path="/605F647C/605B1EE5" Ref="R75"  Part="1" 
F 0 "R35" V 4500 2900 50  0000 L CNN
F 1 "1k8" V 4600 2900 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4630 2950 50  0001 C CNN
F 3 "~" H 4700 2950 50  0001 C CNN
	1    4700 2950
	0    1    1    0   
$EndComp
Connection ~ 4550 2950
$Comp
L Device:R R29
U 1 1 605B1EEC
P 1950 2600
AR Path="/605B9A30/605B1EEC" Ref="R29"  Part="1" 
AR Path="/605E3556/605B1EEC" Ref="R37"  Part="1" 
AR Path="/605E3914/605B1EEC" Ref="R45"  Part="1" 
AR Path="/605E3E72/605B1EEC" Ref="R53"  Part="1" 
AR Path="/605E4182/605B1EEC" Ref="R61"  Part="1" 
AR Path="/605F647C/605B1EEC" Ref="R69"  Part="1" 
F 0 "R29" H 1800 2550 50  0000 C CNN
F 1 "39k" H 1800 2650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1880 2600 50  0001 C CNN
F 3 "~" H 1950 2600 50  0001 C CNN
	1    1950 2600
	-1   0    0    1   
$EndComp
Wire Wire Line
	1950 2750 2150 2750
$Comp
L power:GND #PWR0162
U 1 1 605B1EFF
P 1950 4250
AR Path="/605B9A30/605B1EFF" Ref="#PWR0162"  Part="1" 
AR Path="/605E3556/605B1EFF" Ref="#PWR0168"  Part="1" 
AR Path="/605E3914/605B1EFF" Ref="#PWR0174"  Part="1" 
AR Path="/605E3E72/605B1EFF" Ref="#PWR0180"  Part="1" 
AR Path="/605E4182/605B1EFF" Ref="#PWR0186"  Part="1" 
AR Path="/605F647C/605B1EFF" Ref="#PWR0192"  Part="1" 
F 0 "#PWR0162" H 1950 4000 50  0001 C CNN
F 1 "GND" H 1955 4077 50  0000 C CNN
F 2 "" H 1950 4250 50  0001 C CNN
F 3 "" H 1950 4250 50  0001 C CNN
	1    1950 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D6
U 1 1 605B1F05
P 1950 2900
AR Path="/605B9A30/605B1F05" Ref="D6"  Part="1" 
AR Path="/605E3556/605B1F05" Ref="D8"  Part="1" 
AR Path="/605E3914/605B1F05" Ref="D10"  Part="1" 
AR Path="/605E3E72/605B1F05" Ref="D12"  Part="1" 
AR Path="/605E4182/605B1F05" Ref="D14"  Part="1" 
AR Path="/605F647C/605B1F05" Ref="D16"  Part="1" 
F 0 "D6" V 1989 2782 50  0000 R CNN
F 1 "IR_Receiver" V 1898 2782 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm" H 1950 2900 50  0001 C CNN
F 3 "~" H 1950 2900 50  0001 C CNN
	1    1950 2900
	0    -1   -1   0   
$EndComp
Connection ~ 1950 2750
$Comp
L power:GND #PWR0163
U 1 1 605B1F0C
P 1950 3050
AR Path="/605B9A30/605B1F0C" Ref="#PWR0163"  Part="1" 
AR Path="/605E3556/605B1F0C" Ref="#PWR0169"  Part="1" 
AR Path="/605E3914/605B1F0C" Ref="#PWR0175"  Part="1" 
AR Path="/605E3E72/605B1F0C" Ref="#PWR0181"  Part="1" 
AR Path="/605E4182/605B1F0C" Ref="#PWR0187"  Part="1" 
AR Path="/605F647C/605B1F0C" Ref="#PWR0193"  Part="1" 
F 0 "#PWR0163" H 1950 2800 50  0001 C CNN
F 1 "GND" H 1955 2877 50  0000 C CNN
F 2 "" H 1950 3050 50  0001 C CNN
F 3 "" H 1950 3050 50  0001 C CNN
	1    1950 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R28
U 1 1 605B1F12
P 1400 2600
AR Path="/605B9A30/605B1F12" Ref="R28"  Part="1" 
AR Path="/605E3556/605B1F12" Ref="R36"  Part="1" 
AR Path="/605E3914/605B1F12" Ref="R44"  Part="1" 
AR Path="/605E3E72/605B1F12" Ref="R52"  Part="1" 
AR Path="/605E4182/605B1F12" Ref="R60"  Part="1" 
AR Path="/605F647C/605B1F12" Ref="R68"  Part="1" 
F 0 "R28" H 1250 2550 50  0000 C CNN
F 1 "220" H 1250 2650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1330 2600 50  0001 C CNN
F 3 "~" H 1400 2600 50  0001 C CNN
	1    1400 2600
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D5
U 1 1 605B1F18
P 1400 2900
AR Path="/605B9A30/605B1F18" Ref="D5"  Part="1" 
AR Path="/605E3556/605B1F18" Ref="D7"  Part="1" 
AR Path="/605E3914/605B1F18" Ref="D9"  Part="1" 
AR Path="/605E3E72/605B1F18" Ref="D11"  Part="1" 
AR Path="/605E4182/605B1F18" Ref="D13"  Part="1" 
AR Path="/605F647C/605B1F18" Ref="D15"  Part="1" 
F 0 "D5" V 1439 2782 50  0000 R CNN
F 1 "IR_Sender" V 1348 2782 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm" H 1400 2900 50  0001 C CNN
F 3 "~" H 1400 2900 50  0001 C CNN
	1    1400 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0164
U 1 1 605B1F24
P 1400 3050
AR Path="/605B9A30/605B1F24" Ref="#PWR0164"  Part="1" 
AR Path="/605E3556/605B1F24" Ref="#PWR0170"  Part="1" 
AR Path="/605E3914/605B1F24" Ref="#PWR0176"  Part="1" 
AR Path="/605E3E72/605B1F24" Ref="#PWR0182"  Part="1" 
AR Path="/605E4182/605B1F24" Ref="#PWR0188"  Part="1" 
AR Path="/605F647C/605B1F24" Ref="#PWR0194"  Part="1" 
F 0 "#PWR0164" H 1400 2800 50  0001 C CNN
F 1 "GND" H 1405 2877 50  0000 C CNN
F 2 "" H 1400 3050 50  0001 C CNN
F 3 "" H 1400 3050 50  0001 C CNN
	1    1400 3050
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC817 Q5
U 1 1 605B1F32
P 5050 2950
AR Path="/605B9A30/605B1F32" Ref="Q5"  Part="1" 
AR Path="/605E3556/605B1F32" Ref="Q6"  Part="1" 
AR Path="/605E3914/605B1F32" Ref="Q7"  Part="1" 
AR Path="/605E3E72/605B1F32" Ref="Q8"  Part="1" 
AR Path="/605E4182/605B1F32" Ref="Q9"  Part="1" 
AR Path="/605F647C/605B1F32" Ref="Q10"  Part="1" 
F 0 "Q5" H 5241 2996 50  0000 L CNN
F 1 "BC817" H 5241 2905 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5250 2875 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/BC818-D.pdf" H 5050 2950 50  0001 L CNN
	1    5050 2950
	1    0    0    -1  
$EndComp
Text HLabel 1400 2450 1    50   Input ~ 0
VCC
Text HLabel 1950 2450 1    50   Input ~ 0
VCC
Text HLabel 5150 2750 1    50   Input ~ 0
VCC
Text HLabel 1950 3650 1    50   Input ~ 0
VCC
Text HLabel 5150 3150 2    50   Input ~ 0
IR
$EndSCHEMATC
