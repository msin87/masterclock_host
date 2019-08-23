# UART commands list  

List of commands for controlling the microcontroller via UART  

[TOC]



## Main structure of commands and responses  

|   0    |   1    |  2   | ...  |  27  |    28    |    29    |    30    |    31    |
| :----: | :----: | :--: | :--: | :--: | :------: | :------: | :------: | :------: |
| CMD[1] | CMD[0] | DATA | DATA | DATA | CRC32[3] | CRC32[2] | CRC32[1] | CRC32[0] |

  *Total frame length: 32 bytes.*   

## Commands  

*Frame formats are given without CRC32 bytes.*

### 0x0000 Start/Restart host (stm32)

### 0x0001 Set the number *N* of pulses to send for the specified lines

*Frame format*:

|  0..1  |       2        |       3        |  4   |  5   | ...  |  26  |  27  |
| :----: | :------------: | :------------: | :--: | :--: | :--: | :--: | :--: |
| 0x0001 | ID<sub>1</sub> | ID<sub>0</sub> | N[1] | N[0] | ...  | N[1] | N[0] |

*Explanations*:
**In ID<sub>1</sub> and ID<sub>0</sub> bytes**, the numbers (identifiers) of the lines for which data (number of pulses) will be transmitted in the next bytes are set by bits. ID<sub>0</sub> allows you to set lines with id from 0 to 7. ID<sub>1</sub> - id from 8 to 11. Next are the high and low bytes of the `uint16` type number for the lines specified earlier. First comes the number for the line with the lowest id contained in ID<sub>0</sub> and ID<sub>1</sub> bytes.

| bit  | ID<sub>1</sub>[7] | ID<sub>1</sub>[6] | ID<sub>1</sub>[5] | ID<sub>1</sub>[4] | ID<sub>1</sub>[3] | ID<sub>1</sub>[2] | ID<sub>1</sub>[1] | ID<sub>1</sub>[0] | ID<sub>0</sub>[7] | ID<sub>0</sub>[6] | ID<sub>0</sub>[5] | ID<sub>0</sub>[4] | ID<sub>0</sub>[3] | ID<sub>0</sub>[2] | ID<sub>0</sub>[1] | ID<sub>0</sub>[0] |
| :--: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: | :---------------: |
| line |         -         |         -         |         -         |         -         |        11         |        10         |         9         |         8         |         7         |         6         |         5         |         4         |         3         |         2         |         1         |         0         |

*Example* :

```
(hex): 00 01 04 5A 02 D0 02 CF 02 CE 02 CD 02 CC
```

|  data   |             description             |
| :-----: | :---------------------------------: |
| `00 01` |                 CMD                 |
|  `04`   |        line ID<sub>10</sub>         |
|  `5A`   |     lines ID<sub>6,4,3,1</sub>      |
| `D0 02` | 720 pulses for line ID<sub>1</sub>  |
| `02 CF` | 719 pulses for line ID<sub>3</sub>  |
| `02 CE` | 718 pulses for line ID<sub>4</sub>  |
| `02 CD` | 717 pulses for line ID<sub>6</sub>  |
| `02 CC` | 716 pulses for line ID<sub>10</sub> |

### 0x0002 Increment the pulse counter by 1 for the specified lines

*Frame format*:

|  0..1  |  2   |  3   |
| :----: | :--: | :--: |
| 0x0002 | ID1  | ID0  |

*Explanations*: 
Increases the pulse counter for sending to the line by 1 without overwriting the counter. Also allows you to add one minute during the process of setting the correct time (that is, sending a long sequence of pulses)
*Example* :

```
(hex): 00 02 04 5A
```

See command 0x01. 

### 0x0003 Reset pulse counter for the specified lines

*Frame format*:

|  0..1  |       2        |       3        |
| :----: | :------------: | :------------: |
| 0x0003 | <sub>ID1</sub> | <sub>ID0</sub> |

*Explanations*: 
Resets the pulse counter to 0.
*Example* :

```
(hex): 00 03 04 5A
```

See command 0x0001. 

### 0x0004 Suspend pulse counter processing for the specified lines.

*Frame format*:

|  0..1  |       2        |  3   |
| :----: | :------------: | :--: |
| 0x0004 | <sub>ID1</sub> | ID0  |

*Explanations*: 
Sending pulses is blocked. Changing the pulse counter through the UART is not blocked. It is used when receiving an overload signal or for pausing a clock adjustment,
*Example* :

```
 (hex): 00 04 04 5A
```

See command 0x0001. 

### 0x0005 Resume pulse counter processing for the specified lines.

*Frame format*:

|  0..1  |       2        |       3        |
| :----: | :------------: | :------------: |
| 0x0005 | ID<sub>1</sub> | ID<sub>0</sub> |

*Explanations*: 
Resume pulse counter processing after suspend command 0x0004
*Example* :

```
 (hex): 00 05 04 5A
```

See command 0x0001. 

### 0x0006 Set pulse width

*Frame format*:

|  0..1  |       2        |       3        |    4     |    5     | ...  |    26    |    27    |
| :----: | :------------: | :------------: | :------: | :------: | :--: | :------: | :------: |
| 0x0005 | ID<sub>1</sub> | ID<sub>0</sub> | WIDTH[1] | WIDTH[0] |      | WIDTH[1] | WIDTH[0] |

*Explanations*: 
Set pulse width in milliseconds. 

*Example* :

```
 (hex): 00 06 04 5A 05 DC 0F A0 1F 04 1F 04 05 DC
```

|  data   |           description            |
| :-----: | :------------------------------: |
| `00 06` |               CMD                |
|  `04`   |       line ID<sub>10</sub>       |
|  `5A`   |    lines ID<sub>6,4,3,1</sub>    |
| `05 DC` | 1500 ms for line ID<sub>1</sub>  |
| `0F A0` | 4000 ms for line ID<sub>3</sub>  |
| `1F 04` |  500 ms for line ID<sub>4</sub>  |
| `1F 04` |  500 ms for line ID<sub>6</sub>  |
| `05 DC` | 1500 ms for line ID<sub>10</sub> |

### 0x0007 Set polarity of previous pulse

*Frame format*:

|  0..1  |  2   |  3   |   4    |   5    | ...  |   26   |   27   |
| :----: | :--: | :--: | :----: | :----: | :--: | :----: | :----: |
| 0x0007 | ID1  | ID0  | POL[1] | POL[0] |      | POL[1] | POL[0] |

*Explanations*:
Set polarity of previous pulse.

*Example* :

```
(hex): 00 07 04 5A 00 01 00 00 00 00 00 01 00 01
```

|  data   |             description              |
| :-----: | :----------------------------------: |
| `00 07` |                 CMD                  |
|  `04`   |              line ID10               |
|  `5A`   |           lines ID6,4,3,1            |
| `00 01` | set '+' prev. polarity for line ID1  |
| `00 00` | set '-' prev. polarity for line ID3  |
| `00 00` | set '-' prev. polarity for line ID4  |
| `00 01` | set '+' prev. polarity for line ID6  |
| `00 01` | set '+' prev. polarity for line ID10 |

### 0x0008 Set relay state

*Frame format*:

|  0..1  |  2   |  3   |    4     |    5     | ...  |    26    |    27    |
| :----: | :--: | :--: | :------: | :------: | :--: | :------: | :------: |
| 0x0008 | ID1  | ID0  | RELAY[1] | RELAY[0] |      | RELAY[1] | RELAY[0] |

*Explanations*:
Set relay state.

*Example* :

```
(hex): 00 08 04 5A 00 01 00 00 00 00 00 01 00 01
```

|  data   |    description     |
| :-----: | :----------------: |
| `00 08` |        CMD         |
|  `04`   |     line ID10      |
|  `5A`   |  lines ID6,4,3,1   |
| `00 01` | turn on relay ID1  |
| `00 00` | turn off relay ID3 |
| `00 00` | turn off relay ID4 |
| `00 01` | turn on relay ID6  |
| `00 01` | turn on relay ID10 |



## Responses 

*Frame formats are given without CRC32 bytes.*

### ~~0x00  *Reserved~~*

### 0x0001 Pulse counters

 *Frame format*: 

|  0..1  |       2        |       3        |  4   |  5   | ...  |  26  |  27  |
| :----: | :------------: | :------------: | :--: | :--: | :--: | :--: | :--: |
| 0x0001 | ID<sub>1</sub> | ID<sub>0</sub> | N[1] | N[0] | ...  | N[1] | N[0] |

*Explanations*: 
It is returned after each sending of the pulse. Contains the current states of the pulse counters.

### 0x0002 Measured ADC (12 bit) value of current

*Frame format*: 

|  0..1  |       2        |       3        |  4   |  5   | ...  |  26  |  27  |
| :----: | :------------: | :------------: | :--: | :--: | :--: | :--: | :--: |
| 0x0002 | ID<sub>1</sub> | ID<sub>0</sub> | I[1] | I[0] | ...  | I[1] | I[0] |

*Explanations*: 
It is returned current in milliamperes  after each sending of the pulse.
ID<sub>12</sub> corresponds to the total current consumption received from the total current sensor. ID<sub>12</sub> is transmitted in a separate frame.

### 0x0007 Current pulse polarity

*Frame format*: 

|  0..1  |       2        |       3        |   4    |   5    | ...  |   26   |   27   |
| :----: | :------------: | :------------: | :----: | :----: | :--: | :----: | :----: |
| 0x0007 | ID<sub>1</sub> | ID<sub>0</sub> | POL[1] | POL[0] | ...  | POL[1] | POL[0] |

*Explanations*: 
Returns the polarity of the rising edge of the pulse for lines. POL: 0x0000 = '-', 0x0001 = '+'. 