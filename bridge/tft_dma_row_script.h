static const __attribute__((__section__(".flashdata.tft_dma_script"))) void *
dma_row_script[2][TFT_RAW_HEIGHT + 1] = {
#define TFT_SCRIPT_ROW(x) ((TFT_RAW_WIDTH / TFT_SCALE) * ((x) / TFT_SCALE))
	{
#if TFT_RAW_HEIGHT >= 0
		&buffer[0][TFT_SCRIPT_ROW(0)],
#endif
#if TFT_RAW_HEIGHT >= 1
		&buffer[0][TFT_SCRIPT_ROW(1)],
#endif
#if TFT_RAW_HEIGHT >= 2
		&buffer[0][TFT_SCRIPT_ROW(2)],
#endif
#if TFT_RAW_HEIGHT >= 3
		&buffer[0][TFT_SCRIPT_ROW(3)],
#endif
#if TFT_RAW_HEIGHT >= 4
		&buffer[0][TFT_SCRIPT_ROW(4)],
#endif
#if TFT_RAW_HEIGHT >= 5
		&buffer[0][TFT_SCRIPT_ROW(5)],
#endif
#if TFT_RAW_HEIGHT >= 6
		&buffer[0][TFT_SCRIPT_ROW(6)],
#endif
#if TFT_RAW_HEIGHT >= 7
		&buffer[0][TFT_SCRIPT_ROW(7)],
#endif
#if TFT_RAW_HEIGHT >= 8
		&buffer[0][TFT_SCRIPT_ROW(8)],
#endif
#if TFT_RAW_HEIGHT >= 9
		&buffer[0][TFT_SCRIPT_ROW(9)],
#endif
#if TFT_RAW_HEIGHT >= 10
		&buffer[0][TFT_SCRIPT_ROW(10)],
#endif
#if TFT_RAW_HEIGHT >= 11
		&buffer[0][TFT_SCRIPT_ROW(11)],
#endif
#if TFT_RAW_HEIGHT >= 12
		&buffer[0][TFT_SCRIPT_ROW(12)],
#endif
#if TFT_RAW_HEIGHT >= 13
		&buffer[0][TFT_SCRIPT_ROW(13)],
#endif
#if TFT_RAW_HEIGHT >= 14
		&buffer[0][TFT_SCRIPT_ROW(14)],
#endif
#if TFT_RAW_HEIGHT >= 15
		&buffer[0][TFT_SCRIPT_ROW(15)],
#endif
#if TFT_RAW_HEIGHT >= 16
		&buffer[0][TFT_SCRIPT_ROW(16)],
#endif
#if TFT_RAW_HEIGHT >= 17
		&buffer[0][TFT_SCRIPT_ROW(17)],
#endif
#if TFT_RAW_HEIGHT >= 18
		&buffer[0][TFT_SCRIPT_ROW(18)],
#endif
#if TFT_RAW_HEIGHT >= 19
		&buffer[0][TFT_SCRIPT_ROW(19)],
#endif
#if TFT_RAW_HEIGHT >= 20
		&buffer[0][TFT_SCRIPT_ROW(20)],
#endif
#if TFT_RAW_HEIGHT >= 21
		&buffer[0][TFT_SCRIPT_ROW(21)],
#endif
#if TFT_RAW_HEIGHT >= 22
		&buffer[0][TFT_SCRIPT_ROW(22)],
#endif
#if TFT_RAW_HEIGHT >= 23
		&buffer[0][TFT_SCRIPT_ROW(23)],
#endif
#if TFT_RAW_HEIGHT >= 24
		&buffer[0][TFT_SCRIPT_ROW(24)],
#endif
#if TFT_RAW_HEIGHT >= 25
		&buffer[0][TFT_SCRIPT_ROW(25)],
#endif
#if TFT_RAW_HEIGHT >= 26
		&buffer[0][TFT_SCRIPT_ROW(26)],
#endif
#if TFT_RAW_HEIGHT >= 27
		&buffer[0][TFT_SCRIPT_ROW(27)],
#endif
#if TFT_RAW_HEIGHT >= 28
		&buffer[0][TFT_SCRIPT_ROW(28)],
#endif
#if TFT_RAW_HEIGHT >= 29
		&buffer[0][TFT_SCRIPT_ROW(29)],
#endif
#if TFT_RAW_HEIGHT >= 30
		&buffer[0][TFT_SCRIPT_ROW(30)],
#endif
#if TFT_RAW_HEIGHT >= 31
		&buffer[0][TFT_SCRIPT_ROW(31)],
#endif
#if TFT_RAW_HEIGHT >= 32
		&buffer[0][TFT_SCRIPT_ROW(32)],
#endif
#if TFT_RAW_HEIGHT >= 33
		&buffer[0][TFT_SCRIPT_ROW(33)],
#endif
#if TFT_RAW_HEIGHT >= 34
		&buffer[0][TFT_SCRIPT_ROW(34)],
#endif
#if TFT_RAW_HEIGHT >= 35
		&buffer[0][TFT_SCRIPT_ROW(35)],
#endif
#if TFT_RAW_HEIGHT >= 36
		&buffer[0][TFT_SCRIPT_ROW(36)],
#endif
#if TFT_RAW_HEIGHT >= 37
		&buffer[0][TFT_SCRIPT_ROW(37)],
#endif
#if TFT_RAW_HEIGHT >= 38
		&buffer[0][TFT_SCRIPT_ROW(38)],
#endif
#if TFT_RAW_HEIGHT >= 39
		&buffer[0][TFT_SCRIPT_ROW(39)],
#endif
#if TFT_RAW_HEIGHT >= 40
		&buffer[0][TFT_SCRIPT_ROW(40)],
#endif
#if TFT_RAW_HEIGHT >= 41
		&buffer[0][TFT_SCRIPT_ROW(41)],
#endif
#if TFT_RAW_HEIGHT >= 42
		&buffer[0][TFT_SCRIPT_ROW(42)],
#endif
#if TFT_RAW_HEIGHT >= 43
		&buffer[0][TFT_SCRIPT_ROW(43)],
#endif
#if TFT_RAW_HEIGHT >= 44
		&buffer[0][TFT_SCRIPT_ROW(44)],
#endif
#if TFT_RAW_HEIGHT >= 45
		&buffer[0][TFT_SCRIPT_ROW(45)],
#endif
#if TFT_RAW_HEIGHT >= 46
		&buffer[0][TFT_SCRIPT_ROW(46)],
#endif
#if TFT_RAW_HEIGHT >= 47
		&buffer[0][TFT_SCRIPT_ROW(47)],
#endif
#if TFT_RAW_HEIGHT >= 48
		&buffer[0][TFT_SCRIPT_ROW(48)],
#endif
#if TFT_RAW_HEIGHT >= 49
		&buffer[0][TFT_SCRIPT_ROW(49)],
#endif
#if TFT_RAW_HEIGHT >= 50
		&buffer[0][TFT_SCRIPT_ROW(50)],
#endif
#if TFT_RAW_HEIGHT >= 51
		&buffer[0][TFT_SCRIPT_ROW(51)],
#endif
#if TFT_RAW_HEIGHT >= 52
		&buffer[0][TFT_SCRIPT_ROW(52)],
#endif
#if TFT_RAW_HEIGHT >= 53
		&buffer[0][TFT_SCRIPT_ROW(53)],
#endif
#if TFT_RAW_HEIGHT >= 54
		&buffer[0][TFT_SCRIPT_ROW(54)],
#endif
#if TFT_RAW_HEIGHT >= 55
		&buffer[0][TFT_SCRIPT_ROW(55)],
#endif
#if TFT_RAW_HEIGHT >= 56
		&buffer[0][TFT_SCRIPT_ROW(56)],
#endif
#if TFT_RAW_HEIGHT >= 57
		&buffer[0][TFT_SCRIPT_ROW(57)],
#endif
#if TFT_RAW_HEIGHT >= 58
		&buffer[0][TFT_SCRIPT_ROW(58)],
#endif
#if TFT_RAW_HEIGHT >= 59
		&buffer[0][TFT_SCRIPT_ROW(59)],
#endif
#if TFT_RAW_HEIGHT >= 60
		&buffer[0][TFT_SCRIPT_ROW(60)],
#endif
#if TFT_RAW_HEIGHT >= 61
		&buffer[0][TFT_SCRIPT_ROW(61)],
#endif
#if TFT_RAW_HEIGHT >= 62
		&buffer[0][TFT_SCRIPT_ROW(62)],
#endif
#if TFT_RAW_HEIGHT >= 63
		&buffer[0][TFT_SCRIPT_ROW(63)],
#endif
#if TFT_RAW_HEIGHT >= 64
		&buffer[0][TFT_SCRIPT_ROW(64)],
#endif
#if TFT_RAW_HEIGHT >= 65
		&buffer[0][TFT_SCRIPT_ROW(65)],
#endif
#if TFT_RAW_HEIGHT >= 66
		&buffer[0][TFT_SCRIPT_ROW(66)],
#endif
#if TFT_RAW_HEIGHT >= 67
		&buffer[0][TFT_SCRIPT_ROW(67)],
#endif
#if TFT_RAW_HEIGHT >= 68
		&buffer[0][TFT_SCRIPT_ROW(68)],
#endif
#if TFT_RAW_HEIGHT >= 69
		&buffer[0][TFT_SCRIPT_ROW(69)],
#endif
#if TFT_RAW_HEIGHT >= 70
		&buffer[0][TFT_SCRIPT_ROW(70)],
#endif
#if TFT_RAW_HEIGHT >= 71
		&buffer[0][TFT_SCRIPT_ROW(71)],
#endif
#if TFT_RAW_HEIGHT >= 72
		&buffer[0][TFT_SCRIPT_ROW(72)],
#endif
#if TFT_RAW_HEIGHT >= 73
		&buffer[0][TFT_SCRIPT_ROW(73)],
#endif
#if TFT_RAW_HEIGHT >= 74
		&buffer[0][TFT_SCRIPT_ROW(74)],
#endif
#if TFT_RAW_HEIGHT >= 75
		&buffer[0][TFT_SCRIPT_ROW(75)],
#endif
#if TFT_RAW_HEIGHT >= 76
		&buffer[0][TFT_SCRIPT_ROW(76)],
#endif
#if TFT_RAW_HEIGHT >= 77
		&buffer[0][TFT_SCRIPT_ROW(77)],
#endif
#if TFT_RAW_HEIGHT >= 78
		&buffer[0][TFT_SCRIPT_ROW(78)],
#endif
#if TFT_RAW_HEIGHT >= 79
		&buffer[0][TFT_SCRIPT_ROW(79)],
#endif
#if TFT_RAW_HEIGHT >= 80
		&buffer[0][TFT_SCRIPT_ROW(80)],
#endif
#if TFT_RAW_HEIGHT >= 81
		&buffer[0][TFT_SCRIPT_ROW(81)],
#endif
#if TFT_RAW_HEIGHT >= 82
		&buffer[0][TFT_SCRIPT_ROW(82)],
#endif
#if TFT_RAW_HEIGHT >= 83
		&buffer[0][TFT_SCRIPT_ROW(83)],
#endif
#if TFT_RAW_HEIGHT >= 84
		&buffer[0][TFT_SCRIPT_ROW(84)],
#endif
#if TFT_RAW_HEIGHT >= 85
		&buffer[0][TFT_SCRIPT_ROW(85)],
#endif
#if TFT_RAW_HEIGHT >= 86
		&buffer[0][TFT_SCRIPT_ROW(86)],
#endif
#if TFT_RAW_HEIGHT >= 87
		&buffer[0][TFT_SCRIPT_ROW(87)],
#endif
#if TFT_RAW_HEIGHT >= 88
		&buffer[0][TFT_SCRIPT_ROW(88)],
#endif
#if TFT_RAW_HEIGHT >= 89
		&buffer[0][TFT_SCRIPT_ROW(89)],
#endif
#if TFT_RAW_HEIGHT >= 90
		&buffer[0][TFT_SCRIPT_ROW(90)],
#endif
#if TFT_RAW_HEIGHT >= 91
		&buffer[0][TFT_SCRIPT_ROW(91)],
#endif
#if TFT_RAW_HEIGHT >= 92
		&buffer[0][TFT_SCRIPT_ROW(92)],
#endif
#if TFT_RAW_HEIGHT >= 93
		&buffer[0][TFT_SCRIPT_ROW(93)],
#endif
#if TFT_RAW_HEIGHT >= 94
		&buffer[0][TFT_SCRIPT_ROW(94)],
#endif
#if TFT_RAW_HEIGHT >= 95
		&buffer[0][TFT_SCRIPT_ROW(95)],
#endif
#if TFT_RAW_HEIGHT >= 96
		&buffer[0][TFT_SCRIPT_ROW(96)],
#endif
#if TFT_RAW_HEIGHT >= 97
		&buffer[0][TFT_SCRIPT_ROW(97)],
#endif
#if TFT_RAW_HEIGHT >= 98
		&buffer[0][TFT_SCRIPT_ROW(98)],
#endif
#if TFT_RAW_HEIGHT >= 99
		&buffer[0][TFT_SCRIPT_ROW(99)],
#endif
#if TFT_RAW_HEIGHT >= 100
		&buffer[0][TFT_SCRIPT_ROW(100)],
#endif
#if TFT_RAW_HEIGHT >= 101
		&buffer[0][TFT_SCRIPT_ROW(101)],
#endif
#if TFT_RAW_HEIGHT >= 102
		&buffer[0][TFT_SCRIPT_ROW(102)],
#endif
#if TFT_RAW_HEIGHT >= 103
		&buffer[0][TFT_SCRIPT_ROW(103)],
#endif
#if TFT_RAW_HEIGHT >= 104
		&buffer[0][TFT_SCRIPT_ROW(104)],
#endif
#if TFT_RAW_HEIGHT >= 105
		&buffer[0][TFT_SCRIPT_ROW(105)],
#endif
#if TFT_RAW_HEIGHT >= 106
		&buffer[0][TFT_SCRIPT_ROW(106)],
#endif
#if TFT_RAW_HEIGHT >= 107
		&buffer[0][TFT_SCRIPT_ROW(107)],
#endif
#if TFT_RAW_HEIGHT >= 108
		&buffer[0][TFT_SCRIPT_ROW(108)],
#endif
#if TFT_RAW_HEIGHT >= 109
		&buffer[0][TFT_SCRIPT_ROW(109)],
#endif
#if TFT_RAW_HEIGHT >= 110
		&buffer[0][TFT_SCRIPT_ROW(110)],
#endif
#if TFT_RAW_HEIGHT >= 111
		&buffer[0][TFT_SCRIPT_ROW(111)],
#endif
#if TFT_RAW_HEIGHT >= 112
		&buffer[0][TFT_SCRIPT_ROW(112)],
#endif
#if TFT_RAW_HEIGHT >= 113
		&buffer[0][TFT_SCRIPT_ROW(113)],
#endif
#if TFT_RAW_HEIGHT >= 114
		&buffer[0][TFT_SCRIPT_ROW(114)],
#endif
#if TFT_RAW_HEIGHT >= 115
		&buffer[0][TFT_SCRIPT_ROW(115)],
#endif
#if TFT_RAW_HEIGHT >= 116
		&buffer[0][TFT_SCRIPT_ROW(116)],
#endif
#if TFT_RAW_HEIGHT >= 117
		&buffer[0][TFT_SCRIPT_ROW(117)],
#endif
#if TFT_RAW_HEIGHT >= 118
		&buffer[0][TFT_SCRIPT_ROW(118)],
#endif
#if TFT_RAW_HEIGHT >= 119
		&buffer[0][TFT_SCRIPT_ROW(119)],
#endif
#if TFT_RAW_HEIGHT >= 120
		&buffer[0][TFT_SCRIPT_ROW(120)],
#endif
#if TFT_RAW_HEIGHT >= 121
		&buffer[0][TFT_SCRIPT_ROW(121)],
#endif
#if TFT_RAW_HEIGHT >= 122
		&buffer[0][TFT_SCRIPT_ROW(122)],
#endif
#if TFT_RAW_HEIGHT >= 123
		&buffer[0][TFT_SCRIPT_ROW(123)],
#endif
#if TFT_RAW_HEIGHT >= 124
		&buffer[0][TFT_SCRIPT_ROW(124)],
#endif
#if TFT_RAW_HEIGHT >= 125
		&buffer[0][TFT_SCRIPT_ROW(125)],
#endif
#if TFT_RAW_HEIGHT >= 126
		&buffer[0][TFT_SCRIPT_ROW(126)],
#endif
#if TFT_RAW_HEIGHT >= 127
		&buffer[0][TFT_SCRIPT_ROW(127)],
#endif
#if TFT_RAW_HEIGHT >= 128
		&buffer[0][TFT_SCRIPT_ROW(128)],
#endif
#if TFT_RAW_HEIGHT >= 129
		&buffer[0][TFT_SCRIPT_ROW(129)],
#endif
#if TFT_RAW_HEIGHT >= 130
		&buffer[0][TFT_SCRIPT_ROW(130)],
#endif
#if TFT_RAW_HEIGHT >= 131
		&buffer[0][TFT_SCRIPT_ROW(131)],
#endif
#if TFT_RAW_HEIGHT >= 132
		&buffer[0][TFT_SCRIPT_ROW(132)],
#endif
#if TFT_RAW_HEIGHT >= 133
		&buffer[0][TFT_SCRIPT_ROW(133)],
#endif
#if TFT_RAW_HEIGHT >= 134
		&buffer[0][TFT_SCRIPT_ROW(134)],
#endif
#if TFT_RAW_HEIGHT >= 135
		&buffer[0][TFT_SCRIPT_ROW(135)],
#endif
#if TFT_RAW_HEIGHT >= 136
		&buffer[0][TFT_SCRIPT_ROW(136)],
#endif
#if TFT_RAW_HEIGHT >= 137
		&buffer[0][TFT_SCRIPT_ROW(137)],
#endif
#if TFT_RAW_HEIGHT >= 138
		&buffer[0][TFT_SCRIPT_ROW(138)],
#endif
#if TFT_RAW_HEIGHT >= 139
		&buffer[0][TFT_SCRIPT_ROW(139)],
#endif
#if TFT_RAW_HEIGHT >= 140
		&buffer[0][TFT_SCRIPT_ROW(140)],
#endif
#if TFT_RAW_HEIGHT >= 141
		&buffer[0][TFT_SCRIPT_ROW(141)],
#endif
#if TFT_RAW_HEIGHT >= 142
		&buffer[0][TFT_SCRIPT_ROW(142)],
#endif
#if TFT_RAW_HEIGHT >= 143
		&buffer[0][TFT_SCRIPT_ROW(143)],
#endif
#if TFT_RAW_HEIGHT >= 144
		&buffer[0][TFT_SCRIPT_ROW(144)],
#endif
#if TFT_RAW_HEIGHT >= 145
		&buffer[0][TFT_SCRIPT_ROW(145)],
#endif
#if TFT_RAW_HEIGHT >= 146
		&buffer[0][TFT_SCRIPT_ROW(146)],
#endif
#if TFT_RAW_HEIGHT >= 147
		&buffer[0][TFT_SCRIPT_ROW(147)],
#endif
#if TFT_RAW_HEIGHT >= 148
		&buffer[0][TFT_SCRIPT_ROW(148)],
#endif
#if TFT_RAW_HEIGHT >= 149
		&buffer[0][TFT_SCRIPT_ROW(149)],
#endif
#if TFT_RAW_HEIGHT >= 150
		&buffer[0][TFT_SCRIPT_ROW(150)],
#endif
#if TFT_RAW_HEIGHT >= 151
		&buffer[0][TFT_SCRIPT_ROW(151)],
#endif
#if TFT_RAW_HEIGHT >= 152
		&buffer[0][TFT_SCRIPT_ROW(152)],
#endif
#if TFT_RAW_HEIGHT >= 153
		&buffer[0][TFT_SCRIPT_ROW(153)],
#endif
#if TFT_RAW_HEIGHT >= 154
		&buffer[0][TFT_SCRIPT_ROW(154)],
#endif
#if TFT_RAW_HEIGHT >= 155
		&buffer[0][TFT_SCRIPT_ROW(155)],
#endif
#if TFT_RAW_HEIGHT >= 156
		&buffer[0][TFT_SCRIPT_ROW(156)],
#endif
#if TFT_RAW_HEIGHT >= 157
		&buffer[0][TFT_SCRIPT_ROW(157)],
#endif
#if TFT_RAW_HEIGHT >= 158
		&buffer[0][TFT_SCRIPT_ROW(158)],
#endif
#if TFT_RAW_HEIGHT >= 159
		&buffer[0][TFT_SCRIPT_ROW(159)],
#endif
#if TFT_RAW_HEIGHT >= 160
		&buffer[0][TFT_SCRIPT_ROW(160)],
#endif
#if TFT_RAW_HEIGHT >= 161
		&buffer[0][TFT_SCRIPT_ROW(161)],
#endif
#if TFT_RAW_HEIGHT >= 162
		&buffer[0][TFT_SCRIPT_ROW(162)],
#endif
#if TFT_RAW_HEIGHT >= 163
		&buffer[0][TFT_SCRIPT_ROW(163)],
#endif
#if TFT_RAW_HEIGHT >= 164
		&buffer[0][TFT_SCRIPT_ROW(164)],
#endif
#if TFT_RAW_HEIGHT >= 165
		&buffer[0][TFT_SCRIPT_ROW(165)],
#endif
#if TFT_RAW_HEIGHT >= 166
		&buffer[0][TFT_SCRIPT_ROW(166)],
#endif
#if TFT_RAW_HEIGHT >= 167
		&buffer[0][TFT_SCRIPT_ROW(167)],
#endif
#if TFT_RAW_HEIGHT >= 168
		&buffer[0][TFT_SCRIPT_ROW(168)],
#endif
#if TFT_RAW_HEIGHT >= 169
		&buffer[0][TFT_SCRIPT_ROW(169)],
#endif
#if TFT_RAW_HEIGHT >= 170
		&buffer[0][TFT_SCRIPT_ROW(170)],
#endif
#if TFT_RAW_HEIGHT >= 171
		&buffer[0][TFT_SCRIPT_ROW(171)],
#endif
#if TFT_RAW_HEIGHT >= 172
		&buffer[0][TFT_SCRIPT_ROW(172)],
#endif
#if TFT_RAW_HEIGHT >= 173
		&buffer[0][TFT_SCRIPT_ROW(173)],
#endif
#if TFT_RAW_HEIGHT >= 174
		&buffer[0][TFT_SCRIPT_ROW(174)],
#endif
#if TFT_RAW_HEIGHT >= 175
		&buffer[0][TFT_SCRIPT_ROW(175)],
#endif
#if TFT_RAW_HEIGHT >= 176
		&buffer[0][TFT_SCRIPT_ROW(176)],
#endif
#if TFT_RAW_HEIGHT >= 177
		&buffer[0][TFT_SCRIPT_ROW(177)],
#endif
#if TFT_RAW_HEIGHT >= 178
		&buffer[0][TFT_SCRIPT_ROW(178)],
#endif
#if TFT_RAW_HEIGHT >= 179
		&buffer[0][TFT_SCRIPT_ROW(179)],
#endif
#if TFT_RAW_HEIGHT >= 180
		&buffer[0][TFT_SCRIPT_ROW(180)],
#endif
#if TFT_RAW_HEIGHT >= 181
		&buffer[0][TFT_SCRIPT_ROW(181)],
#endif
#if TFT_RAW_HEIGHT >= 182
		&buffer[0][TFT_SCRIPT_ROW(182)],
#endif
#if TFT_RAW_HEIGHT >= 183
		&buffer[0][TFT_SCRIPT_ROW(183)],
#endif
#if TFT_RAW_HEIGHT >= 184
		&buffer[0][TFT_SCRIPT_ROW(184)],
#endif
#if TFT_RAW_HEIGHT >= 185
		&buffer[0][TFT_SCRIPT_ROW(185)],
#endif
#if TFT_RAW_HEIGHT >= 186
		&buffer[0][TFT_SCRIPT_ROW(186)],
#endif
#if TFT_RAW_HEIGHT >= 187
		&buffer[0][TFT_SCRIPT_ROW(187)],
#endif
#if TFT_RAW_HEIGHT >= 188
		&buffer[0][TFT_SCRIPT_ROW(188)],
#endif
#if TFT_RAW_HEIGHT >= 189
		&buffer[0][TFT_SCRIPT_ROW(189)],
#endif
#if TFT_RAW_HEIGHT >= 190
		&buffer[0][TFT_SCRIPT_ROW(190)],
#endif
#if TFT_RAW_HEIGHT >= 191
		&buffer[0][TFT_SCRIPT_ROW(191)],
#endif
#if TFT_RAW_HEIGHT >= 192
		&buffer[0][TFT_SCRIPT_ROW(192)],
#endif
#if TFT_RAW_HEIGHT >= 193
		&buffer[0][TFT_SCRIPT_ROW(193)],
#endif
#if TFT_RAW_HEIGHT >= 194
		&buffer[0][TFT_SCRIPT_ROW(194)],
#endif
#if TFT_RAW_HEIGHT >= 195
		&buffer[0][TFT_SCRIPT_ROW(195)],
#endif
#if TFT_RAW_HEIGHT >= 196
		&buffer[0][TFT_SCRIPT_ROW(196)],
#endif
#if TFT_RAW_HEIGHT >= 197
		&buffer[0][TFT_SCRIPT_ROW(197)],
#endif
#if TFT_RAW_HEIGHT >= 198
		&buffer[0][TFT_SCRIPT_ROW(198)],
#endif
#if TFT_RAW_HEIGHT >= 199
		&buffer[0][TFT_SCRIPT_ROW(199)],
#endif
#if TFT_RAW_HEIGHT >= 200
		&buffer[0][TFT_SCRIPT_ROW(200)],
#endif
#if TFT_RAW_HEIGHT >= 201
		&buffer[0][TFT_SCRIPT_ROW(201)],
#endif
#if TFT_RAW_HEIGHT >= 202
		&buffer[0][TFT_SCRIPT_ROW(202)],
#endif
#if TFT_RAW_HEIGHT >= 203
		&buffer[0][TFT_SCRIPT_ROW(203)],
#endif
#if TFT_RAW_HEIGHT >= 204
		&buffer[0][TFT_SCRIPT_ROW(204)],
#endif
#if TFT_RAW_HEIGHT >= 205
		&buffer[0][TFT_SCRIPT_ROW(205)],
#endif
#if TFT_RAW_HEIGHT >= 206
		&buffer[0][TFT_SCRIPT_ROW(206)],
#endif
#if TFT_RAW_HEIGHT >= 207
		&buffer[0][TFT_SCRIPT_ROW(207)],
#endif
#if TFT_RAW_HEIGHT >= 208
		&buffer[0][TFT_SCRIPT_ROW(208)],
#endif
#if TFT_RAW_HEIGHT >= 209
		&buffer[0][TFT_SCRIPT_ROW(209)],
#endif
#if TFT_RAW_HEIGHT >= 210
		&buffer[0][TFT_SCRIPT_ROW(210)],
#endif
#if TFT_RAW_HEIGHT >= 211
		&buffer[0][TFT_SCRIPT_ROW(211)],
#endif
#if TFT_RAW_HEIGHT >= 212
		&buffer[0][TFT_SCRIPT_ROW(212)],
#endif
#if TFT_RAW_HEIGHT >= 213
		&buffer[0][TFT_SCRIPT_ROW(213)],
#endif
#if TFT_RAW_HEIGHT >= 214
		&buffer[0][TFT_SCRIPT_ROW(214)],
#endif
#if TFT_RAW_HEIGHT >= 215
		&buffer[0][TFT_SCRIPT_ROW(215)],
#endif
#if TFT_RAW_HEIGHT >= 216
		&buffer[0][TFT_SCRIPT_ROW(216)],
#endif
#if TFT_RAW_HEIGHT >= 217
		&buffer[0][TFT_SCRIPT_ROW(217)],
#endif
#if TFT_RAW_HEIGHT >= 218
		&buffer[0][TFT_SCRIPT_ROW(218)],
#endif
#if TFT_RAW_HEIGHT >= 219
		&buffer[0][TFT_SCRIPT_ROW(219)],
#endif
#if TFT_RAW_HEIGHT >= 220
		&buffer[0][TFT_SCRIPT_ROW(220)],
#endif
#if TFT_RAW_HEIGHT >= 221
		&buffer[0][TFT_SCRIPT_ROW(221)],
#endif
#if TFT_RAW_HEIGHT >= 222
		&buffer[0][TFT_SCRIPT_ROW(222)],
#endif
#if TFT_RAW_HEIGHT >= 223
		&buffer[0][TFT_SCRIPT_ROW(223)],
#endif
#if TFT_RAW_HEIGHT >= 224
		&buffer[0][TFT_SCRIPT_ROW(224)],
#endif
#if TFT_RAW_HEIGHT >= 225
		&buffer[0][TFT_SCRIPT_ROW(225)],
#endif
#if TFT_RAW_HEIGHT >= 226
		&buffer[0][TFT_SCRIPT_ROW(226)],
#endif
#if TFT_RAW_HEIGHT >= 227
		&buffer[0][TFT_SCRIPT_ROW(227)],
#endif
#if TFT_RAW_HEIGHT >= 228
		&buffer[0][TFT_SCRIPT_ROW(228)],
#endif
#if TFT_RAW_HEIGHT >= 229
		&buffer[0][TFT_SCRIPT_ROW(229)],
#endif
#if TFT_RAW_HEIGHT >= 230
		&buffer[0][TFT_SCRIPT_ROW(230)],
#endif
#if TFT_RAW_HEIGHT >= 231
		&buffer[0][TFT_SCRIPT_ROW(231)],
#endif
#if TFT_RAW_HEIGHT >= 232
		&buffer[0][TFT_SCRIPT_ROW(232)],
#endif
#if TFT_RAW_HEIGHT >= 233
		&buffer[0][TFT_SCRIPT_ROW(233)],
#endif
#if TFT_RAW_HEIGHT >= 234
		&buffer[0][TFT_SCRIPT_ROW(234)],
#endif
#if TFT_RAW_HEIGHT >= 235
		&buffer[0][TFT_SCRIPT_ROW(235)],
#endif
#if TFT_RAW_HEIGHT >= 236
		&buffer[0][TFT_SCRIPT_ROW(236)],
#endif
#if TFT_RAW_HEIGHT >= 237
		&buffer[0][TFT_SCRIPT_ROW(237)],
#endif
#if TFT_RAW_HEIGHT >= 238
		&buffer[0][TFT_SCRIPT_ROW(238)],
#endif
#if TFT_RAW_HEIGHT >= 239
		&buffer[0][TFT_SCRIPT_ROW(239)],
#endif
#if TFT_RAW_HEIGHT >= 240
		&buffer[0][TFT_SCRIPT_ROW(240)],
#endif
#if TFT_RAW_HEIGHT >= 241
		&buffer[0][TFT_SCRIPT_ROW(241)],
#endif
#if TFT_RAW_HEIGHT >= 242
		&buffer[0][TFT_SCRIPT_ROW(242)],
#endif
#if TFT_RAW_HEIGHT >= 243
		&buffer[0][TFT_SCRIPT_ROW(243)],
#endif
#if TFT_RAW_HEIGHT >= 244
		&buffer[0][TFT_SCRIPT_ROW(244)],
#endif
#if TFT_RAW_HEIGHT >= 245
		&buffer[0][TFT_SCRIPT_ROW(245)],
#endif
#if TFT_RAW_HEIGHT >= 246
		&buffer[0][TFT_SCRIPT_ROW(246)],
#endif
#if TFT_RAW_HEIGHT >= 247
		&buffer[0][TFT_SCRIPT_ROW(247)],
#endif
#if TFT_RAW_HEIGHT >= 248
		&buffer[0][TFT_SCRIPT_ROW(248)],
#endif
#if TFT_RAW_HEIGHT >= 249
		&buffer[0][TFT_SCRIPT_ROW(249)],
#endif
#if TFT_RAW_HEIGHT >= 250
		&buffer[0][TFT_SCRIPT_ROW(250)],
#endif
#if TFT_RAW_HEIGHT >= 251
		&buffer[0][TFT_SCRIPT_ROW(251)],
#endif
#if TFT_RAW_HEIGHT >= 252
		&buffer[0][TFT_SCRIPT_ROW(252)],
#endif
#if TFT_RAW_HEIGHT >= 253
		&buffer[0][TFT_SCRIPT_ROW(253)],
#endif
#if TFT_RAW_HEIGHT >= 254
		&buffer[0][TFT_SCRIPT_ROW(254)],
#endif
#if TFT_RAW_HEIGHT >= 255
		&buffer[0][TFT_SCRIPT_ROW(255)],
#endif
#if TFT_RAW_HEIGHT >= 256
		&buffer[0][TFT_SCRIPT_ROW(256)],
#endif
#if TFT_RAW_HEIGHT >= 257
		&buffer[0][TFT_SCRIPT_ROW(257)],
#endif
#if TFT_RAW_HEIGHT >= 258
		&buffer[0][TFT_SCRIPT_ROW(258)],
#endif
#if TFT_RAW_HEIGHT >= 259
		&buffer[0][TFT_SCRIPT_ROW(259)],
#endif
#if TFT_RAW_HEIGHT >= 260
		&buffer[0][TFT_SCRIPT_ROW(260)],
#endif
#if TFT_RAW_HEIGHT >= 261
		&buffer[0][TFT_SCRIPT_ROW(261)],
#endif
#if TFT_RAW_HEIGHT >= 262
		&buffer[0][TFT_SCRIPT_ROW(262)],
#endif
#if TFT_RAW_HEIGHT >= 263
		&buffer[0][TFT_SCRIPT_ROW(263)],
#endif
#if TFT_RAW_HEIGHT >= 264
		&buffer[0][TFT_SCRIPT_ROW(264)],
#endif
#if TFT_RAW_HEIGHT >= 265
		&buffer[0][TFT_SCRIPT_ROW(265)],
#endif
#if TFT_RAW_HEIGHT >= 266
		&buffer[0][TFT_SCRIPT_ROW(266)],
#endif
#if TFT_RAW_HEIGHT >= 267
		&buffer[0][TFT_SCRIPT_ROW(267)],
#endif
#if TFT_RAW_HEIGHT >= 268
		&buffer[0][TFT_SCRIPT_ROW(268)],
#endif
#if TFT_RAW_HEIGHT >= 269
		&buffer[0][TFT_SCRIPT_ROW(269)],
#endif
#if TFT_RAW_HEIGHT >= 270
		&buffer[0][TFT_SCRIPT_ROW(270)],
#endif
#if TFT_RAW_HEIGHT >= 271
		&buffer[0][TFT_SCRIPT_ROW(271)],
#endif
#if TFT_RAW_HEIGHT >= 272
		&buffer[0][TFT_SCRIPT_ROW(272)],
#endif
#if TFT_RAW_HEIGHT >= 273
		&buffer[0][TFT_SCRIPT_ROW(273)],
#endif
#if TFT_RAW_HEIGHT >= 274
		&buffer[0][TFT_SCRIPT_ROW(274)],
#endif
#if TFT_RAW_HEIGHT >= 275
		&buffer[0][TFT_SCRIPT_ROW(275)],
#endif
#if TFT_RAW_HEIGHT >= 276
		&buffer[0][TFT_SCRIPT_ROW(276)],
#endif
#if TFT_RAW_HEIGHT >= 277
		&buffer[0][TFT_SCRIPT_ROW(277)],
#endif
#if TFT_RAW_HEIGHT >= 278
		&buffer[0][TFT_SCRIPT_ROW(278)],
#endif
#if TFT_RAW_HEIGHT >= 279
		&buffer[0][TFT_SCRIPT_ROW(279)],
#endif
#if TFT_RAW_HEIGHT >= 280
		&buffer[0][TFT_SCRIPT_ROW(280)],
#endif
#if TFT_RAW_HEIGHT >= 281
		&buffer[0][TFT_SCRIPT_ROW(281)],
#endif
#if TFT_RAW_HEIGHT >= 282
		&buffer[0][TFT_SCRIPT_ROW(282)],
#endif
#if TFT_RAW_HEIGHT >= 283
		&buffer[0][TFT_SCRIPT_ROW(283)],
#endif
#if TFT_RAW_HEIGHT >= 284
		&buffer[0][TFT_SCRIPT_ROW(284)],
#endif
#if TFT_RAW_HEIGHT >= 285
		&buffer[0][TFT_SCRIPT_ROW(285)],
#endif
#if TFT_RAW_HEIGHT >= 286
		&buffer[0][TFT_SCRIPT_ROW(286)],
#endif
#if TFT_RAW_HEIGHT >= 287
		&buffer[0][TFT_SCRIPT_ROW(287)],
#endif
#if TFT_RAW_HEIGHT >= 288
		&buffer[0][TFT_SCRIPT_ROW(288)],
#endif
#if TFT_RAW_HEIGHT >= 289
		&buffer[0][TFT_SCRIPT_ROW(289)],
#endif
#if TFT_RAW_HEIGHT >= 290
		&buffer[0][TFT_SCRIPT_ROW(290)],
#endif
#if TFT_RAW_HEIGHT >= 291
		&buffer[0][TFT_SCRIPT_ROW(291)],
#endif
#if TFT_RAW_HEIGHT >= 292
		&buffer[0][TFT_SCRIPT_ROW(292)],
#endif
#if TFT_RAW_HEIGHT >= 293
		&buffer[0][TFT_SCRIPT_ROW(293)],
#endif
#if TFT_RAW_HEIGHT >= 294
		&buffer[0][TFT_SCRIPT_ROW(294)],
#endif
#if TFT_RAW_HEIGHT >= 295
		&buffer[0][TFT_SCRIPT_ROW(295)],
#endif
#if TFT_RAW_HEIGHT >= 296
		&buffer[0][TFT_SCRIPT_ROW(296)],
#endif
#if TFT_RAW_HEIGHT >= 297
		&buffer[0][TFT_SCRIPT_ROW(297)],
#endif
#if TFT_RAW_HEIGHT >= 298
		&buffer[0][TFT_SCRIPT_ROW(298)],
#endif
#if TFT_RAW_HEIGHT >= 299
		&buffer[0][TFT_SCRIPT_ROW(299)],
#endif
#if TFT_RAW_HEIGHT >= 300
		&buffer[0][TFT_SCRIPT_ROW(300)],
#endif
#if TFT_RAW_HEIGHT >= 301
		&buffer[0][TFT_SCRIPT_ROW(301)],
#endif
#if TFT_RAW_HEIGHT >= 302
		&buffer[0][TFT_SCRIPT_ROW(302)],
#endif
#if TFT_RAW_HEIGHT >= 303
		&buffer[0][TFT_SCRIPT_ROW(303)],
#endif
#if TFT_RAW_HEIGHT >= 304
		&buffer[0][TFT_SCRIPT_ROW(304)],
#endif
#if TFT_RAW_HEIGHT >= 305
		&buffer[0][TFT_SCRIPT_ROW(305)],
#endif
#if TFT_RAW_HEIGHT >= 306
		&buffer[0][TFT_SCRIPT_ROW(306)],
#endif
#if TFT_RAW_HEIGHT >= 307
		&buffer[0][TFT_SCRIPT_ROW(307)],
#endif
#if TFT_RAW_HEIGHT >= 308
		&buffer[0][TFT_SCRIPT_ROW(308)],
#endif
#if TFT_RAW_HEIGHT >= 309
		&buffer[0][TFT_SCRIPT_ROW(309)],
#endif
#if TFT_RAW_HEIGHT >= 310
		&buffer[0][TFT_SCRIPT_ROW(310)],
#endif
#if TFT_RAW_HEIGHT >= 311
		&buffer[0][TFT_SCRIPT_ROW(311)],
#endif
#if TFT_RAW_HEIGHT >= 312
		&buffer[0][TFT_SCRIPT_ROW(312)],
#endif
#if TFT_RAW_HEIGHT >= 313
		&buffer[0][TFT_SCRIPT_ROW(313)],
#endif
#if TFT_RAW_HEIGHT >= 314
		&buffer[0][TFT_SCRIPT_ROW(314)],
#endif
#if TFT_RAW_HEIGHT >= 315
		&buffer[0][TFT_SCRIPT_ROW(315)],
#endif
#if TFT_RAW_HEIGHT >= 316
		&buffer[0][TFT_SCRIPT_ROW(316)],
#endif
#if TFT_RAW_HEIGHT >= 317
		&buffer[0][TFT_SCRIPT_ROW(317)],
#endif
#if TFT_RAW_HEIGHT >= 318
		&buffer[0][TFT_SCRIPT_ROW(318)],
#endif
#if TFT_RAW_HEIGHT >= 319
		&buffer[0][TFT_SCRIPT_ROW(319)],
#endif
		NULL,
	},
	{
#if TFT_RAW_HEIGHT >= 0
		&buffer[1][TFT_SCRIPT_ROW(0)],
#endif
#if TFT_RAW_HEIGHT >= 1
		&buffer[1][TFT_SCRIPT_ROW(1)],
#endif
#if TFT_RAW_HEIGHT >= 2
		&buffer[1][TFT_SCRIPT_ROW(2)],
#endif
#if TFT_RAW_HEIGHT >= 3
		&buffer[1][TFT_SCRIPT_ROW(3)],
#endif
#if TFT_RAW_HEIGHT >= 4
		&buffer[1][TFT_SCRIPT_ROW(4)],
#endif
#if TFT_RAW_HEIGHT >= 5
		&buffer[1][TFT_SCRIPT_ROW(5)],
#endif
#if TFT_RAW_HEIGHT >= 6
		&buffer[1][TFT_SCRIPT_ROW(6)],
#endif
#if TFT_RAW_HEIGHT >= 7
		&buffer[1][TFT_SCRIPT_ROW(7)],
#endif
#if TFT_RAW_HEIGHT >= 8
		&buffer[1][TFT_SCRIPT_ROW(8)],
#endif
#if TFT_RAW_HEIGHT >= 9
		&buffer[1][TFT_SCRIPT_ROW(9)],
#endif
#if TFT_RAW_HEIGHT >= 10
		&buffer[1][TFT_SCRIPT_ROW(10)],
#endif
#if TFT_RAW_HEIGHT >= 11
		&buffer[1][TFT_SCRIPT_ROW(11)],
#endif
#if TFT_RAW_HEIGHT >= 12
		&buffer[1][TFT_SCRIPT_ROW(12)],
#endif
#if TFT_RAW_HEIGHT >= 13
		&buffer[1][TFT_SCRIPT_ROW(13)],
#endif
#if TFT_RAW_HEIGHT >= 14
		&buffer[1][TFT_SCRIPT_ROW(14)],
#endif
#if TFT_RAW_HEIGHT >= 15
		&buffer[1][TFT_SCRIPT_ROW(15)],
#endif
#if TFT_RAW_HEIGHT >= 16
		&buffer[1][TFT_SCRIPT_ROW(16)],
#endif
#if TFT_RAW_HEIGHT >= 17
		&buffer[1][TFT_SCRIPT_ROW(17)],
#endif
#if TFT_RAW_HEIGHT >= 18
		&buffer[1][TFT_SCRIPT_ROW(18)],
#endif
#if TFT_RAW_HEIGHT >= 19
		&buffer[1][TFT_SCRIPT_ROW(19)],
#endif
#if TFT_RAW_HEIGHT >= 20
		&buffer[1][TFT_SCRIPT_ROW(20)],
#endif
#if TFT_RAW_HEIGHT >= 21
		&buffer[1][TFT_SCRIPT_ROW(21)],
#endif
#if TFT_RAW_HEIGHT >= 22
		&buffer[1][TFT_SCRIPT_ROW(22)],
#endif
#if TFT_RAW_HEIGHT >= 23
		&buffer[1][TFT_SCRIPT_ROW(23)],
#endif
#if TFT_RAW_HEIGHT >= 24
		&buffer[1][TFT_SCRIPT_ROW(24)],
#endif
#if TFT_RAW_HEIGHT >= 25
		&buffer[1][TFT_SCRIPT_ROW(25)],
#endif
#if TFT_RAW_HEIGHT >= 26
		&buffer[1][TFT_SCRIPT_ROW(26)],
#endif
#if TFT_RAW_HEIGHT >= 27
		&buffer[1][TFT_SCRIPT_ROW(27)],
#endif
#if TFT_RAW_HEIGHT >= 28
		&buffer[1][TFT_SCRIPT_ROW(28)],
#endif
#if TFT_RAW_HEIGHT >= 29
		&buffer[1][TFT_SCRIPT_ROW(29)],
#endif
#if TFT_RAW_HEIGHT >= 30
		&buffer[1][TFT_SCRIPT_ROW(30)],
#endif
#if TFT_RAW_HEIGHT >= 31
		&buffer[1][TFT_SCRIPT_ROW(31)],
#endif
#if TFT_RAW_HEIGHT >= 32
		&buffer[1][TFT_SCRIPT_ROW(32)],
#endif
#if TFT_RAW_HEIGHT >= 33
		&buffer[1][TFT_SCRIPT_ROW(33)],
#endif
#if TFT_RAW_HEIGHT >= 34
		&buffer[1][TFT_SCRIPT_ROW(34)],
#endif
#if TFT_RAW_HEIGHT >= 35
		&buffer[1][TFT_SCRIPT_ROW(35)],
#endif
#if TFT_RAW_HEIGHT >= 36
		&buffer[1][TFT_SCRIPT_ROW(36)],
#endif
#if TFT_RAW_HEIGHT >= 37
		&buffer[1][TFT_SCRIPT_ROW(37)],
#endif
#if TFT_RAW_HEIGHT >= 38
		&buffer[1][TFT_SCRIPT_ROW(38)],
#endif
#if TFT_RAW_HEIGHT >= 39
		&buffer[1][TFT_SCRIPT_ROW(39)],
#endif
#if TFT_RAW_HEIGHT >= 40
		&buffer[1][TFT_SCRIPT_ROW(40)],
#endif
#if TFT_RAW_HEIGHT >= 41
		&buffer[1][TFT_SCRIPT_ROW(41)],
#endif
#if TFT_RAW_HEIGHT >= 42
		&buffer[1][TFT_SCRIPT_ROW(42)],
#endif
#if TFT_RAW_HEIGHT >= 43
		&buffer[1][TFT_SCRIPT_ROW(43)],
#endif
#if TFT_RAW_HEIGHT >= 44
		&buffer[1][TFT_SCRIPT_ROW(44)],
#endif
#if TFT_RAW_HEIGHT >= 45
		&buffer[1][TFT_SCRIPT_ROW(45)],
#endif
#if TFT_RAW_HEIGHT >= 46
		&buffer[1][TFT_SCRIPT_ROW(46)],
#endif
#if TFT_RAW_HEIGHT >= 47
		&buffer[1][TFT_SCRIPT_ROW(47)],
#endif
#if TFT_RAW_HEIGHT >= 48
		&buffer[1][TFT_SCRIPT_ROW(48)],
#endif
#if TFT_RAW_HEIGHT >= 49
		&buffer[1][TFT_SCRIPT_ROW(49)],
#endif
#if TFT_RAW_HEIGHT >= 50
		&buffer[1][TFT_SCRIPT_ROW(50)],
#endif
#if TFT_RAW_HEIGHT >= 51
		&buffer[1][TFT_SCRIPT_ROW(51)],
#endif
#if TFT_RAW_HEIGHT >= 52
		&buffer[1][TFT_SCRIPT_ROW(52)],
#endif
#if TFT_RAW_HEIGHT >= 53
		&buffer[1][TFT_SCRIPT_ROW(53)],
#endif
#if TFT_RAW_HEIGHT >= 54
		&buffer[1][TFT_SCRIPT_ROW(54)],
#endif
#if TFT_RAW_HEIGHT >= 55
		&buffer[1][TFT_SCRIPT_ROW(55)],
#endif
#if TFT_RAW_HEIGHT >= 56
		&buffer[1][TFT_SCRIPT_ROW(56)],
#endif
#if TFT_RAW_HEIGHT >= 57
		&buffer[1][TFT_SCRIPT_ROW(57)],
#endif
#if TFT_RAW_HEIGHT >= 58
		&buffer[1][TFT_SCRIPT_ROW(58)],
#endif
#if TFT_RAW_HEIGHT >= 59
		&buffer[1][TFT_SCRIPT_ROW(59)],
#endif
#if TFT_RAW_HEIGHT >= 60
		&buffer[1][TFT_SCRIPT_ROW(60)],
#endif
#if TFT_RAW_HEIGHT >= 61
		&buffer[1][TFT_SCRIPT_ROW(61)],
#endif
#if TFT_RAW_HEIGHT >= 62
		&buffer[1][TFT_SCRIPT_ROW(62)],
#endif
#if TFT_RAW_HEIGHT >= 63
		&buffer[1][TFT_SCRIPT_ROW(63)],
#endif
#if TFT_RAW_HEIGHT >= 64
		&buffer[1][TFT_SCRIPT_ROW(64)],
#endif
#if TFT_RAW_HEIGHT >= 65
		&buffer[1][TFT_SCRIPT_ROW(65)],
#endif
#if TFT_RAW_HEIGHT >= 66
		&buffer[1][TFT_SCRIPT_ROW(66)],
#endif
#if TFT_RAW_HEIGHT >= 67
		&buffer[1][TFT_SCRIPT_ROW(67)],
#endif
#if TFT_RAW_HEIGHT >= 68
		&buffer[1][TFT_SCRIPT_ROW(68)],
#endif
#if TFT_RAW_HEIGHT >= 69
		&buffer[1][TFT_SCRIPT_ROW(69)],
#endif
#if TFT_RAW_HEIGHT >= 70
		&buffer[1][TFT_SCRIPT_ROW(70)],
#endif
#if TFT_RAW_HEIGHT >= 71
		&buffer[1][TFT_SCRIPT_ROW(71)],
#endif
#if TFT_RAW_HEIGHT >= 72
		&buffer[1][TFT_SCRIPT_ROW(72)],
#endif
#if TFT_RAW_HEIGHT >= 73
		&buffer[1][TFT_SCRIPT_ROW(73)],
#endif
#if TFT_RAW_HEIGHT >= 74
		&buffer[1][TFT_SCRIPT_ROW(74)],
#endif
#if TFT_RAW_HEIGHT >= 75
		&buffer[1][TFT_SCRIPT_ROW(75)],
#endif
#if TFT_RAW_HEIGHT >= 76
		&buffer[1][TFT_SCRIPT_ROW(76)],
#endif
#if TFT_RAW_HEIGHT >= 77
		&buffer[1][TFT_SCRIPT_ROW(77)],
#endif
#if TFT_RAW_HEIGHT >= 78
		&buffer[1][TFT_SCRIPT_ROW(78)],
#endif
#if TFT_RAW_HEIGHT >= 79
		&buffer[1][TFT_SCRIPT_ROW(79)],
#endif
#if TFT_RAW_HEIGHT >= 80
		&buffer[1][TFT_SCRIPT_ROW(80)],
#endif
#if TFT_RAW_HEIGHT >= 81
		&buffer[1][TFT_SCRIPT_ROW(81)],
#endif
#if TFT_RAW_HEIGHT >= 82
		&buffer[1][TFT_SCRIPT_ROW(82)],
#endif
#if TFT_RAW_HEIGHT >= 83
		&buffer[1][TFT_SCRIPT_ROW(83)],
#endif
#if TFT_RAW_HEIGHT >= 84
		&buffer[1][TFT_SCRIPT_ROW(84)],
#endif
#if TFT_RAW_HEIGHT >= 85
		&buffer[1][TFT_SCRIPT_ROW(85)],
#endif
#if TFT_RAW_HEIGHT >= 86
		&buffer[1][TFT_SCRIPT_ROW(86)],
#endif
#if TFT_RAW_HEIGHT >= 87
		&buffer[1][TFT_SCRIPT_ROW(87)],
#endif
#if TFT_RAW_HEIGHT >= 88
		&buffer[1][TFT_SCRIPT_ROW(88)],
#endif
#if TFT_RAW_HEIGHT >= 89
		&buffer[1][TFT_SCRIPT_ROW(89)],
#endif
#if TFT_RAW_HEIGHT >= 90
		&buffer[1][TFT_SCRIPT_ROW(90)],
#endif
#if TFT_RAW_HEIGHT >= 91
		&buffer[1][TFT_SCRIPT_ROW(91)],
#endif
#if TFT_RAW_HEIGHT >= 92
		&buffer[1][TFT_SCRIPT_ROW(92)],
#endif
#if TFT_RAW_HEIGHT >= 93
		&buffer[1][TFT_SCRIPT_ROW(93)],
#endif
#if TFT_RAW_HEIGHT >= 94
		&buffer[1][TFT_SCRIPT_ROW(94)],
#endif
#if TFT_RAW_HEIGHT >= 95
		&buffer[1][TFT_SCRIPT_ROW(95)],
#endif
#if TFT_RAW_HEIGHT >= 96
		&buffer[1][TFT_SCRIPT_ROW(96)],
#endif
#if TFT_RAW_HEIGHT >= 97
		&buffer[1][TFT_SCRIPT_ROW(97)],
#endif
#if TFT_RAW_HEIGHT >= 98
		&buffer[1][TFT_SCRIPT_ROW(98)],
#endif
#if TFT_RAW_HEIGHT >= 99
		&buffer[1][TFT_SCRIPT_ROW(99)],
#endif
#if TFT_RAW_HEIGHT >= 100
		&buffer[1][TFT_SCRIPT_ROW(100)],
#endif
#if TFT_RAW_HEIGHT >= 101
		&buffer[1][TFT_SCRIPT_ROW(101)],
#endif
#if TFT_RAW_HEIGHT >= 102
		&buffer[1][TFT_SCRIPT_ROW(102)],
#endif
#if TFT_RAW_HEIGHT >= 103
		&buffer[1][TFT_SCRIPT_ROW(103)],
#endif
#if TFT_RAW_HEIGHT >= 104
		&buffer[1][TFT_SCRIPT_ROW(104)],
#endif
#if TFT_RAW_HEIGHT >= 105
		&buffer[1][TFT_SCRIPT_ROW(105)],
#endif
#if TFT_RAW_HEIGHT >= 106
		&buffer[1][TFT_SCRIPT_ROW(106)],
#endif
#if TFT_RAW_HEIGHT >= 107
		&buffer[1][TFT_SCRIPT_ROW(107)],
#endif
#if TFT_RAW_HEIGHT >= 108
		&buffer[1][TFT_SCRIPT_ROW(108)],
#endif
#if TFT_RAW_HEIGHT >= 109
		&buffer[1][TFT_SCRIPT_ROW(109)],
#endif
#if TFT_RAW_HEIGHT >= 110
		&buffer[1][TFT_SCRIPT_ROW(110)],
#endif
#if TFT_RAW_HEIGHT >= 111
		&buffer[1][TFT_SCRIPT_ROW(111)],
#endif
#if TFT_RAW_HEIGHT >= 112
		&buffer[1][TFT_SCRIPT_ROW(112)],
#endif
#if TFT_RAW_HEIGHT >= 113
		&buffer[1][TFT_SCRIPT_ROW(113)],
#endif
#if TFT_RAW_HEIGHT >= 114
		&buffer[1][TFT_SCRIPT_ROW(114)],
#endif
#if TFT_RAW_HEIGHT >= 115
		&buffer[1][TFT_SCRIPT_ROW(115)],
#endif
#if TFT_RAW_HEIGHT >= 116
		&buffer[1][TFT_SCRIPT_ROW(116)],
#endif
#if TFT_RAW_HEIGHT >= 117
		&buffer[1][TFT_SCRIPT_ROW(117)],
#endif
#if TFT_RAW_HEIGHT >= 118
		&buffer[1][TFT_SCRIPT_ROW(118)],
#endif
#if TFT_RAW_HEIGHT >= 119
		&buffer[1][TFT_SCRIPT_ROW(119)],
#endif
#if TFT_RAW_HEIGHT >= 120
		&buffer[1][TFT_SCRIPT_ROW(120)],
#endif
#if TFT_RAW_HEIGHT >= 121
		&buffer[1][TFT_SCRIPT_ROW(121)],
#endif
#if TFT_RAW_HEIGHT >= 122
		&buffer[1][TFT_SCRIPT_ROW(122)],
#endif
#if TFT_RAW_HEIGHT >= 123
		&buffer[1][TFT_SCRIPT_ROW(123)],
#endif
#if TFT_RAW_HEIGHT >= 124
		&buffer[1][TFT_SCRIPT_ROW(124)],
#endif
#if TFT_RAW_HEIGHT >= 125
		&buffer[1][TFT_SCRIPT_ROW(125)],
#endif
#if TFT_RAW_HEIGHT >= 126
		&buffer[1][TFT_SCRIPT_ROW(126)],
#endif
#if TFT_RAW_HEIGHT >= 127
		&buffer[1][TFT_SCRIPT_ROW(127)],
#endif
#if TFT_RAW_HEIGHT >= 128
		&buffer[1][TFT_SCRIPT_ROW(128)],
#endif
#if TFT_RAW_HEIGHT >= 129
		&buffer[1][TFT_SCRIPT_ROW(129)],
#endif
#if TFT_RAW_HEIGHT >= 130
		&buffer[1][TFT_SCRIPT_ROW(130)],
#endif
#if TFT_RAW_HEIGHT >= 131
		&buffer[1][TFT_SCRIPT_ROW(131)],
#endif
#if TFT_RAW_HEIGHT >= 132
		&buffer[1][TFT_SCRIPT_ROW(132)],
#endif
#if TFT_RAW_HEIGHT >= 133
		&buffer[1][TFT_SCRIPT_ROW(133)],
#endif
#if TFT_RAW_HEIGHT >= 134
		&buffer[1][TFT_SCRIPT_ROW(134)],
#endif
#if TFT_RAW_HEIGHT >= 135
		&buffer[1][TFT_SCRIPT_ROW(135)],
#endif
#if TFT_RAW_HEIGHT >= 136
		&buffer[1][TFT_SCRIPT_ROW(136)],
#endif
#if TFT_RAW_HEIGHT >= 137
		&buffer[1][TFT_SCRIPT_ROW(137)],
#endif
#if TFT_RAW_HEIGHT >= 138
		&buffer[1][TFT_SCRIPT_ROW(138)],
#endif
#if TFT_RAW_HEIGHT >= 139
		&buffer[1][TFT_SCRIPT_ROW(139)],
#endif
#if TFT_RAW_HEIGHT >= 140
		&buffer[1][TFT_SCRIPT_ROW(140)],
#endif
#if TFT_RAW_HEIGHT >= 141
		&buffer[1][TFT_SCRIPT_ROW(141)],
#endif
#if TFT_RAW_HEIGHT >= 142
		&buffer[1][TFT_SCRIPT_ROW(142)],
#endif
#if TFT_RAW_HEIGHT >= 143
		&buffer[1][TFT_SCRIPT_ROW(143)],
#endif
#if TFT_RAW_HEIGHT >= 144
		&buffer[1][TFT_SCRIPT_ROW(144)],
#endif
#if TFT_RAW_HEIGHT >= 145
		&buffer[1][TFT_SCRIPT_ROW(145)],
#endif
#if TFT_RAW_HEIGHT >= 146
		&buffer[1][TFT_SCRIPT_ROW(146)],
#endif
#if TFT_RAW_HEIGHT >= 147
		&buffer[1][TFT_SCRIPT_ROW(147)],
#endif
#if TFT_RAW_HEIGHT >= 148
		&buffer[1][TFT_SCRIPT_ROW(148)],
#endif
#if TFT_RAW_HEIGHT >= 149
		&buffer[1][TFT_SCRIPT_ROW(149)],
#endif
#if TFT_RAW_HEIGHT >= 150
		&buffer[1][TFT_SCRIPT_ROW(150)],
#endif
#if TFT_RAW_HEIGHT >= 151
		&buffer[1][TFT_SCRIPT_ROW(151)],
#endif
#if TFT_RAW_HEIGHT >= 152
		&buffer[1][TFT_SCRIPT_ROW(152)],
#endif
#if TFT_RAW_HEIGHT >= 153
		&buffer[1][TFT_SCRIPT_ROW(153)],
#endif
#if TFT_RAW_HEIGHT >= 154
		&buffer[1][TFT_SCRIPT_ROW(154)],
#endif
#if TFT_RAW_HEIGHT >= 155
		&buffer[1][TFT_SCRIPT_ROW(155)],
#endif
#if TFT_RAW_HEIGHT >= 156
		&buffer[1][TFT_SCRIPT_ROW(156)],
#endif
#if TFT_RAW_HEIGHT >= 157
		&buffer[1][TFT_SCRIPT_ROW(157)],
#endif
#if TFT_RAW_HEIGHT >= 158
		&buffer[1][TFT_SCRIPT_ROW(158)],
#endif
#if TFT_RAW_HEIGHT >= 159
		&buffer[1][TFT_SCRIPT_ROW(159)],
#endif
#if TFT_RAW_HEIGHT >= 160
		&buffer[1][TFT_SCRIPT_ROW(160)],
#endif
#if TFT_RAW_HEIGHT >= 161
		&buffer[1][TFT_SCRIPT_ROW(161)],
#endif
#if TFT_RAW_HEIGHT >= 162
		&buffer[1][TFT_SCRIPT_ROW(162)],
#endif
#if TFT_RAW_HEIGHT >= 163
		&buffer[1][TFT_SCRIPT_ROW(163)],
#endif
#if TFT_RAW_HEIGHT >= 164
		&buffer[1][TFT_SCRIPT_ROW(164)],
#endif
#if TFT_RAW_HEIGHT >= 165
		&buffer[1][TFT_SCRIPT_ROW(165)],
#endif
#if TFT_RAW_HEIGHT >= 166
		&buffer[1][TFT_SCRIPT_ROW(166)],
#endif
#if TFT_RAW_HEIGHT >= 167
		&buffer[1][TFT_SCRIPT_ROW(167)],
#endif
#if TFT_RAW_HEIGHT >= 168
		&buffer[1][TFT_SCRIPT_ROW(168)],
#endif
#if TFT_RAW_HEIGHT >= 169
		&buffer[1][TFT_SCRIPT_ROW(169)],
#endif
#if TFT_RAW_HEIGHT >= 170
		&buffer[1][TFT_SCRIPT_ROW(170)],
#endif
#if TFT_RAW_HEIGHT >= 171
		&buffer[1][TFT_SCRIPT_ROW(171)],
#endif
#if TFT_RAW_HEIGHT >= 172
		&buffer[1][TFT_SCRIPT_ROW(172)],
#endif
#if TFT_RAW_HEIGHT >= 173
		&buffer[1][TFT_SCRIPT_ROW(173)],
#endif
#if TFT_RAW_HEIGHT >= 174
		&buffer[1][TFT_SCRIPT_ROW(174)],
#endif
#if TFT_RAW_HEIGHT >= 175
		&buffer[1][TFT_SCRIPT_ROW(175)],
#endif
#if TFT_RAW_HEIGHT >= 176
		&buffer[1][TFT_SCRIPT_ROW(176)],
#endif
#if TFT_RAW_HEIGHT >= 177
		&buffer[1][TFT_SCRIPT_ROW(177)],
#endif
#if TFT_RAW_HEIGHT >= 178
		&buffer[1][TFT_SCRIPT_ROW(178)],
#endif
#if TFT_RAW_HEIGHT >= 179
		&buffer[1][TFT_SCRIPT_ROW(179)],
#endif
#if TFT_RAW_HEIGHT >= 180
		&buffer[1][TFT_SCRIPT_ROW(180)],
#endif
#if TFT_RAW_HEIGHT >= 181
		&buffer[1][TFT_SCRIPT_ROW(181)],
#endif
#if TFT_RAW_HEIGHT >= 182
		&buffer[1][TFT_SCRIPT_ROW(182)],
#endif
#if TFT_RAW_HEIGHT >= 183
		&buffer[1][TFT_SCRIPT_ROW(183)],
#endif
#if TFT_RAW_HEIGHT >= 184
		&buffer[1][TFT_SCRIPT_ROW(184)],
#endif
#if TFT_RAW_HEIGHT >= 185
		&buffer[1][TFT_SCRIPT_ROW(185)],
#endif
#if TFT_RAW_HEIGHT >= 186
		&buffer[1][TFT_SCRIPT_ROW(186)],
#endif
#if TFT_RAW_HEIGHT >= 187
		&buffer[1][TFT_SCRIPT_ROW(187)],
#endif
#if TFT_RAW_HEIGHT >= 188
		&buffer[1][TFT_SCRIPT_ROW(188)],
#endif
#if TFT_RAW_HEIGHT >= 189
		&buffer[1][TFT_SCRIPT_ROW(189)],
#endif
#if TFT_RAW_HEIGHT >= 190
		&buffer[1][TFT_SCRIPT_ROW(190)],
#endif
#if TFT_RAW_HEIGHT >= 191
		&buffer[1][TFT_SCRIPT_ROW(191)],
#endif
#if TFT_RAW_HEIGHT >= 192
		&buffer[1][TFT_SCRIPT_ROW(192)],
#endif
#if TFT_RAW_HEIGHT >= 193
		&buffer[1][TFT_SCRIPT_ROW(193)],
#endif
#if TFT_RAW_HEIGHT >= 194
		&buffer[1][TFT_SCRIPT_ROW(194)],
#endif
#if TFT_RAW_HEIGHT >= 195
		&buffer[1][TFT_SCRIPT_ROW(195)],
#endif
#if TFT_RAW_HEIGHT >= 196
		&buffer[1][TFT_SCRIPT_ROW(196)],
#endif
#if TFT_RAW_HEIGHT >= 197
		&buffer[1][TFT_SCRIPT_ROW(197)],
#endif
#if TFT_RAW_HEIGHT >= 198
		&buffer[1][TFT_SCRIPT_ROW(198)],
#endif
#if TFT_RAW_HEIGHT >= 199
		&buffer[1][TFT_SCRIPT_ROW(199)],
#endif
#if TFT_RAW_HEIGHT >= 200
		&buffer[1][TFT_SCRIPT_ROW(200)],
#endif
#if TFT_RAW_HEIGHT >= 201
		&buffer[1][TFT_SCRIPT_ROW(201)],
#endif
#if TFT_RAW_HEIGHT >= 202
		&buffer[1][TFT_SCRIPT_ROW(202)],
#endif
#if TFT_RAW_HEIGHT >= 203
		&buffer[1][TFT_SCRIPT_ROW(203)],
#endif
#if TFT_RAW_HEIGHT >= 204
		&buffer[1][TFT_SCRIPT_ROW(204)],
#endif
#if TFT_RAW_HEIGHT >= 205
		&buffer[1][TFT_SCRIPT_ROW(205)],
#endif
#if TFT_RAW_HEIGHT >= 206
		&buffer[1][TFT_SCRIPT_ROW(206)],
#endif
#if TFT_RAW_HEIGHT >= 207
		&buffer[1][TFT_SCRIPT_ROW(207)],
#endif
#if TFT_RAW_HEIGHT >= 208
		&buffer[1][TFT_SCRIPT_ROW(208)],
#endif
#if TFT_RAW_HEIGHT >= 209
		&buffer[1][TFT_SCRIPT_ROW(209)],
#endif
#if TFT_RAW_HEIGHT >= 210
		&buffer[1][TFT_SCRIPT_ROW(210)],
#endif
#if TFT_RAW_HEIGHT >= 211
		&buffer[1][TFT_SCRIPT_ROW(211)],
#endif
#if TFT_RAW_HEIGHT >= 212
		&buffer[1][TFT_SCRIPT_ROW(212)],
#endif
#if TFT_RAW_HEIGHT >= 213
		&buffer[1][TFT_SCRIPT_ROW(213)],
#endif
#if TFT_RAW_HEIGHT >= 214
		&buffer[1][TFT_SCRIPT_ROW(214)],
#endif
#if TFT_RAW_HEIGHT >= 215
		&buffer[1][TFT_SCRIPT_ROW(215)],
#endif
#if TFT_RAW_HEIGHT >= 216
		&buffer[1][TFT_SCRIPT_ROW(216)],
#endif
#if TFT_RAW_HEIGHT >= 217
		&buffer[1][TFT_SCRIPT_ROW(217)],
#endif
#if TFT_RAW_HEIGHT >= 218
		&buffer[1][TFT_SCRIPT_ROW(218)],
#endif
#if TFT_RAW_HEIGHT >= 219
		&buffer[1][TFT_SCRIPT_ROW(219)],
#endif
#if TFT_RAW_HEIGHT >= 220
		&buffer[1][TFT_SCRIPT_ROW(220)],
#endif
#if TFT_RAW_HEIGHT >= 221
		&buffer[1][TFT_SCRIPT_ROW(221)],
#endif
#if TFT_RAW_HEIGHT >= 222
		&buffer[1][TFT_SCRIPT_ROW(222)],
#endif
#if TFT_RAW_HEIGHT >= 223
		&buffer[1][TFT_SCRIPT_ROW(223)],
#endif
#if TFT_RAW_HEIGHT >= 224
		&buffer[1][TFT_SCRIPT_ROW(224)],
#endif
#if TFT_RAW_HEIGHT >= 225
		&buffer[1][TFT_SCRIPT_ROW(225)],
#endif
#if TFT_RAW_HEIGHT >= 226
		&buffer[1][TFT_SCRIPT_ROW(226)],
#endif
#if TFT_RAW_HEIGHT >= 227
		&buffer[1][TFT_SCRIPT_ROW(227)],
#endif
#if TFT_RAW_HEIGHT >= 228
		&buffer[1][TFT_SCRIPT_ROW(228)],
#endif
#if TFT_RAW_HEIGHT >= 229
		&buffer[1][TFT_SCRIPT_ROW(229)],
#endif
#if TFT_RAW_HEIGHT >= 230
		&buffer[1][TFT_SCRIPT_ROW(230)],
#endif
#if TFT_RAW_HEIGHT >= 231
		&buffer[1][TFT_SCRIPT_ROW(231)],
#endif
#if TFT_RAW_HEIGHT >= 232
		&buffer[1][TFT_SCRIPT_ROW(232)],
#endif
#if TFT_RAW_HEIGHT >= 233
		&buffer[1][TFT_SCRIPT_ROW(233)],
#endif
#if TFT_RAW_HEIGHT >= 234
		&buffer[1][TFT_SCRIPT_ROW(234)],
#endif
#if TFT_RAW_HEIGHT >= 235
		&buffer[1][TFT_SCRIPT_ROW(235)],
#endif
#if TFT_RAW_HEIGHT >= 236
		&buffer[1][TFT_SCRIPT_ROW(236)],
#endif
#if TFT_RAW_HEIGHT >= 237
		&buffer[1][TFT_SCRIPT_ROW(237)],
#endif
#if TFT_RAW_HEIGHT >= 238
		&buffer[1][TFT_SCRIPT_ROW(238)],
#endif
#if TFT_RAW_HEIGHT >= 239
		&buffer[1][TFT_SCRIPT_ROW(239)],
#endif
#if TFT_RAW_HEIGHT >= 240
		&buffer[1][TFT_SCRIPT_ROW(240)],
#endif
#if TFT_RAW_HEIGHT >= 241
		&buffer[1][TFT_SCRIPT_ROW(241)],
#endif
#if TFT_RAW_HEIGHT >= 242
		&buffer[1][TFT_SCRIPT_ROW(242)],
#endif
#if TFT_RAW_HEIGHT >= 243
		&buffer[1][TFT_SCRIPT_ROW(243)],
#endif
#if TFT_RAW_HEIGHT >= 244
		&buffer[1][TFT_SCRIPT_ROW(244)],
#endif
#if TFT_RAW_HEIGHT >= 245
		&buffer[1][TFT_SCRIPT_ROW(245)],
#endif
#if TFT_RAW_HEIGHT >= 246
		&buffer[1][TFT_SCRIPT_ROW(246)],
#endif
#if TFT_RAW_HEIGHT >= 247
		&buffer[1][TFT_SCRIPT_ROW(247)],
#endif
#if TFT_RAW_HEIGHT >= 248
		&buffer[1][TFT_SCRIPT_ROW(248)],
#endif
#if TFT_RAW_HEIGHT >= 249
		&buffer[1][TFT_SCRIPT_ROW(249)],
#endif
#if TFT_RAW_HEIGHT >= 250
		&buffer[1][TFT_SCRIPT_ROW(250)],
#endif
#if TFT_RAW_HEIGHT >= 251
		&buffer[1][TFT_SCRIPT_ROW(251)],
#endif
#if TFT_RAW_HEIGHT >= 252
		&buffer[1][TFT_SCRIPT_ROW(252)],
#endif
#if TFT_RAW_HEIGHT >= 253
		&buffer[1][TFT_SCRIPT_ROW(253)],
#endif
#if TFT_RAW_HEIGHT >= 254
		&buffer[1][TFT_SCRIPT_ROW(254)],
#endif
#if TFT_RAW_HEIGHT >= 255
		&buffer[1][TFT_SCRIPT_ROW(255)],
#endif
#if TFT_RAW_HEIGHT >= 256
		&buffer[1][TFT_SCRIPT_ROW(256)],
#endif
#if TFT_RAW_HEIGHT >= 257
		&buffer[1][TFT_SCRIPT_ROW(257)],
#endif
#if TFT_RAW_HEIGHT >= 258
		&buffer[1][TFT_SCRIPT_ROW(258)],
#endif
#if TFT_RAW_HEIGHT >= 259
		&buffer[1][TFT_SCRIPT_ROW(259)],
#endif
#if TFT_RAW_HEIGHT >= 260
		&buffer[1][TFT_SCRIPT_ROW(260)],
#endif
#if TFT_RAW_HEIGHT >= 261
		&buffer[1][TFT_SCRIPT_ROW(261)],
#endif
#if TFT_RAW_HEIGHT >= 262
		&buffer[1][TFT_SCRIPT_ROW(262)],
#endif
#if TFT_RAW_HEIGHT >= 263
		&buffer[1][TFT_SCRIPT_ROW(263)],
#endif
#if TFT_RAW_HEIGHT >= 264
		&buffer[1][TFT_SCRIPT_ROW(264)],
#endif
#if TFT_RAW_HEIGHT >= 265
		&buffer[1][TFT_SCRIPT_ROW(265)],
#endif
#if TFT_RAW_HEIGHT >= 266
		&buffer[1][TFT_SCRIPT_ROW(266)],
#endif
#if TFT_RAW_HEIGHT >= 267
		&buffer[1][TFT_SCRIPT_ROW(267)],
#endif
#if TFT_RAW_HEIGHT >= 268
		&buffer[1][TFT_SCRIPT_ROW(268)],
#endif
#if TFT_RAW_HEIGHT >= 269
		&buffer[1][TFT_SCRIPT_ROW(269)],
#endif
#if TFT_RAW_HEIGHT >= 270
		&buffer[1][TFT_SCRIPT_ROW(270)],
#endif
#if TFT_RAW_HEIGHT >= 271
		&buffer[1][TFT_SCRIPT_ROW(271)],
#endif
#if TFT_RAW_HEIGHT >= 272
		&buffer[1][TFT_SCRIPT_ROW(272)],
#endif
#if TFT_RAW_HEIGHT >= 273
		&buffer[1][TFT_SCRIPT_ROW(273)],
#endif
#if TFT_RAW_HEIGHT >= 274
		&buffer[1][TFT_SCRIPT_ROW(274)],
#endif
#if TFT_RAW_HEIGHT >= 275
		&buffer[1][TFT_SCRIPT_ROW(275)],
#endif
#if TFT_RAW_HEIGHT >= 276
		&buffer[1][TFT_SCRIPT_ROW(276)],
#endif
#if TFT_RAW_HEIGHT >= 277
		&buffer[1][TFT_SCRIPT_ROW(277)],
#endif
#if TFT_RAW_HEIGHT >= 278
		&buffer[1][TFT_SCRIPT_ROW(278)],
#endif
#if TFT_RAW_HEIGHT >= 279
		&buffer[1][TFT_SCRIPT_ROW(279)],
#endif
#if TFT_RAW_HEIGHT >= 280
		&buffer[1][TFT_SCRIPT_ROW(280)],
#endif
#if TFT_RAW_HEIGHT >= 281
		&buffer[1][TFT_SCRIPT_ROW(281)],
#endif
#if TFT_RAW_HEIGHT >= 282
		&buffer[1][TFT_SCRIPT_ROW(282)],
#endif
#if TFT_RAW_HEIGHT >= 283
		&buffer[1][TFT_SCRIPT_ROW(283)],
#endif
#if TFT_RAW_HEIGHT >= 284
		&buffer[1][TFT_SCRIPT_ROW(284)],
#endif
#if TFT_RAW_HEIGHT >= 285
		&buffer[1][TFT_SCRIPT_ROW(285)],
#endif
#if TFT_RAW_HEIGHT >= 286
		&buffer[1][TFT_SCRIPT_ROW(286)],
#endif
#if TFT_RAW_HEIGHT >= 287
		&buffer[1][TFT_SCRIPT_ROW(287)],
#endif
#if TFT_RAW_HEIGHT >= 288
		&buffer[1][TFT_SCRIPT_ROW(288)],
#endif
#if TFT_RAW_HEIGHT >= 289
		&buffer[1][TFT_SCRIPT_ROW(289)],
#endif
#if TFT_RAW_HEIGHT >= 290
		&buffer[1][TFT_SCRIPT_ROW(290)],
#endif
#if TFT_RAW_HEIGHT >= 291
		&buffer[1][TFT_SCRIPT_ROW(291)],
#endif
#if TFT_RAW_HEIGHT >= 292
		&buffer[1][TFT_SCRIPT_ROW(292)],
#endif
#if TFT_RAW_HEIGHT >= 293
		&buffer[1][TFT_SCRIPT_ROW(293)],
#endif
#if TFT_RAW_HEIGHT >= 294
		&buffer[1][TFT_SCRIPT_ROW(294)],
#endif
#if TFT_RAW_HEIGHT >= 295
		&buffer[1][TFT_SCRIPT_ROW(295)],
#endif
#if TFT_RAW_HEIGHT >= 296
		&buffer[1][TFT_SCRIPT_ROW(296)],
#endif
#if TFT_RAW_HEIGHT >= 297
		&buffer[1][TFT_SCRIPT_ROW(297)],
#endif
#if TFT_RAW_HEIGHT >= 298
		&buffer[1][TFT_SCRIPT_ROW(298)],
#endif
#if TFT_RAW_HEIGHT >= 299
		&buffer[1][TFT_SCRIPT_ROW(299)],
#endif
#if TFT_RAW_HEIGHT >= 300
		&buffer[1][TFT_SCRIPT_ROW(300)],
#endif
#if TFT_RAW_HEIGHT >= 301
		&buffer[1][TFT_SCRIPT_ROW(301)],
#endif
#if TFT_RAW_HEIGHT >= 302
		&buffer[1][TFT_SCRIPT_ROW(302)],
#endif
#if TFT_RAW_HEIGHT >= 303
		&buffer[1][TFT_SCRIPT_ROW(303)],
#endif
#if TFT_RAW_HEIGHT >= 304
		&buffer[1][TFT_SCRIPT_ROW(304)],
#endif
#if TFT_RAW_HEIGHT >= 305
		&buffer[1][TFT_SCRIPT_ROW(305)],
#endif
#if TFT_RAW_HEIGHT >= 306
		&buffer[1][TFT_SCRIPT_ROW(306)],
#endif
#if TFT_RAW_HEIGHT >= 307
		&buffer[1][TFT_SCRIPT_ROW(307)],
#endif
#if TFT_RAW_HEIGHT >= 308
		&buffer[1][TFT_SCRIPT_ROW(308)],
#endif
#if TFT_RAW_HEIGHT >= 309
		&buffer[1][TFT_SCRIPT_ROW(309)],
#endif
#if TFT_RAW_HEIGHT >= 310
		&buffer[1][TFT_SCRIPT_ROW(310)],
#endif
#if TFT_RAW_HEIGHT >= 311
		&buffer[1][TFT_SCRIPT_ROW(311)],
#endif
#if TFT_RAW_HEIGHT >= 312
		&buffer[1][TFT_SCRIPT_ROW(312)],
#endif
#if TFT_RAW_HEIGHT >= 313
		&buffer[1][TFT_SCRIPT_ROW(313)],
#endif
#if TFT_RAW_HEIGHT >= 314
		&buffer[1][TFT_SCRIPT_ROW(314)],
#endif
#if TFT_RAW_HEIGHT >= 315
		&buffer[1][TFT_SCRIPT_ROW(315)],
#endif
#if TFT_RAW_HEIGHT >= 316
		&buffer[1][TFT_SCRIPT_ROW(316)],
#endif
#if TFT_RAW_HEIGHT >= 317
		&buffer[1][TFT_SCRIPT_ROW(317)],
#endif
#if TFT_RAW_HEIGHT >= 318
		&buffer[1][TFT_SCRIPT_ROW(318)],
#endif
#if TFT_RAW_HEIGHT >= 319
		&buffer[1][TFT_SCRIPT_ROW(319)],
#endif
		NULL,
	},
};
