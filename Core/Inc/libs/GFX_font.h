#ifndef FONT5X7_H
#define FONT5X7_H

static const unsigned char battery_bitmap[] = {
		0x7E, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x66, 0x3C
};

static const unsigned char bell_bitmap[] = {
		0x30, 0x2C, 0x62, 0xA1, 0xA1, 0x62, 0x2C, 0x30
};

static const unsigned char unmuted_bitmap[] = {
	0x00, 0x3C, 0x24, 0x3C, 0x42, 0x81, 0xFF, 0x00
};

static const unsigned char charge_bitmap[] = {
	0x7E, 0x42, 0x42, 0x5A, 0x66, 0x66, 0x42, 0x42, 0x66, 0x3C
};

static const unsigned char backlight_bitmap[] = {
	0x00, 0x1C, 0x62, 0xC1, 0xC1, 0x62, 0x1C, 0x00
};

static const unsigned char death_bitmap[] = {
	0x3C, 0xE6, 0x76, 0xDE, 0x76, 0xE6, 0x3C, 0x00
};

static const unsigned char satellite_bitmap[] = {
		0x3C, 0x42, 0x99, 0xA5, 0xB5, 0x91, 0x42, 0x3C
};

// Standard ASCII 5x7 font
static const unsigned char font[] = {
		0x00, 0x00, 0x00, 0x00, 0x00 ,  // 0x20   space
		  0x00, 0x00, 0x5f, 0x00, 0x00 ,  // 0x21   !
		  0x00, 0x07, 0x00, 0x07, 0x00 ,  // 0x22   "
		  0x14, 0x7f, 0x14, 0x7f, 0x14 ,  // 0x23   #
		  0x24, 0x2a, 0x7f, 0x2a, 0x12 ,  // 0x24   $
		  0x23, 0x13, 0x08, 0x64, 0x62 ,  // 0x25   %
		  0x36, 0x49, 0x55, 0x22, 0x50 ,  // 0x26   &
		  0x00, 0x05, 0x03, 0x00, 0x00 ,  // 0x27   '
		  0x00, 0x1c, 0x22, 0x41, 0x00 ,  // 0x28   (
		  0x00, 0x41, 0x22, 0x1c, 0x00 ,  // 0x29   )
		  0x14, 0x08, 0x3e, 0x08, 0x14 ,  // 0x2a   *
		  0x08, 0x08, 0x3e, 0x08, 0x08 ,  // 0x2b   +
		  0x00, 0x50, 0x30, 0x00, 0x00 ,  // 0x2c   ,
		  0x08, 0x08, 0x08, 0x08, 0x08 ,  // 0x2d   -
		  0x00, 0x60, 0x60, 0x00, 0x00 ,  // 0x2e   .
		  0x20, 0x10, 0x08, 0x04, 0x02 ,  // 0x2f   /
		  0x3e, 0x51, 0x49, 0x45, 0x3e ,  // 0x30   0
		  0x00, 0x42, 0x7f, 0x40, 0x00 ,  // 0x31   1
		  0x42, 0x61, 0x51, 0x49, 0x46 ,  // 0x32   2
		  0x21, 0x41, 0x45, 0x4b, 0x31 ,  // 0x33   3
		  0x18, 0x14, 0x12, 0x7f, 0x10 ,  // 0x34   4
		  0x27, 0x45, 0x45, 0x45, 0x39 ,  // 0x35   5
		  0x3c, 0x4a, 0x49, 0x49, 0x30 ,  // 0x36   6
		  0x01, 0x71, 0x09, 0x05, 0x03 ,  // 0x37   7
		  0x36, 0x49, 0x49, 0x49, 0x36 ,  // 0x38   8
		  0x06, 0x49, 0x49, 0x29, 0x1e ,  // 0x39   9
		  0x00, 0x36, 0x36, 0x00, 0x00 ,  // 0x3a   :
		  0x00, 0x56, 0x36, 0x00, 0x00 ,  // 0x3b   ;
		  0x08, 0x14, 0x22, 0x41, 0x00 ,  // 0x3c   <
		  0x14, 0x14, 0x14, 0x14, 0x14 ,  // 0x3d   =
		  0x00, 0x41, 0x22, 0x14, 0x08 ,  // 0x3e   >
		  0x02, 0x01, 0x51, 0x09, 0x06 ,  // 0x3f   ?
		  0x32, 0x49, 0x79, 0x41, 0x3e ,  // 0x40   @
		  0x7e, 0x11, 0x11, 0x11, 0x7e ,  // 0x41   A
		  0x7f, 0x49, 0x49, 0x49, 0x36 ,  // 0x42   B
		  0x3e, 0x41, 0x41, 0x41, 0x22 ,  // 0x43   C
		  0x7f, 0x41, 0x41, 0x22, 0x1c ,  // 0x44   D
		  0x7f, 0x49, 0x49, 0x49, 0x41 ,  // 0x45   E
		  0x7f, 0x09, 0x09, 0x09, 0x01 ,  // 0x46   F
		  0x3e, 0x41, 0x49, 0x49, 0x7a ,  // 0x47   G
		  0x7f, 0x08, 0x08, 0x08, 0x7f ,  // 0x48   H
		  0x00, 0x41, 0x7f, 0x41, 0x00 ,  // 0x49   I
		  0x20, 0x40, 0x41, 0x3f, 0x01 ,  // 0x4a   J
		  0x7f, 0x08, 0x14, 0x22, 0x41 ,  // 0x4b   K
		  0x7f, 0x40, 0x40, 0x40, 0x40 ,  // 0x4c   L
		  0x7f, 0x02, 0x0c, 0x02, 0x7f ,  // 0x4d   M
		  0x7f, 0x04, 0x08, 0x10, 0x7f ,  // 0x4e   N
		  0x3e, 0x41, 0x41, 0x41, 0x3e ,  // 0x4f   O
		  0x7f, 0x09, 0x09, 0x09, 0x06 ,  // 0x50   P
		  0x3e, 0x41, 0x51, 0x21, 0x5e ,  // 0x51   Q
		  0x7f, 0x09, 0x19, 0x29, 0x46 ,  // 0x52   R
		  0x46, 0x49, 0x49, 0x49, 0x31 ,  // 0x53   S
		  0x01, 0x01, 0x7f, 0x01, 0x01 ,  // 0x54   T
		  0x3f, 0x40, 0x40, 0x40, 0x3f ,  // 0x55   U
		  0x1f, 0x20, 0x40, 0x20, 0x1f ,  // 0x56   V
		  0x3f, 0x40, 0x38, 0x40, 0x3f ,  // 0x57   W
		  0x63, 0x14, 0x08, 0x14, 0x63 ,  // 0x58   X
		  0x07, 0x08, 0x70, 0x08, 0x07 ,  // 0x59   Y
		  0x61, 0x51, 0x49, 0x45, 0x43 ,  // 0x5a   Z
		  0x00, 0x7f, 0x41, 0x41, 0x00 ,  // 0x5b   [
		  0x02, 0x04, 0x08, 0x10, 0x20 ,  // 0x5c   backslash
		  0x00, 0x41, 0x41, 0x7f, 0x00 ,  // 0x5d   ]
		  0x04, 0x02, 0x01, 0x02, 0x04 ,  // 0x5e   ^
		  0x40, 0x40, 0x40, 0x40, 0x40 ,  // 0x5f   _
		  0x00, 0x24, 0x2E, 0x24, 0x00,  // 0x60   +-
		  0x20, 0x54, 0x54, 0x54, 0x78 ,  // 0x61   a
		  0x7f, 0x48, 0x44, 0x44, 0x38 ,  // 0x62   b
		  0x38, 0x44, 0x44, 0x44, 0x20 ,  // 0x63   c
		  0x38, 0x44, 0x44, 0x48, 0x7f ,  // 0x64   d
		  0x38, 0x54, 0x54, 0x54, 0x18 ,  // 0x65   e
		  0x08, 0x7e, 0x09, 0x01, 0x02 ,  // 0x66   f
		  0x0c, 0x52, 0x52, 0x52, 0x3e ,  // 0x67   g
		  0x7f, 0x08, 0x04, 0x04, 0x78 ,  // 0x68   h
		  0x00, 0x44, 0x7d, 0x40, 0x00 ,  // 0x69   i
		  0x20, 0x40, 0x44, 0x3d, 0x00 ,  // 0x6a   j
		  0x7f, 0x10, 0x28, 0x44, 0x00 ,  // 0x6b   k
		  0x00, 0x41, 0x7f, 0x40, 0x00 ,  // 0x6c   l
		  0x7c, 0x04, 0x18, 0x04, 0x78 ,  // 0x6d   m
		  0x7c, 0x08, 0x04, 0x04, 0x78 ,  // 0x6e   n
		  0x38, 0x44, 0x44, 0x44, 0x38 ,  // 0x6f   o
		  0x7c, 0x14, 0x14, 0x14, 0x08 ,  // 0x70   p
		  0x08, 0x14, 0x14, 0x18, 0x7c ,  // 0x71   q
		  0x7c, 0x08, 0x04, 0x04, 0x08 ,  // 0x72   r
		  0x48, 0x54, 0x54, 0x54, 0x20 ,  // 0x73   s
		  0x04, 0x3f, 0x44, 0x40, 0x20 ,  // 0x74   t
		  0x3c, 0x40, 0x40, 0x20, 0x7c ,  // 0x75   u
		  0x1c, 0x20, 0x40, 0x20, 0x1c ,  // 0x76   v
		  0x3c, 0x40, 0x30, 0x40, 0x3c ,  // 0x77   w
		  0x44, 0x28, 0x10, 0x28, 0x44 ,  // 0x78   x
		  0x0c, 0x50, 0x50, 0x50, 0x3c ,  // 0x79   y
		  0x44, 0x64, 0x54, 0x4c, 0x44 ,  // 0x7a   z
		  0x00, 0x08, 0x36, 0x41, 0x00 ,  // 0x7b   {
		  0x00, 0x00, 0x7f, 0x00, 0x00 ,  // 0x7c   |
		  0x00, 0x41, 0x36, 0x08, 0x00 ,  // 0x7d   }
		  0x10, 0x08, 0x08, 0x10, 0x08 ,  // 0x7e   ~
		  0x00, 0x00, 0x00, 0x00, 0x00 ,  // 0x7f
		  0x7e, 0x11, 0x11, 0x11, 0x7e ,  // 0x80   A  // Русские символы
		  0x7f, 0x49, 0x49, 0x49, 0x33 ,  // 0x81   Б
		  0x7f, 0x49, 0x49, 0x49, 0x36 ,  // 0x82   В
		  0x7f, 0x01, 0x01, 0x01, 0x03 ,  // 0x83   Г
		  0xe0, 0x51, 0x4f, 0x41, 0xff ,  // 0x84   Д
		  0x7f, 0x49, 0x49, 0x49, 0x41 ,  // 0x85   E
		  0x77, 0x08, 0x7f, 0x08, 0x77 ,  // 0x86   Ж
		  0x41, 0x49, 0x49, 0x49, 0x36 ,  // 0x87   З
		  0x7f, 0x10, 0x08, 0x04, 0x7f ,  // 0x88   И
		  0x7c, 0x21, 0x12, 0x09, 0x7c ,  // 0x89   Й
		  0x7f, 0x08, 0x14, 0x22, 0x41 ,  // 0x8A   K
		  0x20, 0x41, 0x3f, 0x01, 0x7f ,  // 0x8B   Л
		  0x7f, 0x02, 0x0c, 0x02, 0x7f ,  // 0x8C   M
		  0x7f, 0x08, 0x08, 0x08, 0x7f ,  // 0x8D   H
		  0x3e, 0x41, 0x41, 0x41, 0x3e ,  // 0x8E   O
		  0x7f, 0x01, 0x01, 0x01, 0x7f ,  // 0x8F   П
		  0x7f, 0x09, 0x09, 0x09, 0x06 ,  // 0x90   P
		  0x3e, 0x41, 0x41, 0x41, 0x22 ,  // 0x91   C
		  0x01, 0x01, 0x7f, 0x01, 0x01 ,  // 0x92   T
		  0x47, 0x28, 0x10, 0x08, 0x07 ,  // 0x93   У
		  0x1c, 0x22, 0x7f, 0x22, 0x1c ,  // 0x94   Ф
		  0x63, 0x14, 0x08, 0x14, 0x63 ,  // 0x95   X
		  0x7f, 0x40, 0x40, 0x40, 0xff ,  // 0x96   Ц
		  0x07, 0x08, 0x08, 0x08, 0x7f ,  // 0x97   Ч
		  0x7f, 0x40, 0x7f, 0x40, 0x7f ,  // 0x98   Ш
		  0x7f, 0x40, 0x7f, 0x40, 0xff ,  // 0x99   Щ
		  0x01, 0x7f, 0x48, 0x48, 0x30 ,  // 0x9A   Ъ
		  0x7f, 0x48, 0x30, 0x00, 0x7f ,  // 0x9B   Ы
		  0x00, 0x7f, 0x48, 0x48, 0x30 ,  // 0x9C   Э
		  0x22, 0x41, 0x49, 0x49, 0x3e ,  // 0x9D   Ь
		  0x7f, 0x08, 0x3e, 0x41, 0x3e ,  // 0x9E   Ю
		  0x46, 0x29, 0x19, 0x09, 0x7f ,  // 0x9F   Я
		  0x20, 0x54, 0x54, 0x54, 0x78 ,  // 0xA0   a
		  0x3c, 0x4a, 0x4a, 0x49, 0x31 ,  // 0xA1   б
		  0x7c, 0x54, 0x54, 0x54, 0x28 ,  // 0xA2   в
		  0x7c, 0x04, 0x04, 0x04, 0x0c ,  // 0xA3   г
		  0xe0, 0x54, 0x4c, 0x44, 0xfc ,  // 0xA4   д
		  0x38, 0x54, 0x54, 0x54, 0x18 ,  // 0xA5   e
		  0x6c, 0x10, 0x7c, 0x10, 0x6c ,  // 0xA6   ж
		  0x44, 0x44, 0x54, 0x54, 0x28 ,  // 0xA7   з
		  0x7c, 0x20, 0x10, 0x08, 0x7c ,  // 0xA8   и
		  0x7c, 0x41, 0x22, 0x11, 0x7c ,  // 0xA9   й
		  0x7c, 0x10, 0x10, 0x28, 0x44 ,  // 0xAA   к
		  0x20, 0x44, 0x3c, 0x04, 0x7c ,  // 0xAB   л
		  0x7c, 0x08, 0x10, 0x08, 0x7c ,  // 0xAC   м
		  0x7c, 0x10, 0x10, 0x10, 0x7c ,  // 0xAD   н
		  0x38, 0x44, 0x44, 0x44, 0x38 ,  // 0xAE   o
		  0x7c, 0x04, 0x04, 0x04, 0x7c ,  // 0xAF   п
		  0x7C, 0x14, 0x14, 0x14, 0x08 ,  // 0xB0   p
		  0x38, 0x44, 0x44, 0x44, 0x20 ,  // 0xB1   c
		  0x04, 0x04, 0x7c, 0x04, 0x04 ,  // 0xB2   т
		  0x0C, 0x50, 0x50, 0x50, 0x3C ,  // 0xB3   у
		  0x30, 0x48, 0xfc, 0x48, 0x30 ,  // 0xB4   ф
		  0x44, 0x28, 0x10, 0x28, 0x44 ,  // 0xB5   x
		  0x7c, 0x40, 0x40, 0x40, 0xfc ,  // 0xB6   ц
		  0x0c, 0x10, 0x10, 0x10, 0x7c ,  // 0xB7   ч
		  0x7c, 0x40, 0x7c, 0x40, 0x7c ,  // 0xB8   ш
		  0x7c, 0x40, 0x7c, 0x40, 0xfc ,  // 0xB9   щ
		  0x04, 0x7c, 0x50, 0x50, 0x20 ,  // 0xBA   ъ
		  0x7c, 0x50, 0x50, 0x20, 0x7c ,  // 0xBB   ы
		  0x7c, 0x50, 0x50, 0x20, 0x00 ,  // 0xBC   ь
		  0x28, 0x44, 0x54, 0x54, 0x38 ,  // 0xBD   э
		  0x7c, 0x10, 0x38, 0x44, 0x38 ,  // 0xBE   ю
		  0x08, 0x54, 0x34, 0x14, 0x7c ,  // 0xBF   я
};
#endif // FONT5X7_H
