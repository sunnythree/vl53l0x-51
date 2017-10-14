# vl53l0x-51


程序使用的串口为串口2，对应的是p1^0(rx2)和P1^1(tx2)引脚
i2c是软件模拟的，使用的引脚是p1^2(scl)和p1^3(sdl)

结果展示：
val--精确到毫米
c0,c1,c2为验证i2c正确性的，值固定为0xee,0xaa,0x10
c0=ee  
c1=aa
c2=10
val =0
val =8191
val =8191
val =8191
val =383
val =383
val =383
val =383
val =383
val =383
val =383