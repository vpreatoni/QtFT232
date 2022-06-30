# QtFT232
Qt5 FTDI FT232 full featured class

FTDI chips are great!! They save us a lot of time, they work quite well, and they almost don't need any device driver.
 

FTDI also provides a very nice and well documented DLL for low level access to FTDI chip internals, but, of course, it is closed source. If you need low level access to FTDI chips, there is an open-source alternative:

http://developer.intra2net.com/git/?p=libftdi

git://developer.intra2net.com/libftdi

 
I highly recommend Intra2Net libFTDI, as it is portable to any platform, and works very fast and very well.

 
Libraries provide a nice low-level access to FTDI chips, but unfortunately, end-users don't like low level UIs. So, you will most probably be using your FT232 chip with some kind of graphical interface. Well, Qt is a very nice piece of software, it has an excellent GUI, it is portable to almost any current platform, and it is very lightweight and fast!!!. It provides a lot of classes and libraries for serial port acces (QSerialPort), Qt Bluetooth, and so.

Unfortunately, it does not provide a class for FTDI chip low level access. Of course you can use standard QSerialPort for basic read and write functions, but if you need some advanced methods (like custom baud rates, reading internal EEPROM, using GPIOs), you will need a custom class.
 

Here is a basic implementation of the FT232 class, trying to copy some of the QSerialPort behaviour.
