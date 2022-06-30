/* FT232 polling reader
 *
 * 29/03/2020	First version
 *
 *
 * Author: Victor Preatoni
 *
 */

#include "ft232.h"



/* Creates ftdi_context struct
 */
FT232::FT232(QObject *parent)
	: QIODevice (parent)
{
	ftdi = ftdi_new();
}

/* Releases ftdi_context struct
 */
FT232::~FT232()
{
	ftdi_free(ftdi);
}

/* Open FT232 and sets basic parameters:
 * baudRate, latency and chunksize.
 * It will also try to read some FT232 information,
 * like chipID, vendor name and product name.
 *
 * Main polling thread is started here too
 */
bool FT232::open(QIODevice::OpenMode mode)
{
	int ret;
	struct ftdi_version_info version;

	ret = ftdi_usb_open(ftdi, usbVID, usbPID);
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		return false;
	}

	ret = ftdi_set_baudrate(ftdi, FTDIbaudRate);
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		close();
		return false;
	}

	ret = ftdi_set_latency_timer(ftdi, FTDI_LATENCY);
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		close();
		return false;
	}

	ret = ftdi_read_data_set_chunksize(ftdi, FTDI_MTU);
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		close();
		return false;
	}


	/* Now read some FT232R specifics
	 * Doesnt mater if error, that will
	 * be handled by user if read data
	 * makes non-sense
	 */
	ftdi_read_chipid(ftdi, &FTDIchipID);
	if (ftdi_read_eeprom(ftdi) == 0)
		ftdi_eeprom_decode(ftdi, 0);

	productName = QString(ftdi->eeprom->product);
	serialNmb = QString(ftdi->eeprom->serial).toUpper().toUtf8();
	manufacturerName = QString(ftdi->eeprom->manufacturer);
	version = ftdi_get_library_version();
	libraryVersion = version.version_str;

	/* Clear buffers */
	ftdi_tciflush(ftdi);

	/* Notify parent class we are open*/
	QIODevice::open(mode);

	/* Create new thread */
	auto thread = new QThread;
	connect(this, &FT232::destroyed, thread, &QThread::quit);
	connect(thread, &QThread::finished, thread, &QThread::deleteLater);

	/* Creater worker */
	worker = new FT232Poll(ftdi, &ftdiMutex);
	connect(this, &FT232::destroyed, worker, &FT232Poll::deleteLater);

	/* Move worker to thread */
	worker->moveToThread(thread);

	/* Clean connects */
	connect(worker, &FT232Poll::finished, this, &FT232::close);
	connect(worker, &FT232Poll::finished, thread, &QThread::quit);


	/* Sync connects */
	connect(thread, &QThread::started, worker, &FT232Poll::poll);

	connect(worker, &FT232Poll::readError, this, &FT232::on_FTDIerror);
	connect(worker, &FT232Poll::modemError, this, &FT232::on_FTDImodemError);
	connect(worker, &FT232Poll::dataReady, this, &FT232::on_FTDIreceive);

	thread->start();

	return true;
}


/* Stop working thread and
 * close port
 */
void FT232::close()
{
	/* If worker still working,
	 * stop it
	 */
	if (worker)
		worker->stop();

	/* Close FTDI
	 *
	 * Use mutex here to avoid polling
	 * thread to read while closing port
	 */
	ftdiMutex.lock();
	ftdi_usb_close(ftdi);
	ftdiMutex.unlock();

	setOpenMode(NotOpen);

	/* Signal we are closing */
	emit aboutToClose();
}


/* This function is called by QIODevice::read()
 */
qint64 FT232::readData(char *data, qint64 maxSize)
{
	/* Check bounds */
	qint64 n = qMin(maxSize, (qint64)FTDIreadBuffer.size());

	/* No data, return immediately */
	if (!n)
		return 0;

	/* Try to catch n bytes */
	if (!sem.tryAcquire(n))
		return 0;

	/* Copy data to QIODevice provided buffer */
	memcpy(data, FTDIreadBuffer.data(), n);

	/* Erase data from my buffer */
	FTDIreadBuffer.remove(0, n);

	return n;
}

/* This function is called by QIODevice::write()
 */
qint64 FT232::writeData(const char *data, qint64 maxSize)
{
	int ret;

	/* Write to FTDI
	 *
	 * Use mutex here to avoid writing
	 * while polling thread is reading
	 */
	ftdiMutex.lock();
	ret = ftdi_write_data(ftdi, (const unsigned char *)data, maxSize);
	ftdiMutex.unlock();

	/* If error, setErrorString */
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		return -1;
	}

	return ret;
}




bool FT232::setBaudRate(qint32 baud)
{
	int ret;
	FTDIbaudRate = baud;

	/* Change baud rate
	 *
	 * Use mutex here to avoid changing
	 * while polling thread is reading
	 */
	ftdiMutex.lock();
	ret = ftdi_set_baudrate(ftdi, baud);
	ftdiMutex.unlock();

	/* If error, setErrorString */
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		return false;
	}

	/* Signal baudrate has changed */
	emit baudRateChanged(baud);

	return true;
}


bool FT232::setLineProperty(LineProperty line)
{
	int ret;
	ftdi_bits_type bits;
	ftdi_stopbits_type sbit;
	ftdi_parity_type parity;

	bits = BITS_8;
	FTDIlineProperty = line;

	switch (line) {
	case SERIAL_8N1: parity = NONE; sbit = STOP_BIT_1; break;
	case SERIAL_8N2: parity = NONE; sbit = STOP_BIT_2; break;
	case SERIAL_8N15: parity = NONE; sbit = STOP_BIT_15; break;
	case SERIAL_8E1: parity = EVEN; sbit = STOP_BIT_1; break;
	case SERIAL_8E2: parity = EVEN; sbit = STOP_BIT_2; break;
	case SERIAL_8E15: parity = EVEN; sbit = STOP_BIT_15; break;
	case SERIAL_8O1: parity = ODD; sbit = STOP_BIT_1; break;
	case SERIAL_8O2: parity = ODD; sbit = STOP_BIT_2; break;
	case SERIAL_8O15: parity = ODD; sbit = STOP_BIT_15; break;
	case SERIAL_8M1: parity = MARK; sbit = STOP_BIT_1; break;
	case SERIAL_8M2: parity = MARK; sbit = STOP_BIT_2; break;
	case SERIAL_8M15: parity = MARK; sbit = STOP_BIT_15; break;
	case SERIAL_8S1: parity = SPACE; sbit = STOP_BIT_1; break;
	case SERIAL_8S2: parity = SPACE; sbit = STOP_BIT_2; break;
	case SERIAL_8S15: parity = SPACE; sbit = STOP_BIT_15; break;
	default: parity = NONE; sbit = STOP_BIT_1; break;
	}

	/* Change line properties
	 *
	 * Use mutex here to avoid changing
	 * while polling thread is reading
	 */
	ftdiMutex.lock();
	ret = ftdi_set_line_property(ftdi, bits, sbit, parity);
	ftdiMutex.unlock();

	/* If error, setErrorString */
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		return false;
	}

	/* Signal line property has changed */
	emit linePropertyChanged(line);

	return true;
}


bool FT232::setFlowControl(FlowControl flow)
{
	int ret;
	int flowctrl;

	switch (flow) {
	case NoFlowControl: flowctrl = SIO_DISABLE_FLOW_CTRL; break;
	case HardwareControl: flowctrl = SIO_RTS_CTS_HS; break;
	case SoftwareControl: flowctrl = SIO_XON_XOFF_HS; break;
	case DTR_DSR_FlowControl: flowctrl = SIO_DTR_DSR_HS; break;
	default: flowctrl = SIO_DISABLE_FLOW_CTRL;
	}

	/* Change flow control
	 *
	 * Use mutex here to avoid changing
	 * while polling thread is reading
	 */
	ftdiMutex.lock();
	ret = ftdi_setflowctrl(ftdi, flowctrl);
	ftdiMutex.unlock();

	/* If error, setErrorString */
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		return false;
	}

	FTDIflowControl = flow;
	/* Signal flow control has changed */
	emit flowControlChanged(flow);

	return true;
}


bool FT232::setDataTerminalReady(bool set)
{
	int ret;

	/* DTR can only be set while port
	 * is open
	 */
	if (!isOpen())
		return false;

	/* Set DTR
	 *
	 * Use mutex here to avoid changing
	 * while polling thread is reading
	 */
	ftdiMutex.lock();
	ret = ftdi_setdtr(ftdi, set);
	ftdiMutex.unlock();

	/* If error, setErrorString */
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		return false;
	}

	FTDIdtr = set;
	/* Signal DTR has changed */
	emit dataTerminalReadyChanged(set);

	return true;
}


bool FT232::setRequestToSend(bool set)
{
	int ret;

	/* RTS can only be set while port
	 * is open
	 */
	if (!isOpen())
		return false;

	/* Set RTS
	 *
	 * Use mutex here to avoid changing
	 * while polling thread is reading
	 */
	ftdiMutex.lock();
	ret = ftdi_setrts(ftdi, set);
	ftdiMutex.unlock();

	/* If error, setErrorString */
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		return false;
	}

	FTDIrts = set;
	/* Signal RTS has changed */
	emit requestToSendChanged(set);

	return true;
}


/* Parses first byte of modemStatus and returns
 * active data lines
 */
FT232::PinoutSignals FT232::pinoutSignals()
{
	/* Line status bit field
	*	B0..B3 - must be 0
	*	B4 Clear to send (CTS) 0 = inactive; 1 = active
	*	B5 Data set ready (DTS) 0 = inactive; 1 = active
	*	B6 Ring indicator (RI) 0 = inactive; 1 = active
	*	B7 Receive line signal detect (RLSD) 0 = inactive; 1 = active
	*/

	int ret;

	/* Signals can only be
	 * read while port is open
	 */
	if (!isOpen())
		return NoSignal;

	unsigned short modemStatus;
	/* Read modem status
	 *
	 * Use mutex here to avoid reading status
	 * while polling thread is reading data
	 */
	ftdiMutex.lock();
	ret = ftdi_poll_modem_status(ftdi, &modemStatus);
	ftdiMutex.unlock();

	/* If error, setErrorString */
	if (ret < 0) {
		setErrorString(ftdi_get_error_string(ftdi));
		return NoSignal;
	}

	FT232::PinoutSignals retval = NoSignal;

	if (modemStatus & 0x0080)
		retval |= ReceivedDataSignal;

	if (modemStatus & 0x0040)
		retval |= RingIndicatorSignal;

	if (modemStatus & 0x0020)
		retval |= DataSetReadySignal;

	if (modemStatus & 0x0010)
		retval |= ClearToSendSignal;

	return retval;
}


/* Parses second byte of modemStatus and returns
 * active error flags
 */
void FT232::on_FTDImodemError(unsigned short modemStatus)
{
	/* modemStatus Bit field
	* B8	Data Ready (DR)
	* B9	Overrun Error (OE)
	* B10	Parity Error (PE)
	* B11	Framing Error (FE)
	* B12	Break Interrupt (BI)
	* B13	Transmitter Holding Register (THRE)
	* B14	Transmitter Empty (TEMT)
	* B15	Error in RCVR FIFO
	*/


	FT232::PortErrors retval = NoError;

	if (modemStatus & 0x8000)
		retval |= FIFOError;

	if (modemStatus & 0x1000)
		retval |= BreakConditionError;

	if (modemStatus & 0x0800)
		retval |= FramingError;

	if (modemStatus & 0x0400)
		retval |= ParityError;

	if (modemStatus & 0x0200)
		retval |= OverrunError;

	errFlag = retval;

	//Modem errors are quite frequent, so not emited
//	emit errorOccurred();
}


/* Returns the number of bytes that are available for reading.
 * Subclasses that reimplement this function must call
 * the base implementation in order to include the size of the buffer of QIODevice
 */
qint64 FT232::bytesAvailable() const
{
	qint64 my_size = FTDIreadBuffer.size();
	qint64 builtin_size = QIODevice::bytesAvailable();

	return (my_size + builtin_size);
}


/* Write received data to the end of internal
 * intermediate buffer
 */
void FT232::on_FTDIreceive(QByteArray data)
{
	/* Append new data */
	FTDIreadBuffer.append(data);

	/* Release n bytes */
	sem.release(data.size());

	/* Emit signals */
	emit readyRead();
	emit QIODevice::readyRead();
}


/* Slot for thread to inform read error
 */
void FT232::on_FTDIerror()
{
	/* setErrorString */
	setErrorString(ftdi_get_error_string(ftdi));

	if (!isOpen())
		errFlag = NotOpenError;
	else
		errFlag = ReadError;

	emit errorOccurred();
}


/* Timeout blocking function that waits
 * for bytes available on buffer to read
 */
bool FT232::waitForReadyRead(int msecs)
{
	if (!isOpen())
		return false;

	/* Create a timer and an
	 * Event Loop
	 */
	QEventLoop loop;
	QTimer timer;
	timer.setSingleShot(true);
	connect(this, &FT232::readyRead, &loop, &QEventLoop::quit);
	connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

	/* Trigger timer and run
	 * event loop
	 */
	timer.start(msecs);
	loop.exec();

	/* If timer timed out, means
	 * we had a timeout!
	 */
	if (!timer.isActive()) {
		setErrorString(tr("Read timeout"));
		return false;
	}

	/* Otherwise, we exit event loop because
	 * we catch a readyRead signal
	 */
	return true;
}


/* FTDI reader worker thread */
void FT232Poll::poll()
{
	/* modemStatus Bit field  (*):serious errors
	* B8	Data Ready (DR)
	* B9*	Overrun Error (OE)
	* B10*	Parity Error (PE)
	* B11*	Framing Error (FE)
	* B12	Break Interrupt (BI)
	* B13	Transmitter Holding Register (THRE)
	* B14	Transmitter Empty (TEMT)
	* B15*	Error in RCVR FIFO */
	unsigned short modemStatus;

	char buff[FTDI_MTU];
	int ret;

	pauseCond = false;
	stopCond = false;

	while (!stopCond) {
		/* Get mutex before reading */
		mutexptr->lock();
		ret = ftdi_poll_modem_status(ftptr, &modemStatus);
		mutexptr->unlock();

		/* Very serious error, stop thread */
		if (ret < 0) {
			emit readError();
			stop();

			continue;
		}

		/* Mask out serious errors */
		if (modemStatus & 0b1000111000000000) {
			emit modemError(modemStatus);
			/* Get mutex before flushing buffers */
			mutexptr->lock();
			ftdi_tciflush(ftptr);
			mutexptr->unlock();

			continue;
		}

		/* Get mutex before reading */
		mutexptr->lock();
		ret = ftdi_read_data(ftptr, (unsigned char *)buff, sizeof (buff));
		mutexptr->unlock();

		/* FTDI buffer overflow */
		if (ret == LIBUSB_ERROR_OVERFLOW) {
			emit readError();
			/* Get mutex before flushing buffers */
			mutexptr->lock();
			ftdi_tciflush(ftptr);
			mutexptr->unlock();

			continue;
		}

		/* Very serious error, stop thread */
		if (ret < 0) {
			emit readError();
			stop();

			continue;
		}

		/* Read OK, emit data */
		if (ret > 0)
			emit dataReady(QByteArray(buff, ret));

		/* Check if a pause was requested */
		pauseMutex.lock();
		if (pauseCond)
			resumeCond.wait(&pauseMutex);
		pauseMutex.unlock();
	} //End loop

	emit finished();
}


/* Static function that finds all available FTDI
 * ports and returns a list to FT232Info items
 */
QList<FT232Info> FT232Info::availablePorts(int VID, int PID)
{
	QList<FT232Info> list;
	list.clear();
	FT232Info device;

	struct ftdi_context ftdic;
	struct ftdi_device_list *devlist, *curdev;
	char description[32], serialNum[32], manufacturer[32];

	/* If cannot init ftdi,
	 * return empty list
	 */
	if (ftdi_init(&ftdic) < 0)
		return list;

	/* Find FTDI devices using selected USB VID and PID */
	if (ftdi_usb_find_all(&ftdic, &devlist, VID, PID) > 0) {
		/* Iterate thru list */
		for (curdev = devlist; curdev != nullptr; curdev = curdev->next) {
			/* Try to read FTDI data */
			if (ftdi_usb_get_strings(&ftdic, curdev->dev, manufacturer, sizeof(manufacturer),
									 description, sizeof(description), serialNum, sizeof(serialNum)) < 0)
				/* Return if fail*/
				return list;

			device.manuf = manufacturer;
			device.descr = description;
			device.serialN = serialNum;
			device.vid = VID;
			device.pid = PID;

			/* Append data to list */
			list.append(device);
		}
	}

	/* Release resources */
	ftdi_list_free(&devlist);
	ftdi_deinit(&ftdic);

	return list;
}
