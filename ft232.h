#ifndef FT232_H
#define FT232_H



#include <QIODevice>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QTimer>
#include <QSemaphore>
#include <QPointer>
#include <QList>
#include <QEventLoop>
#include <QDebug>

#include "ftdi.h"
#include "ftdi_i.h"
#include "libusb.h"


/* USB max packet size = 64 bytes */
static constexpr uint16_t	FTDI_MTU		=	64;
/* Timeout before ftdi_read function returns if not enough data */
static constexpr uint16_t	FTDI_LATENCY	=	10;
/* FTDI fixed port name */
static constexpr const char *FTDI_NAME		=	"FTDI";
/* Default FTDI  port parameters */
static constexpr int FTDI_VID					=	0x0403;
static constexpr int FTDI_PID					=	0x1925;


/* Polling class
 *
 * Since libftdi does not provide a metod to
 * get notified when new data arrives, only
 * way is to have a worker thread to poll FT232 every
 * FTDI_LATENCY milliseconds.
 *
 * Constructor parameters:
 *		struct ftdi_context*: already opened ftdi
 *		QMutex*: mutex created on main class, used
 *				to avoid concurrent access to
 *				ftdi_context*
 *
 * poll(): main worker thread loop
 * stop(): call this to stop thread
 * pause(): pauses thread
 *
 * finished(): emited when main loop ends
 * dataReady(): emited when new data received
 * readError(): emited when error trying to read data
 * modemError(): emited when serial modem errors
 */
class FT232Poll : public QObject {
	Q_OBJECT

public:
	FT232Poll(struct ftdi_context * p, QMutex * mtx) {
		ftptr = p;
		mutexptr = mtx;
	}

	void stop() {
		stopCond = true;
	}

	void pause(bool cond) {
		pauseMutex.lock();
		pauseCond = cond;
		pauseMutex.unlock();
		if (!cond)
			resumeCond.wakeAll();
	}

public slots:
	void poll();

signals:
	void finished();
	void dataReady(QByteArray);
	void readError();
	void modemError(unsigned short);

private:
	struct ftdi_context *ftptr;
	QMutex * mutexptr;
	QWaitCondition resumeCond;
	QMutex pauseMutex;
	bool stopCond, pauseCond;
};




/* Main FT232 class
 *
 * Mainly copied from QSerialPort class with some
 * mods to fit FT232.
 * Eg.:		line property is set on a single fuction call
 *			setPort() and portName() will get/return USB VID/PID
 *			values.
 *			setBaudRate() will accept any arbitrary baudrate
 * on_FTDIreceive(): slot for receiving Thread data
 * on_FTDIerror(): slot for receiving Thread errors
 * on_FTDImodemError(): slot for receiving Thread modem errors
 *
 * When data is received, it will be stored on an internal
 * buffer. readyRead() signal will be emitted.
 *
 * QIODevice::read() function call will read from this buffer,
 * so call is non-blocking.
 *
 * If need a blocking call, waitForReadyRead() is implemented
 * and will return true if new data is available.
 *
 * Check bytesAvailable() if need to know how many bytes are
 * stored on buffer.
 *
 */
class FT232 : public QIODevice
{
	Q_OBJECT

public:
	enum PortError {NoError = 0x00, NotOpenError = 0x01, OverrunError = 0x02, ParityError = 0x04,
					FramingError = 0x10, BreakConditionError = 0x20, FIFOError = 0x40, ReadError = 0x80};
	Q_FLAG(PortError)
	Q_DECLARE_FLAGS(PortErrors, PortError)

	enum LineProperty {SERIAL_8N1, SERIAL_8N2, SERIAL_8N15, SERIAL_8E1, SERIAL_8E2, SERIAL_8E15,
					   SERIAL_8O1, SERIAL_8O2, SERIAL_8O15, SERIAL_8M1, SERIAL_8M2, SERIAL_8M15,
					   SERIAL_8S1, SERIAL_8S2, SERIAL_8S15};
	Q_ENUM(LineProperty)

	enum FlowControl {NoFlowControl, HardwareControl, SoftwareControl, DTR_DSR_FlowControl};
	Q_ENUM(FlowControl)

	enum PinoutSignal {NoSignal = 0x00, ReceivedDataSignal = 0x02, DataSetReadySignal = 0x10,
					   RingIndicatorSignal = 0x20, ClearToSendSignal = 0x80};
	Q_FLAG(PinoutSignal)
	Q_DECLARE_FLAGS(PinoutSignals, PinoutSignal)

	FT232(QObject * parent = nullptr);
    virtual ~FT232();
	bool open(QIODevice::OpenMode mode = QIODevice::ReadWrite);
	bool isSequential() const {return true;}
	qint64 bytesAvailable() const;
	bool waitForReadyRead(int msecs = 30000);

	void setPort(int VID = FTDI_VID, int PID = FTDI_PID) {usbVID = VID; usbPID = PID;}
	bool setBaudRate(qint32 baud);
	qint32 baudRate() {return FTDIbaudRate;}
	bool setLineProperty(LineProperty line);
	LineProperty lineProperty() {return  FTDIlineProperty;}
	bool setFlowControl(FlowControl flow);
	FlowControl flowControl() {return  FTDIflowControl;}
	bool setDataTerminalReady(bool set);
	bool isDataTerminalReady() {return FTDIdtr;}
	bool setRequestToSend(bool set);
	bool isRequestToSend() {return FTDIrts;}
	PinoutSignals pinoutSignals();
	PortErrors error() {return errFlag;}
	void clearError() {errFlag = NoError;}

	/* FT232 specifics QSerialPortInfo like functions */
	unsigned int chipID() {return FTDIchipID;}
	bool hasVendorIdentifier() {return true;}
	bool hasProductIdentifier() {return true;}
	int vendorIdentifier() {return usbVID;}
	int productIdentifier() {return usbPID;}
	QString portName() {return productName;}
	QString manufacturer() {return manufacturerName;}
	QString libVersion() {return libraryVersion;}
	QByteArray serialNumber() {return serialNmb;}

private:
	bool FTDIdtr, FTDIrts;
	LineProperty FTDIlineProperty;
	FlowControl FTDIflowControl;
	PortErrors errFlag;
	uint32_t FTDIbaudRate = 57600;
	int usbVID, usbPID;
	unsigned int FTDIchipID;
	QString productName;
	QByteArray serialNmb;
	QString manufacturerName;
	QString libraryVersion;

	QMutex ftdiMutex;
	struct ftdi_context *ftdi;
	QByteArray FTDIreadBuffer;
	QSemaphore sem;

	QPointer<FT232Poll> worker;


public slots:
	void on_FTDIreceive(QByteArray data);
	void on_FTDIerror();
	void on_FTDImodemError(unsigned short modemStatus);
	void on_Pause(bool val) {worker->pause(val);}
	void close();

protected:
	qint64 readData(char * data, qint64 maxSize);
	qint64 writeData(const char *data, qint64 maxSize);

signals:
	void baudRateChanged(qint32);
    void linePropertyChanged(FT232::LineProperty);
    void flowControlChanged(FT232::FlowControl);
	void dataTerminalReadyChanged(bool);
	void requestToSendChanged(bool);
	void errorOccurred();
	void readyRead();
};


/* Info class
 * Similar to QSerialPortInfo class
 */
class FT232Info {

public:
	static QList<FT232Info> availablePorts(int VID = FTDI_VID, int PID = FTDI_PID);
	QString portName() {return FTDI_NAME;}
	QString description() {return descr;}
	QString manufacturer() {return manuf;}
	QString serialNumber() {return serialN;}
	quint16 vendorIdentifier() {return vid;}
	quint16 productIdentifier()  {return pid;}
	bool hasVendorIdentifier() {return true;}
	bool hasProductIdentifier() {return true;}

private:
	QString descr;
	QString manuf;
	QString serialN;
	quint16 vid;
	quint16 pid;
};


#endif // FT232_H

