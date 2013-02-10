#include "Common.hpp"
#include "MainWindow.hpp"
#include <fcntl.h>
#include <unistd.h>
//#include <sys/ioctl.h>
#include <linux/input.h>

MainWindow::MainWindow(QWidget* _parent)
  :QMainWindow(_parent),
    m_settings(new Settings()),
    m_ui(new Ui::MainWindow()),
    m_copterCtrl(),
    m_tcpServer(),
    m_tcpConnection(),
    m_accelerometerInputFd(-1),
    m_accelerometerInputNotifier(0),
    m_lastTiltX(0),
    m_lastTiltY(0)
{
  m_ui->setupUi(this);
  const auto s_ctrl_path = m_settings->getControlPath();
  QSharedPointer<CopterMotor> mx1(new CopterMotor(m_settings, s_ctrl_path+"ehrpwm.0/pwm/ehrpwm.0:0/duty_percent", m_ui->motor_x1));
  QSharedPointer<CopterMotor> mx2(new CopterMotor(m_settings, s_ctrl_path+"ehrpwm.0/pwm/ehrpwm.0:1/duty_percent", m_ui->motor_x2));
  QSharedPointer<CopterMotor> my1(new CopterMotor(m_settings, s_ctrl_path+"ehrpwm.1/pwm/ehrpwm.1:0/duty_percent", m_ui->motor_y1));
  QSharedPointer<CopterMotor> my2(new CopterMotor(m_settings, s_ctrl_path+"ehrpwm.1/pwm/ehrpwm.1:1/duty_percent", m_ui->motor_y2));
  QSharedPointer<CopterAxis>  m_axisX(new CopterAxis(mx1, mx2));
  QSharedPointer<CopterAxis>  m_axisY(new CopterAxis(my1, my2));
  m_copterCtrl = new CopterCtrl(m_settings, m_axisX, m_axisY, m_ui->motor_all);

  m_tcpServer.listen(QHostAddress::Any, m_settings->getTcpPort());
  connect(&m_tcpServer, SIGNAL(newConnection()), this, SLOT(onConnection()));

  auto const s_accel_input_path = m_settings->getAccelInputPath();
  m_accelerometerInputFd = ::open(s_accel_input_path.toLatin1().data(), O_SYNC, O_RDONLY);
  if (m_accelerometerInputFd == -1)
    qDebug() << "Cannot open accelerometer input file " << s_accel_input_path << ", reason: " << errno;

  m_accelerometerInputNotifier = new QSocketNotifier(m_accelerometerInputFd, QSocketNotifier::Read, this);
  connect(m_accelerometerInputNotifier, SIGNAL(activated(int)), this, SLOT(onAccelerometerRead()));
  m_accelerometerInputNotifier->setEnabled(true);

  m_copterCtrl->adjustPower(0);

  showFullScreen();
  showMaximized();
}

void MainWindow::onConnection()
{
  if (!m_tcpConnection.isNull())
    qDebug() << "Replacing existing connection";
  m_tcpConnection = m_tcpServer.nextPendingConnection();
  qDebug() << "Accepted new connection";
  m_tcpConnection->setSocketOption(QAbstractSocket::LowDelayOption, 1);
  connect(m_tcpConnection, SIGNAL(disconnected()), this, SLOT(onDisconnected()));
  connect(m_tcpConnection, SIGNAL(readyRead()), this, SLOT(onNetworkRead()));
}

void MainWindow::onDisconnected()
{
  qDebug() << "Existing connection disconnected";
  m_tcpConnection = 0;
}

void MainWindow::onNetworkRead()
{
  if (m_tcpConnection.isNull())
    return;

  static const auto s_tilt_step = m_settings->getTiltStep();
  static const auto s_power_max = m_settings->getPowerMax();
  //static const auto s_power_min = m_settings->getPowerMin();
  static const auto s_power_step1 = m_settings->getPowerStep1();
  static const auto s_power_step2 = m_settings->getPowerStep2();

  while (m_tcpConnection->isReadable())
  {
    char c;
    if (!m_tcpConnection->getChar(&c))
      break;
    switch (c)
    {
      case '1': m_copterCtrl->adjustTilt(-s_tilt_step, -s_tilt_step); break;
      case '2': m_copterCtrl->adjustTilt(0,            -s_tilt_step); break;
      case '3': m_copterCtrl->adjustTilt(+s_tilt_step, -s_tilt_step); break;
      case '4': m_copterCtrl->adjustTilt(-s_tilt_step, 0); break;
      case '5': m_copterCtrl->tiltX(0); m_copterCtrl->tiltY(0); break;
      case '6': m_copterCtrl->adjustTilt(+s_tilt_step, 0); break;
      case '7': m_copterCtrl->adjustTilt(-s_tilt_step, +s_tilt_step); break;
      case '8': m_copterCtrl->adjustTilt(0,            +s_tilt_step); break;
      case '9': m_copterCtrl->adjustTilt(+s_tilt_step, +s_tilt_step); break;
      case 'Z': m_copterCtrl->adjustPower(-s_power_max); break;
      case 'z': m_copterCtrl->adjustPower(-s_power_step2); break;
      case 'x': m_copterCtrl->adjustPower(-s_power_step1); break;
      case 'c': m_copterCtrl->adjustPower(+s_power_step1); break;
      case 'v': m_copterCtrl->adjustPower(+s_power_step2); break;
      case 'V': m_copterCtrl->adjustPower(+s_power_max); break;
    }
  }
}

void MainWindow::onAccelerometerRead()
{
  struct input_event evt;

  if (read(m_accelerometerInputFd, reinterpret_cast<char*>(&evt), sizeof(evt)) != sizeof(evt))
  {
    qDebug() << "Incomplete accelerometer data read";
    return;
  }

  if (evt.type != EV_ABS)
  {
    if (evt.type != EV_SYN)
      qDebug() << "Input event type is not EV_ABS or EV_SYN: " << evt.type;
    return;
  }

  char code = 0;
  switch (evt.code)
  {
    case ABS_X: code = 'x'; handleTiltX(evt.value); break;
    case ABS_Y: code = 'y'; handleTiltY(evt.value); break;
    case ABS_Z: code = 'z'; break;
  }
  if (code == 0 || m_tcpConnection.isNull())
    return;

  QString buf;
  buf.sprintf("%c%i ", code, static_cast<int>(evt.value));
  m_tcpConnection->write(buf.toAscii());
}

void MainWindow::handleTiltX(double _tilt)
{
  static const auto s_accel_linear = m_settings->getAccelLinear();
  static const auto s_accel_derivative = m_settings->getAccelDerivative();
  double adj = s_accel_linear*_tilt + s_accel_derivative*(_tilt - m_lastTiltX);
  m_copterCtrl->adjustTilt(adj, 0);
  m_lastTiltX = _tilt;
}

void MainWindow::handleTiltY(double _tilt)
{
  static const auto s_accel_linear = m_settings->getAccelLinear();
  static const auto s_accel_derivative = m_settings->getAccelDerivative();

  double adj = s_accel_linear*_tilt + s_accel_derivative*(_tilt - m_lastTiltY);
  m_copterCtrl->adjustTilt(0, adj);
  m_lastTiltY = _tilt;
}



