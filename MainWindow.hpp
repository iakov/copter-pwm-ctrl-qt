#ifndef MAIN_WINDOW_H_
#define MAIN_WINDOW_H_
#include "Common.hpp"
#include "Settings.hpp"
#include "ui_MainWindow.h"
#include "CopterCtrl.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT
  private:
    Ui::MainWindow* m_ui;
    Settings::sptr m_settings;

  public:
    MainWindow(QWidget* _parent = 0);

  protected:
    QPointer<CopterCtrl> m_copterCtrl;
    QTcpServer           m_tcpServer;
    QPointer<QTcpSocket> m_tcpConnection;
    int                  m_accelerometerInputFd;
    QPointer<QSocketNotifier> m_accelerometerInputNotifier;

    void handleTiltX(double _tilt);
    void handleTiltY(double _tilt);
    double m_lastTiltX;
    double m_lastTiltY;

  protected slots:
    void onConnection();
    void onDisconnected();
    void onNetworkRead();
    void onAccelerometerRead();

};


#endif // !MAIN_WINDOW_H_
