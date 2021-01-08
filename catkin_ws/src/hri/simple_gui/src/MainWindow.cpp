#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    QIcon icoFwd(":/images/btnUp");
    QIcon icoBwd(":/images/btnDown");
    QIcon icoLeft(":/images/btnLeft");
    QIcon icoRight(":/images/btnRight");
    ui->btnFwd->setIcon(icoFwd);
    ui->btnBwd->setIcon(icoBwd);
    ui->btnLeft->setIcon(icoLeft);
    ui->btnRight->setIcon(icoRight);

    QObject::connect(ui->btnFwd, SIGNAL(pressed()), this, SLOT(btnFwdPressed()));
    QObject::connect(ui->btnFwd, SIGNAL(released()), this, SLOT(btnFwdReleased()));
    QObject::connect(ui->btnBwd, SIGNAL(pressed()), this, SLOT(btnBwdPressed()));
    QObject::connect(ui->btnBwd, SIGNAL(released()), this, SLOT(btnBwdReleased()));
    QObject::connect(ui->btnLeft, SIGNAL(pressed()), this, SLOT(btnLeftPressed()));
    QObject::connect(ui->btnLeft, SIGNAL(released()), this, SLOT(btnLeftReleased()));
    QObject::connect(ui->btnRight, SIGNAL(pressed()), this, SLOT(btnRightPressed()));
    QObject::connect(ui->btnRight, SIGNAL(released()), this, SLOT(btnRightReleased()));
    QObject::connect(ui->btnCmdVel, SIGNAL(pressed()), this, SLOT(btnCmdVelPressed()));
    QObject::connect(ui->btnCmdVel, SIGNAL(released()), this, SLOT(btnCmdVelReleased()));

    QObject::connect(ui->sbFlipperFL, SIGNAL(valueChanged(double)), this, SLOT(sbFlipperFLValueChanged(double)));
    QObject::connect(ui->sbFlipperFR, SIGNAL(valueChanged(double)), this, SLOT(sbFlipperFRValueChanged(double)));
    QObject::connect(ui->sbFlipperBL, SIGNAL(valueChanged(double)), this, SLOT(sbFlipperBLValueChanged(double)));
    QObject::connect(ui->sbFlipperBR, SIGNAL(valueChanged(double)), this, SLOT(sbFlipperBRValueChanged(double)));
    QObject::connect(ui->cbFlipperPositions, SIGNAL(currentTextChanged(QString)), this, SLOT(cbFlipperPositionsTextChanged(QString)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode)
{
    this->qtRosNode = qtRosNode;

    //Connect signals from QtRosNode to MainWindow
    //For example, when ros finishes or when a rostopic is received
    QObject::connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    QObject::connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsReceived()));
}

//
//SLOTS FOR SIGNALS EMITTED IN THE MAINWINDOW
//
void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    //event->accept();
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    //pmCamera.loadFromData(qtRosNode->imgCompressed.data(), qtRosNode->imgCompressed.size(), "JPG");
    //giCamera->setPixmap(pmCamera);
}

void MainWindow::btnFwdPressed()
{
    qtRosNode->start_publishing_cmd_vel(0.3, 0, 0);
}

void MainWindow::btnFwdReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::btnBwdPressed()
{
    qtRosNode->start_publishing_cmd_vel(-0.3, 0, 0);
}

void MainWindow::btnBwdReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::btnLeftPressed()
{
    qtRosNode->start_publishing_cmd_vel(0, 0, 0.5);
}

void MainWindow::btnLeftReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::btnRightPressed()
{
    qtRosNode->start_publishing_cmd_vel(0, 0, -0.5);
}

void MainWindow::btnRightReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::btnCmdVelPressed()
{
    std::stringstream ssLinearX(ui->txtLinearX->text().toStdString());
    std::stringstream ssAngular(ui->txtAngular->text().toStdString());
    float linearX = 0;
    float linearY = 0;
    float angular = 0;
    bool correct_format = true;
    if(!(ssLinearX >> linearX))
    {
        ui->txtLinearX->setText("Invalid format");
        correct_format = false;
    }
    if(!(ssAngular >> angular))
    {
        ui->txtAngular->setText("Invalid format");
        correct_format = false;
    }
    if(correct_format)
        qtRosNode->start_publishing_cmd_vel(linearX, linearY, angular);
}

void MainWindow::btnCmdVelReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::sbFlipperFLValueChanged(double d)
{
    qtRosNode->publish_flipper_fl(d);
}

void MainWindow::sbFlipperFRValueChanged(double d)
{
    qtRosNode->publish_flipper_fr(d);
}

void MainWindow::sbFlipperBLValueChanged(double d)
{
    qtRosNode->publish_flipper_bl(d);
}

void MainWindow::sbFlipperBRValueChanged(double d)
{
    qtRosNode->publish_flipper_br(d);
}

void MainWindow::cbFlipperPositionsTextChanged(const QString& text)
{
    qtRosNode->publish_flipper_predef_position(text.toStdString());
}
