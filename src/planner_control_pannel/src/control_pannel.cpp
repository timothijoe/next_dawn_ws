#include "control_pannel.h"
#include "ui_control_pannel.h"
#include <unistd.h>
ControlPannel::ControlPannel(QWidget *parent) :
  QMainWindow(parent),
  c_inter(new ComInterface),
  ui(new Ui::ControlPannel)
{
  ui->setupUi(this);
  this->setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);
  c_inter->init();
  customPlot = ui->wavePlotter;
  setupFigure(customPlot);
  customPlot->replot();
}

ControlPannel::~ControlPannel()
{
  delete ui;
}

void ControlPannel::setupFigure(QCustomPlot *customPlot)
{
        std::cout<<"Start setup figure!"<<std::endl;
        QCPGraph *graph1 = customPlot->addGraph();
        graph1->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, QPen(Qt::blue, 1.5), QBrush(Qt::white), 4));
        graph1->setPen(QPen(QColor(130, 130, 130), 2));

        QCPGraph *graph2 = customPlot->addGraph();
        graph2->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssTriangle, QPen(Qt::red, 1.5), QBrush(Qt::white), 4));
        graph2->setPen(QPen(QColor(130, 130, 130), 2));
//        graph2->setBrush(QColor(200, 200, 200, 20));
//        graph2->setChannelFillGraph(graph1);
//        customPlot->xAxis->grid()->setLayer("belowmain");
//        customPlot->yAxis->grid()->setLayer("belowmain");
        customPlot->xAxis->setBasePen(QPen(Qt::black, 1));
        customPlot->yAxis->setBasePen(QPen(Qt::black, 1));
        customPlot->xAxis->setTickPen(QPen(Qt::black, 1));
        customPlot->yAxis->setTickPen(QPen(Qt::black, 1));
        customPlot->xAxis->setSubTickPen(QPen(Qt::black, 1));
        customPlot->yAxis->setSubTickPen(QPen(Qt::black, 1));
        customPlot->xAxis->setTickLabelColor(Qt::black);
        customPlot->yAxis->setTickLabelColor(Qt::black);
        customPlot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
        customPlot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
        customPlot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
        customPlot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
        customPlot->xAxis->grid()->setSubGridVisible(true);
        customPlot->yAxis->grid()->setSubGridVisible(true);
        customPlot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
        customPlot->yAxis->grid()->setZeroLinePen(Qt::NoPen);
        customPlot->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
        customPlot->yAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
        QLinearGradient plotGradient;
        plotGradient.setStart(0, 0);
        plotGradient.setFinalStop(0, 350);
        plotGradient.setColorAt(0, QColor(80, 80, 80));
        plotGradient.setColorAt(1, QColor(50, 50, 50));
        //customPlot->setBackground(plotGradient);
        customPlot->setBackground(Qt::darkCyan);
        QLinearGradient axisRectGradient;
        axisRectGradient.setStart(0, 0);
        axisRectGradient.setFinalStop(0, 350);
        axisRectGradient.setColorAt(0, QColor(80, 80, 80));
        axisRectGradient.setColorAt(1, QColor(30, 30, 30));
        customPlot->axisRect()->setBackground(Qt::white);
        //customPlot->axisRect()->setBackground(Qt::darkGreen);
        //std::cout << "ge ming bi sheng" << std::endl;
        // pedCollision(int pedestrian_id)
        connect(c_inter, SIGNAL(figureUpdated(double, double)), this, SLOT(updateFigureView(double, double)));//Should be after qnode initialization
        connect(c_inter, SIGNAL(pedCollision(int)), this, SLOT(setupCollisionLight(int)));
        connect(c_inter, SIGNAL(taskCompleted(int)), this, SLOT(setupTaskCompletedLight(int)));
        connect(c_inter, SIGNAL(rosShutdown()), this, SLOT(close()));//Should be after qnode initialization
        customPlot->legend->setVisible(true);
}


void ControlPannel::updateFigureView(double data1, double data2)
{
  counter += 1;
  double data3 = data2 - 1000;
  customPlot->graph(0)->addData(counter, data1);//添加数据1到曲线1
  customPlot->graph(1)->addData(counter, data3);//添加数据2到曲线2
  customPlot->graph(0)->rescaleValueAxis(true);
  customPlot->graph(1)->rescaleValueAxis(true);
  customPlot->xAxis->setRange(counter+0.25, 120, Qt::AlignRight);//设定x轴的范围
  customPlot->replot();
}

void ControlPannel::setupCollisionLight(int times_)
{
  if(game_ok){
       ui->Llight->setStatebyNum(1);
//       c_inter->callStopService();
       game_ok = false;
       if(failed_times + success_times >= 40){
         on_system_stop_btn_clicked();
         return;
       }

       failed_times += 1;
       QString seed_str =  QString::number(failed_times);
       ui->textEdit_fail_times->setText(seed_str);
       double total_times = success_times + failed_times;
       seed_str =  QString::number(total_times);
       ui->textEdit_total_times->setText(seed_str);
       on_restRobotPositionBtn_clicked();
//       ui->Llight->setStatebyNum(0);
//       c_inter->callContinueService();

       game_ok = true;
  }

}

void ControlPannel::setupTaskCompletedLight(int times_ )
{
  if(game_ok){
    ui->Llight->setStatebyNum(2);
//    c_inter->callStopService();
    game_ok = false;
    if(failed_times + success_times >= 40){
      on_system_stop_btn_clicked();
      return;
    }
    success_times += 1;
    QString seed_str =  QString::number(success_times);
    ui->textEdit_succ_times->setText(seed_str);
    double total_times = success_times + failed_times;
    seed_str =  QString::number(total_times);
    ui->textEdit_total_times->setText(seed_str);
    on_restRobotPositionBtn_clicked();
//    ui->Llight->setStatebyNum(0);
//    c_inter->callContinueService();

    game_ok = true;
  }

}

void ControlPannel::on_defaultCheckBox_clicked()
{
  bool checked_label = ui->defaultCheckBox->isChecked();
  if(!checked_label){
    ui->x_spin_box->setEnabled(true);
    ui->y_spin_box->setEnabled(true);
    ui->theta_spin_box->setEnabled(true);
}
  else{
    ui->x_spin_box->setEnabled(false);
    ui->y_spin_box->setEnabled(false);
    ui->theta_spin_box->setEnabled(false);
  }
  CONFIRM_LABEL = false;
}

void ControlPannel::on_entroBtn_clicked()
{
    bool checked_label = ui->entroBtn->isChecked();
    if(checked_label){
      ui->compareList->setEnabled(true);
    }
    else{
      ui->compareList->setEnabled(false);
    }
    CONFIRM_LABEL = false;
}

void ControlPannel::on_testBtn_clicked()
{
//  test_num += 1;
//  test_num = test_num % 2;
//  if(test_num == 0){
//    ui->Llight->setStatebyNum(0);
//  }
//  else{
//    ui->Llight->setStatebyNum(1);
//  }
//  CONFIRM_LABEL = false;
  game_ok = true;
  ui->Llight->setStatebyNum(0);
}

void ControlPannel::on_conformBtn_clicked()
{
    CONFIRM_LABEL = true;
    int random_seed_num = ui->randSeedList->currentIndex();
    QString seed_str = "random seed:     " + QString::number(random_seed_num);
    ui->textEdit->setText(seed_str);
    QString task_str = "task: " + ui->metaTaskList->currentText();
    ui->textEdit->append(task_str);
    bool entro_calc_label = ui->entroBtn->isChecked();
    QString entro_str;
    if(!entro_calc_label){
      entro_str = "entro calc: disabled!";
    }
    else{
      entro_str = "entro calc: "+ ui->compareList->currentText();
    }
    ui->textEdit->append(entro_str);
}

void ControlPannel::on_exePlanBtn_clicked()
{
    //std::cout <<"ge ming bi sheng "<< std::endl;
    c_inter->callService();
}

void ControlPannel::on_stop_btn_clicked()
{
  if(STOP_LABEL == false){
    c_inter->callStopService();
    STOP_LABEL = true;
  }
  else{
    c_inter->callContinueService();
    STOP_LABEL = false;
    std::cout <<"ge ming bi sheng "<< std::endl;
  }
}

void ControlPannel::on_restart_btn_clicked()
{
  if(CONFIRM_LABEL == false){
    QString info_str = "Please click confirm button to confirm the parameter!";
    ui->textEdit->setText(info_str);
  }
  else{
    get_information();
    std::vector<double> robot_pos;
    r_x = 7;
    r_y = 4;
    r_theta = M_PI / 2;
    robot_pos.push_back(r_x);
    robot_pos.push_back(r_y);
    robot_pos.push_back(r_theta);
    c_inter->callRestartService(default_robot_label, random_seed_id, scenario_str, robot_pos);
    // c_inter->callRestartService(random_seed_id, scenario_str);
  }
}

void ControlPannel::on_compareList_activated(const QString &arg1)
{
    CONFIRM_LABEL = false;
}

void ControlPannel::on_randSeedList_activated(int index)
{
    CONFIRM_LABEL = false;
}

void ControlPannel::on_metaTaskList_activated(const QString &arg1)
{
  CONFIRM_LABEL = false;
}

void ControlPannel::get_information()
{
  default_robot_label = ui->defaultCheckBox->isChecked();
  if(!default_robot_label){
    r_x = ui->x_spin_box->value();
    r_y = ui->y_spin_box->value();
    r_theta = ui->theta_spin_box->value();
  }
  else{
    r_x = 0;
    r_y = 0;
    r_theta = 0;
  }

  entro_calc_label = ui->entroBtn->isChecked();
  if(entro_calc_label){
    evaluation_func_id = ui->compareList->currentIndex();
    evaluation_func_str = ui->compareList->currentText().toStdString();
  }
  else{
    evaluation_func_id = -1;
    evaluation_func_str = "Do Nothing";
  }

  random_seed_id = ui->randSeedList->currentIndex();
  task_id = ui->metaTaskList->currentIndex();
  task_str = ui->metaTaskList->currentText().toStdString();
  scenario_str = ui->sceneList->currentText().toStdString();
}



void ControlPannel::on_robot_stop_button_clicked()
{
    c_inter->setRobotStop();
}

void ControlPannel::on_robot_continue_button_clicked()
{
    c_inter->setRobotContinue();
}

void ControlPannel::on_restRobotPositionBtn_clicked()
{
  bool checked_label = ui->defaultCheckBox->isChecked();
  double robot_x_pos;
  double robot_y_pos;
  double robot_theta_pos;
  if(!checked_label){
    robot_x_pos = ui->x_spin_box->value();
    robot_y_pos = ui->y_spin_box->value();
    robot_theta_pos = ui->theta_spin_box->value();
   c_inter->setRobotPos(checked_label, robot_x_pos, robot_y_pos, robot_theta_pos);}
  else{
   c_inter->setRobotPos(checked_label, robot_x_pos, robot_y_pos, robot_theta_pos);

  }
}

void ControlPannel::on_system_stop_btn_clicked()
{
    //c_inter->setRobotStop();
    c_inter->callStopService();
}



void ControlPannel::on_system_continue_btn_clicked()
{
    //c_inter->setRobotContinue();
    c_inter->callContinueService();
}

void ControlPannel::on_reset_target_btn_clicked()
{
  double target_x_pos = ui->x_spin_box_2->value();
  double target_y_pos = ui->y_spin_box_2->value();
  double target_theta_pos = ui->theta_spin_box_2->value();
  c_inter->setTargetPos(target_x_pos, target_y_pos, target_theta_pos);

}
