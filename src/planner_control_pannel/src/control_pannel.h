#ifndef CONTROL_PANNEL_H
#define CONTROL_PANNEL_H

#include <QMainWindow>
#include "qcustomplot.h"
#include "com_interface.h"
#include <chrono>

namespace Ui {
class ControlPannel;
}

class ControlPannel : public QMainWindow
{
  Q_OBJECT

public:
  explicit ControlPannel(QWidget *parent = 0);
  ~ControlPannel();
  void setupFigure(QCustomPlot *customPlot);


private slots:
  void updateFigureView(double data1, double data2);

  void setupCollisionLight(int times_);

  void setupTaskCompletedLight(int times_);

  void on_defaultCheckBox_clicked();

  void on_entroBtn_clicked();

  void on_testBtn_clicked();

  void on_conformBtn_clicked();

  void on_exePlanBtn_clicked();

  void on_stop_btn_clicked();

  void on_restart_btn_clicked();

  void on_compareList_activated(const QString &arg1);

  void on_randSeedList_activated(int index);

  void on_metaTaskList_activated(const QString &arg1);

  void get_information();


  void on_robot_stop_button_clicked();

  void on_robot_continue_button_clicked();

  void on_restRobotPositionBtn_clicked();

  void on_system_stop_btn_clicked();

  void on_system_continue_btn_clicked();

  void on_reset_target_btn_clicked();

private:
  Ui::ControlPannel *ui;
  QCustomPlot *customPlot;
  ComInterface *c_inter;
  double counter;
  int test_num = 0;
  bool CONFIRM_LABEL = false;
  bool IGNORE_COLLISION_LABEL = false;
  bool STOP_LABEL = false;
  bool game_ok = true;


  //something need to call srv
  int random_seed_id;
  int task_id;

  int total_experiments_needed = 30;
  int success_times = 0;
  int failed_times = 0;
  int total_times = 0;
  std::vector<double> exec_time_list;
  std::vector<double> exec_traj_len_list;


  std::string task_str;
  std::string scenario_str;
  std::string evaluation_func_str;
  double r_x, r_y, r_theta;
  int evaluation_func_id; // 0 for compare and 1 for record
  bool entro_calc_label;
  bool default_robot_label;
};

#endif // CONTROL_PANNEL_H
