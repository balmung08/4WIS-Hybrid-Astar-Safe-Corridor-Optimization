#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSpinBox>
#include <QVariant>
#include <QWidget>

#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"

namespace fourwis_hybrid_astar_cpp
{

class StatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit StatusPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void updateStatus(const std::string & text);
  void updateMetrics(const std::string & text);
  void setMetricLabel(QLabel * label, double value, int precision = 2);

  QLabel * status_label_{nullptr};
  QLabel * time_{nullptr};
  QLabel * comfort_{nullptr};
  QLabel * length_{nullptr};
  QLabel * states_{nullptr};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metrics_sub_;
};

class PlannerParamPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PlannerParamPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  enum class EditorType { Bool, Int, Double, IntEnum, StringEnum, String };

  struct Option
  {
    QString label;
    QVariant value;
  };

  struct ParameterSpec
  {
    std::string name;
    QString label;
    QString group;
    EditorType type;
    double minimum{0.0};
    double maximum{1.0};
    double step{0.1};
    int decimals{2};
    std::vector<Option> options;
    bool map_node{false};
  };

  std::vector<ParameterSpec> parameterSpecs() const;
  QWidget * createEditor(const ParameterSpec & spec);
  QWidget * createSection(const QString & title, const std::vector<const ParameterSpec *> & specs);
  bool isMapParameter(const std::string & name) const;
  void applyParameterToEditor(const rclcpp::Parameter & parameter);
  void loadParameters();
  void setBoolParameter(const std::string & name, bool value);
  void setIntParameter(const std::string & name, int value);
  void setDoubleParameter(const std::string & name, double value);
  void setStringParameter(const std::string & name, const std::string & value);
  void setParameter(const rclcpp::Parameter & parameter);
  void setControlsEnabled(bool enabled);

  std::vector<ParameterSpec> specs_;
  std::unordered_map<std::string, QWidget *> editors_;
  QLabel * status_label_{nullptr};
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::AsyncParametersClient::SharedPtr map_parameters_client_;
  bool loading_{false};
};

}  // namespace fourwis_hybrid_astar_cpp
