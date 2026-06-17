#include "fourwis_hybrid_astar_cpp/status_panel.hpp"

#include <QFrame>
#include <QFormLayout>
#include <QGridLayout>
#include <QLineEdit>
#include <QMetaObject>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QToolButton>
#include <QVBoxLayout>

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace fourwis_hybrid_astar_cpp
{

StatusPanel::StatusPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setStyleSheet(
    "QWidget { background-color: #f7f8fa; color: #111827; }"
    "QLabel#Title { font-size: 18px; font-weight: 700; }"
    "QLabel#Section { font-size: 11px; font-weight: 700; color: #4b5563; letter-spacing: 0px; }"
    "QLabel#State { background: #fff7d6; border: 1px solid #f0d46a; border-radius: 4px;"
    " padding: 8px; font-size: 20px; font-weight: 700; color: #7a4a00; }"
    "QFrame#MetricBox { background: #ffffff; border: 1px solid #d1d5db; border-radius: 4px; }"
    "QLabel#MetricHead { font-size: 11px; font-weight: 700; color: #374151; }"
    "QLabel#MetricName { font-size: 12px; font-weight: 700; color: #111827; }"
    "QLabel#MetricValue { font-family: Monospace; font-size: 13px; color: #000; }");

  auto * layout = new QVBoxLayout;
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(8);

  auto * title = new QLabel("Ackermann Planner");
  title->setObjectName("Title");
  layout->addWidget(title);

  auto * status_title = new QLabel("State");
  status_title->setObjectName("Section");
  layout->addWidget(status_title);

  status_label_ = new QLabel("WAITING MAP");
  status_label_->setObjectName("State");
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  auto * divider = new QFrame;
  divider->setFrameShape(QFrame::HLine);
  divider->setFrameShadow(QFrame::Sunken);
  layout->addWidget(divider);

  auto * metrics_title = new QLabel("Metrics");
  metrics_title->setObjectName("Section");
  layout->addWidget(metrics_title);

  auto * metric_box = new QFrame;
  metric_box->setObjectName("MetricBox");
  auto * metric_grid = new QGridLayout;
  metric_grid->setContentsMargins(8, 8, 8, 8);
  metric_grid->setHorizontalSpacing(12);
  metric_grid->setVerticalSpacing(6);
  auto addName = [&](int row, const QString & text) {
    auto * label = new QLabel(text);
    label->setObjectName("MetricName");
    metric_grid->addWidget(label, row, 0);
  };
  auto makeValue = [&]() {
    auto * label = new QLabel("--");
    label->setObjectName("MetricValue");
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    label->setMinimumWidth(78);
    return label;
  };
  addName(0, "Time [s]");
  addName(1, "Comfort");
  addName(2, "Length [m]");
  addName(3, "States");
  time_ = makeValue();
  comfort_ = makeValue();
  length_ = makeValue();
  states_ = makeValue();
  metric_grid->addWidget(time_, 0, 1);
  metric_grid->addWidget(comfort_, 1, 1);
  metric_grid->addWidget(length_, 2, 1);
  metric_grid->addWidget(states_, 3, 1);
  metric_box->setLayout(metric_grid);
  layout->addWidget(metric_box);

  layout->addStretch();

  setLayout(layout);
}

void StatusPanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    return;
  }
  auto node = node_abstraction->get_raw_node();
  state_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/fourwis_planner_state", rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      updateStatus(msg->data);
    });
  metrics_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/fourwis_metrics_text", rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      updateMetrics(msg->data);
    });
}

void StatusPanel::updateStatus(const std::string & text)
{
  QMetaObject::invokeMethod(
    this,
    [this, text]() {
      status_label_->setText(QString::fromStdString(text));
    },
    Qt::QueuedConnection);
}

void StatusPanel::updateMetrics(const std::string & text)
{
  QMetaObject::invokeMethod(
    this,
    [this, text]() {
      std::unordered_map<std::string, std::string> values;
      std::stringstream stream(text);
      std::string item;
      while (std::getline(stream, item, ';')) {
        const std::size_t eq = item.find('=');
        if (eq != std::string::npos) {
          values[item.substr(0, eq)] = item.substr(eq + 1);
        }
      }
      auto get = [&](const std::string & key, double fallback = 0.0) {
        const auto found = values.find(key);
        if (found == values.end()) {
          return fallback;
        }
        try {
          return std::stod(found->second);
        } catch (...) {
          return fallback;
        }
      };
      setMetricLabel(time_, get("time"));
      setMetricLabel(comfort_, get("comfort"));
      setMetricLabel(length_, get("length"));
      setMetricLabel(states_, get("states"), 0);
    },
    Qt::QueuedConnection);
}

void StatusPanel::setMetricLabel(QLabel * label, double value, int precision)
{
  if (!label) {
    return;
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  label->setText(QString::fromStdString(stream.str()));
}

PlannerParamPanel::PlannerParamPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  specs_ = parameterSpecs();
  setStyleSheet(
    "QWidget { background-color: #f7f8fa; color: #111827; }"
    "QLabel { color: #374151; }"
    "QLabel#ParamTitle { font-size: 18px; font-weight: 700; color: #111827; }"
    "QLabel#ParamStatus { font-size: 12px; color: #4b5563; padding-top: 4px; }"
    "QToolButton#SectionButton { background: #ffffff; border: 1px solid #d1d5db;"
    " border-radius: 4px; padding: 7px; font-weight: 700; color: #111827; }"
    "QWidget#SectionContent { background: #ffffff; border-left: 1px solid #d1d5db;"
    " border-right: 1px solid #d1d5db; border-bottom: 1px solid #d1d5db;"
    " border-bottom-left-radius: 4px; border-bottom-right-radius: 4px; }");

  auto * root = new QVBoxLayout;
  root->setContentsMargins(10, 10, 10, 10);
  root->setSpacing(8);

  auto * title = new QLabel("Planner Parameters");
  title->setObjectName("ParamTitle");
  root->addWidget(title);

  auto * scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto * content = new QWidget;
  auto * content_layout = new QVBoxLayout;
  content_layout->setContentsMargins(0, 0, 0, 0);
  content_layout->setSpacing(6);

  const std::vector<QString> group_order = {
    "Demo Map", "Input & Occupancy", "Search Grid", "Goal & Analytic",
    "Vehicle Geometry", "Vehicle Limits", "Motion Sampling", "Front-end Costs",
    "Backend Solver", "Backend Corridor", "Backend Objective & Bounds", "Visualization"};
  for (const auto & group : group_order) {
    std::vector<const ParameterSpec *> group_specs;
    for (const auto & spec : specs_) {
      if (spec.group == group) {
        group_specs.push_back(&spec);
      }
    }
    if (!group_specs.empty()) {
      content_layout->addWidget(createSection(group, group_specs));
    }
  }
  content_layout->addStretch();
  content->setLayout(content_layout);
  scroll->setWidget(content);
  root->addWidget(scroll, 1);

  status_label_ = new QLabel("Waiting for planner node");
  status_label_->setObjectName("ParamStatus");
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);

  setLayout(root);
  setControlsEnabled(false);
}

std::vector<PlannerParamPanel::ParameterSpec> PlannerParamPanel::parameterSpecs() const
{
  const std::vector<Option> map_scenarios = {
    {"legacy_maze", QString("legacy_maze")},
    {"legacy_maze_inflated_0_1", QString("legacy_maze_inflated_0_1")},
    {"legacy_parking", QString("legacy_parking")},
    {"reference_parking", QString("reference_parking")},
    {"tight_complex", QString("tight_complex")}};
  return {
    {"map_scenario", "Map scenario", "Demo Map", EditorType::StringEnum, 0.0, 0.0, 0.0, 0, map_scenarios, true},

    {"obstacle_threshold", "Obstacle threshold", "Input & Occupancy", EditorType::Int, 0, 100, 1, 0, {}},
    {"unknown_is_obstacle", "Unknown is obstacle", "Input & Occupancy", EditorType::Bool, 0.0, 0.0, 0.0, 0, {}},

    {"xy_resolution", "XY resolution", "Search Grid", EditorType::Double, 0.02, 2.0, 0.02, 3, {}},
    {"yaw_resolution", "Yaw resolution [rad]", "Search Grid", EditorType::Double, 0.01, 3.14, 0.01, 4, {}},
    {"heuristic_resolution", "Heuristic resolution", "Search Grid", EditorType::Double, 0.05, 2.0, 0.05, 3, {}},
    {"max_iterations", "Max search iterations", "Search Grid", EditorType::Int, 1000, 1000000, 10000, 0, {}},

    {"goal_distance_tolerance", "Goal distance tolerance", "Goal & Analytic", EditorType::Double, 0.01, 5.0, 0.05, 3, {}},
    {"goal_yaw_tolerance", "Goal yaw tolerance [rad]", "Goal & Analytic", EditorType::Double, 0.01, 3.14, 0.01, 4, {}},
    {"analytic_expansion_distance", "Analytic expansion distance", "Goal & Analytic", EditorType::Double, 0.0, 30.0, 0.5, 2, {}},

    {"wheelbase", "Wheelbase", "Vehicle Geometry", EditorType::Double, 0.05, 5.0, 0.05, 3, {}},
    {"robot_length", "Robot length", "Vehicle Geometry", EditorType::Double, 0.05, 10.0, 0.05, 3, {}},
    {"robot_width", "Robot width", "Vehicle Geometry", EditorType::Double, 0.05, 10.0, 0.05, 3, {}},
    {"collision_clearance", "Collision clearance", "Vehicle Geometry", EditorType::Double, 0.0, 2.0, 0.01, 3, {}},

    {"reference_velocity", "Reference velocity", "Vehicle Limits", EditorType::Double, 0.05, 5.0, 0.05, 3, {}},
    {"max_steer_deg", "Max steer [deg]", "Vehicle Limits", EditorType::Double, 0.0, 90.0, 1.0, 2, {}},

    {"sampling_time", "Sampling time", "Motion Sampling", EditorType::Double, 0.05, 5.0, 0.05, 3, {}},
    {"integration_dt", "Integration dt", "Motion Sampling", EditorType::Double, 0.001, 1.0, 0.005, 4, {}},

    {"reverse_penalty", "Reverse penalty", "Front-end Costs", EditorType::Double, 0.0, 100.0, 0.1, 3, {}},
    {"steer_penalty", "Steer penalty", "Front-end Costs", EditorType::Double, 0.0, 100.0, 0.1, 3, {}},
    {"steer_change_penalty", "Steer change penalty", "Front-end Costs", EditorType::Double, 0.0, 100.0, 0.1, 3, {}},
    {"direction_change_penalty", "Direction change penalty", "Front-end Costs", EditorType::Double, 0.0, 100.0, 0.1, 3, {}},

    {"backend_casadi_python", "CasADi Python", "Backend Solver", EditorType::String, 0.0, 0.0, 0.0, 0, {}},
    {"backend_casadi_script", "CasADi script", "Backend Solver", EditorType::String, 0.0, 0.0, 0.0, 0, {}},
    {"backend_max_iterations", "Backend outer iterations", "Backend Solver", EditorType::Int, 1, 50, 1, 0, {}},
    {"backend_ipopt_max_iterations", "IPOPT max iterations", "Backend Solver", EditorType::Int, 1, 5000, 10, 0, {}},
    {"backend_ipopt_tol", "IPOPT tolerance", "Backend Solver", EditorType::Double, 0.000000000001, 0.1, 0.000001, 12, {}},

    {"backend_resample_distance", "Resample distance", "Backend Corridor", EditorType::Double, 0.02, 2.0, 0.02, 3, {}},
    {"backend_corridor_max_distance", "Corridor max distance", "Backend Corridor", EditorType::Double, 0.05, 10.0, 0.05, 3, {}},
    {"backend_corridor_fast_step", "Corridor fast step", "Backend Corridor", EditorType::Double, 0.01, 2.0, 0.01, 3, {}},
    {"backend_corridor_fine_step", "Corridor fine step", "Backend Corridor", EditorType::Double, 0.001, 1.0, 0.005, 4, {}},
    {"backend_corridor_axis_aligned", "Axis-aligned corridors", "Backend Corridor", EditorType::Bool, 0.0, 0.0, 0.0, 0, {}},

    {"backend_comfort_weight", "Comfort weight", "Backend Objective & Bounds", EditorType::Double, 0.0, 1000.0, 0.05, 4, {}},
    {"backend_constraint_penalty", "Constraint penalty", "Backend Objective & Bounds", EditorType::Double, 0.0, 100000000.0, 1000.0, 1, {}},
    {"backend_infeasibility_tolerance", "Infeasibility tolerance", "Backend Objective & Bounds", EditorType::Double, 0.000000000001, 0.1, 0.000001, 12, {}},
    {"backend_max_velocity", "Backend max velocity", "Backend Objective & Bounds", EditorType::Double, 0.05, 10.0, 0.05, 3, {}},
    {"backend_max_acceleration", "Backend max acceleration", "Backend Objective & Bounds", EditorType::Double, 0.05, 20.0, 0.05, 3, {}},

    {"pose_arrow_length", "Pose arrow length", "Visualization", EditorType::Double, 0.05, 5.0, 0.05, 3, {}},
    {"pose_arrow_width", "Pose arrow width", "Visualization", EditorType::Double, 0.01, 2.0, 0.01, 3, {}},
    {"pose_arrow_height", "Pose arrow height", "Visualization", EditorType::Double, 0.01, 2.0, 0.01, 3, {}},
    {"pose_marker_z", "Pose marker z", "Visualization", EditorType::Double, 0.0, 2.0, 0.01, 3, {}},
    {"body_trajectory_visualization_enabled", "Show body trajectory", "Visualization", EditorType::Bool, 0.0, 0.0, 0.0, 0, {}},
    {"body_trajectory_marker_z", "Body trajectory z", "Visualization", EditorType::Double, 0.0, 2.0, 0.01, 3, {}},
    {"backend_corridor_visualization_enabled", "Show safe corridors", "Visualization", EditorType::Bool, 0.0, 0.0, 0.0, 0, {}},
    {"backend_corridor_visualization_stride", "Corridor stride", "Visualization", EditorType::Int, 1, 200, 1, 0, {}},
    {"backend_corridor_marker_z", "Corridor marker z", "Visualization", EditorType::Double, 0.0, 2.0, 0.01, 3, {}},
  };
}

QWidget * PlannerParamPanel::createEditor(const ParameterSpec & spec)
{
  if (spec.type == EditorType::Bool) {
    auto * editor = new QCheckBox;
    connect(editor, &QCheckBox::toggled, this, [this, name = spec.name](bool value) {
      if (!loading_) {
        setBoolParameter(name, value);
      }
    });
    return editor;
  }
  if (spec.type == EditorType::Int) {
    auto * editor = new QSpinBox;
    editor->setRange(static_cast<int>(spec.minimum), static_cast<int>(spec.maximum));
    editor->setSingleStep(static_cast<int>(spec.step));
    connect(editor, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, name = spec.name](int value) {
      if (!loading_) {
        setIntParameter(name, value);
      }
    });
    return editor;
  }
  if (spec.type == EditorType::Double) {
    auto * editor = new QDoubleSpinBox;
    editor->setRange(spec.minimum, spec.maximum);
    editor->setSingleStep(spec.step);
    editor->setDecimals(spec.decimals);
    connect(editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, name = spec.name](double value) {
      if (!loading_) {
        setDoubleParameter(name, value);
      }
    });
    return editor;
  }
  if (spec.type == EditorType::String) {
    auto * editor = new QLineEdit;
    connect(editor, &QLineEdit::editingFinished, this, [this, name = spec.name, editor]() {
      if (!loading_) {
        setStringParameter(name, editor->text().toStdString());
      }
    });
    return editor;
  }

  auto * editor = new QComboBox;
  for (const auto & option : spec.options) {
    editor->addItem(option.label, option.value);
  }
  connect(editor, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this, name = spec.name, type = spec.type, editor](int index) {
    if (loading_ || index < 0) {
      return;
    }
    if (type == EditorType::IntEnum) {
      setIntParameter(name, editor->itemData(index).toInt());
    } else {
      setStringParameter(name, editor->itemData(index).toString().toStdString());
    }
  });
  return editor;
}

QWidget * PlannerParamPanel::createSection(
  const QString & title, const std::vector<const ParameterSpec *> & specs)
{
  auto * section = new QWidget;
  auto * section_layout = new QVBoxLayout;
  section_layout->setContentsMargins(0, 0, 0, 0);
  section_layout->setSpacing(0);

  auto * button = new QToolButton;
  button->setObjectName("SectionButton");
  button->setText(title);
  button->setCheckable(true);
  button->setChecked(title == "Search Grid" || title == "Backend Solver");
  button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  button->setArrowType(button->isChecked() ? Qt::DownArrow : Qt::RightArrow);
  button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  auto * content = new QWidget;
  content->setObjectName("SectionContent");
  auto * form = new QFormLayout;
  form->setContentsMargins(8, 8, 8, 8);
  form->setSpacing(6);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  for (const auto * spec : specs) {
    auto * editor = createEditor(*spec);
    editors_[spec->name] = editor;
    form->addRow(spec->label, editor);
  }
  content->setLayout(form);
  content->setVisible(button->isChecked());
  connect(button, &QToolButton::toggled, this, [button, content](bool checked) {
    button->setArrowType(checked ? Qt::DownArrow : Qt::RightArrow);
    content->setVisible(checked);
  });

  section_layout->addWidget(button);
  section_layout->addWidget(content);
  section->setLayout(section_layout);
  return section;
}

void PlannerParamPanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    return;
  }
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_abstraction->get_raw_node(), "/fourwis_hybrid_astar_cpp");
  map_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_abstraction->get_raw_node(), "/fourwis_demo_map_cpp");
  loadParameters();
}

void PlannerParamPanel::loadParameters()
{
  if (!parameters_client_) {
    return;
  }
  std::vector<std::string> names;
  std::vector<std::string> map_names;
  names.reserve(specs_.size());
  for (const auto & spec : specs_) {
    if (spec.map_node) {
      map_names.push_back(spec.name);
    } else {
      names.push_back(spec.name);
    }
  }
  auto apply_params = [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      const auto params = future.get();
      QMetaObject::invokeMethod(
        this,
        [this, params]() {
          loading_ = true;
          setControlsEnabled(false);
          for (const auto & param : params) {
            applyParameterToEditor(param);
          }
          loading_ = false;
          setControlsEnabled(true);
          status_label_->setText("Parameters loaded. Changes apply immediately to the running node.");
        },
        Qt::QueuedConnection);
    };
  if (!names.empty()) {
    parameters_client_->get_parameters(names, apply_params);
  }
  if (map_parameters_client_ && !map_names.empty()) {
    map_parameters_client_->get_parameters(map_names, apply_params);
  }
}

bool PlannerParamPanel::isMapParameter(const std::string & name) const
{
  for (const auto & spec : specs_) {
    if (spec.name == name) {
      return spec.map_node;
    }
  }
  return false;
}

void PlannerParamPanel::applyParameterToEditor(const rclcpp::Parameter & parameter)
{
  const auto found = editors_.find(parameter.get_name());
  if (found == editors_.end()) {
    return;
  }
  QWidget * widget = found->second;
  QSignalBlocker block(widget);
  if (auto * editor = qobject_cast<QCheckBox *>(widget)) {
    editor->setChecked(parameter.as_bool());
  } else if (auto * editor = qobject_cast<QSpinBox *>(widget)) {
    editor->setValue(static_cast<int>(parameter.as_int()));
  } else if (auto * editor = qobject_cast<QDoubleSpinBox *>(widget)) {
    editor->setValue(parameter.as_double());
  } else if (auto * editor = qobject_cast<QLineEdit *>(widget)) {
    editor->setText(QString::fromStdString(parameter.as_string()));
  } else if (auto * editor = qobject_cast<QComboBox *>(widget)) {
    int index = -1;
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      index = editor->findData(static_cast<int>(parameter.as_int()));
    } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      index = editor->findData(QString::fromStdString(parameter.as_string()));
    }
    if (index >= 0) {
      editor->setCurrentIndex(index);
    }
  }
}

void PlannerParamPanel::setBoolParameter(const std::string & name, bool value)
{
  setParameter(rclcpp::Parameter(name, value));
}

void PlannerParamPanel::setIntParameter(const std::string & name, int value)
{
  setParameter(rclcpp::Parameter(name, value));
}

void PlannerParamPanel::setDoubleParameter(const std::string & name, double value)
{
  setParameter(rclcpp::Parameter(name, value));
}

void PlannerParamPanel::setStringParameter(const std::string & name, const std::string & value)
{
  setParameter(rclcpp::Parameter(name, value));
}

void PlannerParamPanel::setParameter(const rclcpp::Parameter & parameter)
{
  auto client = isMapParameter(parameter.get_name()) ? map_parameters_client_ : parameters_client_;
  if (!client) {
    return;
  }
  status_label_->setText(QString::fromStdString("Setting " + parameter.get_name() + "..."));
  client->set_parameters(
    {parameter},
    [this, name = parameter.get_name()](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      const bool ok = !results.empty() && results.front().successful;
      const std::string reason = !results.empty() ? results.front().reason : "no response";
      QMetaObject::invokeMethod(
        this,
        [this, name, ok, reason]() {
          status_label_->setText(QString::fromStdString(ok ? "Applied: " + name : "Rejected: " + reason));
        },
        Qt::QueuedConnection);
    });
}

void PlannerParamPanel::setControlsEnabled(bool enabled)
{
  for (auto & item : editors_) {
    item.second->setEnabled(enabled);
  }
}

}  // namespace fourwis_hybrid_astar_cpp

PLUGINLIB_EXPORT_CLASS(fourwis_hybrid_astar_cpp::StatusPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(fourwis_hybrid_astar_cpp::PlannerParamPanel, rviz_common::Panel)
