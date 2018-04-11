# Contributing to the ATF
## Extend the ATF with your own metric
The following steps are needed to implement a new metrics in ATF:
### Python File
- Create new python-file for the metrics, using the following nameconvention:
```
calculate_*name*.py
```
- copy existing structure from one of the implemented metrics, looking like:
```python
class CalculatePublishRateParamHandler
    def parse_parameter(self, testblock_name, params):
class CalculatePublishRate:
    def __init__(self, groundtruth, groundtruth_epsilon):
    def start(self, timestamp):
    def stop(self, timestamp):  
    def pause(self, timestamp):
    def purge(self, timestamp):   
    def get_result(self):
```
  using the "publish\_rate"-metrics as an example. Replace "PublishRate" with the name of your newly generated metrics.
- In file ```atf/src/atf/atf_metrics/src/atf_metrics/__init__.py``` add:
```python
from atf_metrics.calculate_*name* import Calculate*Name*, Calculate*Name*ParamHandler
```
  e.g.
```python
from atf_metrics.calculate_jerk import CalculateJerk, CalculateJerkParamHandler
```
  here *name* stands for the name of your new metric (obviously).
  
- In file ```atf/src/atf/atf_metrics/config/metrics.yaml``` add:
```
*name*:
   handler: Calculate*Name*ParamHandler
```
  e.g.
```
jerk:
  handler: CalculateJerkParamHandler
```
### ATF Presenter
- In file ```atf/atf_presenter/html/js/atf_tools/test_list.js``` add (using "jerk" as an example):
```javascript
var plot_options = {
      jerk: {
        chart: {
          defaultSeriesType: 'column',
          type: 'column',
          zoomType: 'xy'
        },
        title: {
          text: 'Jerk'
        },
        yAxis: {
          title: {
            text: 'Jerk [m/s^3]'
          }
        },
        xAxis: {
          labels: {
            enabled: false
          }
        },
        plotOptions: {},
        tooltip: plot_tooltip
      },
};
```
  search the following if-statement:
```javascript
if ((metric_name == 'time') || (metric_name == 'path_length') || (metric_name == 'publish_rate') || (metric_name == 'interface') || (metric_name == 'jerk'))
```
  and add the new metrics as ```|| (metric_name == '*name*')```. In the following lines...
```javascript
if (metric_name == 'path_length') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['root_frame'] + " to " + metric_data['details']['measured_frame'] + ")"
if (metric_name == 'publish_rate') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['topic'] + ")"
if (metric_name == 'interface') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
if (metric_name == 'jerk') chart_legend_name = testblock_name + "<br>(" + metric_data['details']['topic'] + ")"
```
  add...
```javascript
if (metric_name == '*name*') chart_legend_name = testblock_name + "<br>(" + metric_data['details'] + ")"
```

To get additional information in the presenter. The "details" you store in the "metrics\_data" will be shown below the metrics-name in brackets.


# Setting up new ATF test
Generate a new ATF test following these steps:
1. generate project with the following structure:
```
test_project/
├── CMakeLists.txt
├── config
│   ├── robot_envs
│   │   └── env1.yaml
│   ├── robots
│   │   ├── robot1.yaml
│   │   └── robot2.yaml
│   ├── test_configs
│   │   ├── test1.yaml
│   │   └── test2.yaml
│   ├── test_generation_config.yaml     # specifies how to generate test files
│   └── test_suites.yaml                # specifies the combination of test_configs, robots and robot_envs.
├── launch
│   └── application.launch              # add all launch files that you want to run additionally to your robot bringup like moveit, etc...
├── package.xml
└── scripts
    └── application.py                  # send navigation goal to the robot
```

 - robot_envs:
   directory for environment specific settings, each environment is configured in one <robot_env_name>.yaml file
    (e.g. `env1.yaml`)
 - robots:
   directory for robot specific settings, each robot is configured in one <robot_name>.yaml file
    (e.g. `robot1.yaml`)
 - test_configs:
   directory for test configurations, each test is configured in one <test_config_name>.yaml file
   (e.g. `test1.yaml`)
 - `application.launch`:
   `launch`-file includes information about the ROS configs (e.g. 'saturn-ingolstadt' or 'ipa-apartment' environment config),
   and includes gazebo and RVIZ startup

2. config:
  - robot_envs:
```yaml
additional_parameters: {}                                # Name and value of additional parameter which will be included in every recording test file
additional_arguments: {}                                 # Name and value of additional arguments which will be included in every recording test file
```
  - robots:
```yaml
goal:                                          
  topics:                                             # Topics for the metric 'goal'
    - "/base_pose_ground_truth"
    - "/move_base/goal"
wait_for_topics: [/move_base/status]                  # Names of the topics to wait for before beginning the test
wait_for_services: []                                 # Names of the services to wait for before beginning the test
additional_parameters: []                             # Name and value of additional parameter which will be included in every recording test file
additional_arguments: []                              # Name and value of additional arguments which will be included in every recording test file

```
   In the `robot1.yaml` you can define topics which are included when the corresponding metric (e.g. "goal") is running
   (i.e. topics you need to **collect data** from for further calculations).

  - test_configs:
```yaml
testblock_nav:

  goal:
    - topic: /move_base/goal
      groundtruth: 0.2
      groundtruth_epsilon: 20

  jerk:
    - topic: /base/odometry_controller/odometry
      groundtruth: 0.0
      groundtruth_epsilon: 4.0

  time:
    - groundtruth: 6.0
      groundtruth_epsilon: 4.0

  publish_rate:
    - topic: /base/odometry_controller/odometry
      groundtruth: 33
      groundtruth_epsilon: 5
```
  In the `test1.yaml` you define the different metrics, the groundtruth and groundtruth epsilon for those metrics,
  and the topics (if necessary). All the chosen metrics are applied in *one* test-run of ATF.
  You can add additional parameters to extend the provided information for the metrics:
```yaml
testblock_nav:

  goal:
    - topic: /move_base/goal
      groundtruth_angle: 0.0 # [degree]
      groundtruth_angle_epsilon: 20.0 # [degree]
      groundtruth: 0.0 # [m]
      groundtruth_epsilon: 0.2 # [m]
```
  Those additional parameters have to be collected in the `metrics.py`-file using the following line:
```python
    def parse_parameter(self, testblock_name, params):

    [...]

        metrics.append(CalculateGoal(metric["topic"],
                                     metric["groundtruth_angle"],           # added in 'test.yaml' config in 'test_configs'
                                     metric["groundtruth_angle_epsilon"],   # added in 'test.yaml' config in 'test_configs'
                                     groundtruth,
                                     groundtruth_epsilon))
```

  | Name | Definition |
  |:------------:|:---------------:|
  | groundtruth |defines the expected value, example: 5 [m]
  | groundtruth_epsilon |defines max allowed +/- for the groundtruth, example: +/- 0,1 [m] |
  | topic |defines subscribed topic |

