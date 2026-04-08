#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <memory>

//#include "kalman.cpp"
#include "visualize.cpp"
//#include "structs.cpp"

#define QT true
#define CONTROL true

////////////////// параметры
//float CAMERA_WIDTH = 640;
//float CAMERA_FOV = 1.047;
float CAMERA_WIDTH = 640;
float CAMERA_FOV = 1.466;

char const *LIDAR_TOPIC = "/scan";
char const *CAMERA_TOPIC = "/camera/image";
//char const *CAMERA_TOPIC = "/front_camera/image_raw";
//char const *CAMERA_TOPIC = "/front_camera/image_raw/compressed";
char const *TWIST_TOPIC = "/cmd_vel";

float SHIFT[3] = {0, 0, 0.02};
float ROTATE[3][3] = {{0, 0, 0.02}, {0, 0, 0.02}, {0, 0, 0.02}};

float Kd = 0.8f; // по расстоянию
float Ka = 1.5f; // по углу

float max_linear = 0.5f; // скорость
float max_angular = 0.35f;

float desired_distance = 1.5f; // удерживаемое расстояние
float dist_deadband = 0.10f;   // мертвая зона расстояния
float angle_deadband = 0.05f;  // угла
/////////////////

struct Detection {
  bool found = false;
  cv::Rect bbox;
  cv::Point2f center;
  int cell_x = -1;
  int cell_y = -1;
};

//*
struct LidarPoint { //  в системе лидара
  float x = 0.0f;
  float y = 0.0f;
  float range = 0.0f;
  float angle = 0.0f;
};//*/

struct MotionCommand {
  float linear = 0.0f;  // линейная скорость
  float angular = 0.0f; // угловая скорость
};

struct TargetState {  // состояние цели
  bool valid = false; // найдена ли цель
  bool lost = true;
  float distance = 0.0f; // расстояние до цели
  float angle = 0.0f;    // угол до цели
  float rel_x = 0.0f; // вперед/назад относительно робота
  float rel_y = 0.0f; // вбок относительно робота
};

// режимы преследования
enum class FollowMode { SEARCH, ALIGN, FOLLOW, STOP, LOST };



class ImageViewer : public rclcpp::Node {
public:
  ImageViewer() : Node("image_viewer") {
    rclcpp::QoS qos(rclcpp::KeepLast(20));
    qos.best_effort();
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      CAMERA_TOPIC, qos,
      std::bind(&ImageViewer::onImage, this, std::placeholders::_1));
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      LIDAR_TOPIC, qos,
      std::bind(&ImageViewer::onScan, this, std::placeholders::_1));
    /*
      lidar_sub_.subscribe(this, LIDAR_TOPIC, qos.get_rmw_qos_profile());
    camera_sub_.subscribe(this, CAMERA_TOPIC, qos.get_rmw_qos_profile());
    sync_ =
        std::make_shared<Synchronizer>(SyncPolicy(20), lidar_sub_, camera_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(2));
    sync_->registerCallback(std::bind(&ImageViewer::fusionCallback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));*/
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(TWIST_TOPIC, 10);

#if QT == true
    cv::namedWindow("camera", cv::WINDOW_NORMAL);
    cv::namedWindow(win_, cv::WINDOW_AUTOSIZE);
#endif

    RCLCPP_INFO(get_logger(), "Subscribed to: %s", CAMERA_TOPIC);
  }

  ~ImageViewer() override { cv::destroyAllWindows(); }

private:
  size_t n_ = 0;
  //using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  //    sensor_msgs::msg::LaserScan, sensor_msgs::msg::Image>;
  //using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  //message_filters::Subscriber<sensor_msgs::msg::LaserScan> lidar_sub_;
  //message_filters::Subscriber<sensor_msgs::msg::Image> camera_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  //std::shared_ptr<Synchronizer> sync_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  //sensor_msgs::msg::Image::ConstSharedPtr last_image_;
  Detection last_detection;
  rclcpp::Time last_image_stamp_{0, 0, RCL_ROS_TIME};

  void  fusionCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &lidar_msg,
                 const sensor_msgs::msg::Image::ConstSharedPtr &camera_msg) {
    auto t0 = this->now();
    const double ts_scan = rclcpp::Time(lidar_msg->header.stamp).seconds();
    const double ts_img  = rclcpp::Time(camera_msg->header.stamp).seconds();
    const double dt = std::abs(ts_scan - ts_img);

    RCLCPP_INFO(this->get_logger(),
              "scan=%.6f image=%.6f dt=%.3f",
              ts_scan, ts_img, dt);
    
    RCLCPP_INFO(this->get_logger(), "onscan in");
    auto lidar_data = onScan(lidar_msg);
    RCLCPP_INFO(this->get_logger(), "onImage in");
    auto camera_data = onImage(camera_msg);
    RCLCPP_INFO(this->get_logger(), "processTogether in");
    processTogether(lidar_data, camera_data);
    auto dtf = (this->now() - t0).seconds();
    RCLCPP_INFO(this->get_logger(), "fusionCallback took %.3f s", dtf);
  }
  struct convertPoint { //  в системе лидара
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
  };
  void processTogether(std::vector<LidarPoint> lidar_data,
                       Detection camera_data) {
    // перевести точки лидара в систему координат камеры
    float t[3] = {0, 0, 0.02};
    std::vector<convertPoint> converted, box;
    converted.reserve(lidar_data.size());
    for (const auto &p : lidar_data) {
      convertPoint cp;
      cp.x = -p.y + t[1]; // вправо в камере
      cp.y = 0.0f + t[2]; // высота, грубое допущение
      cp.z = p.x + t[0];  // глубина вперёд
      converted.push_back(cp);
    }
    //RCLCPP_INFO(this->get_logger(), "processTogether in");
    RCLCPP_INFO(this->get_logger(), "check x %i, width %i", camera_data.bbox.x,
                camera_data.bbox.width);
    float W = CAMERA_WIDTH;
    float FOV = CAMERA_FOV;
    float fx = W / (2.0f * tan(FOV / 2.0f));
    float cx = W / 2.0f;
    // RCLCPP_INFO(this->get_logger(), "check 1");
    float u_left = camera_data.bbox.x;
    float u_right = camera_data.bbox.x + camera_data.bbox.width;
    // RCLCPP_INFO(this->get_logger(), "check 2");
    float theta_right = -atan2(u_left - cx, fx);
    float theta_left = -atan2(u_right - cx, fx);
    // RCLCPP_INFO(this->get_logger(), "check 3");
    if (theta_left > theta_right) {
      std::swap(theta_left, theta_right);
    }
    // RCLCPP_INFO(this->get_logger(), "check 4");
    /*
    if (!lidar_data.empty()){
      RCLCPP_INFO(this->get_logger(), "theta_left %f, lidarPoints %f",
    theta_left, lidar_data[0].angle); }else{ RCLCPP_INFO(this->get_logger(),
    "lidar_data is empty");
    }*/

    std::vector<LidarPoint> result;
    // RCLCPP_INFO(this->get_logger(), "select in");
    for (const auto &p : lidar_data) {
      float angle = p.angle; // возможно + offset

      if (angle >= theta_left && angle <= theta_right) {
        result.push_back(p);
      }
    }
    // RCLCPP_INFO(this->get_logger(), "cmd form in");
    if (!result.empty()) { // в result точки объекта. Далее нужно перенести в
                           // функцию
      float dist = result[0].range;
      // float mindistangle;
      for (const auto &p : result) {
        if (p.range < dist) {
          dist = p.range;
          // mindistangle = p.angle;
        }
      }
    }
    // RCLCPP_INFO(this->get_logger(), "минимальное расстояние %f", dist);
    // RCLCPP_INFO(this->get_logger(), "score in");
    TargetState state = score(result, true);

    //RCLCPP_INFO(this->get_logger(), "state %d", state.valid);
    FollowMode mode = decide(state);
    //RCLCPP_INFO(this->get_logger(), "статус %d", (int)mode);
    MotionCommand cmd = compute(state, mode);
    RCLCPP_INFO(this->get_logger(), "поворот %f", cmd.angular);
    RCLCPP_INFO(this->get_logger(), "линейная скорость %f", cmd.linear);
#if CONTROL == true
    sendCommand(cmd);
#endif
    // RCLCPP_INFO(this->get_logger(), "send out");

    // RCLCPP_INFO(this->get_logger(),
    //             "Получили синхронизированную пару и обработали её");
  }

  ///////////////////////// формирование команд

  FollowMode decide(const TargetState &target) const {
    if (!target.valid)
      return FollowMode::LOST;

    if (std::fabs(target.angle) > angle_deadband)
      return FollowMode::ALIGN;

    float dist_error = target.distance - desired_distance;

    if (std::fabs(dist_error) < dist_deadband)
      return FollowMode::STOP;

    return FollowMode::FOLLOW;
  }

  MotionCommand compute(const TargetState &target, FollowMode mode) const {
    MotionCommand cmd;
    //if (!target.valid) {
      // цель потеряна
      //cmd.linear = 0.0f;
      //cmd.angular = 0.0f;
      //return cmd;
    //}

    float dist_error = target.distance - desired_distance;
    float angle_error = target.angle;

    switch (mode) {
    case FollowMode::ALIGN:
      cmd.linear = 0.0f;
      cmd.angular = std::clamp(Ka * angle_error, -max_angular, max_angular);
      break;

    case FollowMode::FOLLOW:
      if (std::fabs(dist_error) >= dist_deadband)
        cmd.linear = std::clamp(Kd * dist_error, -max_linear, max_linear);

      if (std::fabs(angle_error) >= angle_deadband)
        cmd.angular = std::clamp(Ka * angle_error, -max_angular, max_angular);
      break;

    case FollowMode::STOP:
      cmd.linear = 0.0f;
      cmd.angular = 0.0f;
      break;

    case FollowMode::SEARCH:
    case FollowMode::LOST:
      cmd.linear = 0.0f;
      cmd.angular = 0.2f; // медленно крутиться искать цель
      break;
    }

    return cmd;
  }

  TargetState score(const std::vector<LidarPoint> &object_points,
                    bool camera_found) {
    TargetState state;

    if (!camera_found || object_points.empty()) {
      state.valid = false;
      state.lost = true;
      return state;
    }

    state.valid = true;
    state.lost = false;

    // можно взять ближайшую точку
    float min_range = object_points.front().range;
    float best_angle = object_points.front().angle;

    for (const auto &p : object_points) {
      if (p.range < min_range) {
        min_range = p.range;
        best_angle = p.angle;
      }
    }

    state.distance = min_range;
    state.angle = best_angle;

    return state;
  }

  void sendCommand(MotionCommand cmd) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = cmd.linear;
    msg.angular.z = cmd.angular;
    cmd_pub_->publish(msg);
  }

  ////////////////////////////////////////////////////////////////

  Detection onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    Detection defDetect;
    n_++;
    if (n_ % 30 == 0) {
      RCLCPP_INFO(get_logger(), "frames=%zu stamp=%u.%u encoding=%s", n_,
                  msg->header.stamp.sec, msg->header.stamp.nanosec,
                  msg->encoding.c_str());
    }
    try {

      cv_bridge::CvImageConstPtr cv_ptr;

      if (msg->encoding == "rgb8") {
        cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
        cv::Mat bgr;
        cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_RGB2BGR);

        // cv::imshow("camera", bgr);

        // ищем квадрат и клетку (например, сетка 8x6)
        auto det = detectRedSquareAndCell(bgr, 30, 30);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "maxAspect %.1f",
                             this->maxAspect);
        if (det.found) {
          cv::rectangle(bgr, det.bbox, cv::Scalar(0, 255, 0), 2);
          cv::circle(bgr, det.center, 3, cv::Scalar(255, 255, 255), -1);

          // подпись
          std::string txt = "cell=(" + std::to_string(det.cell_x) + "," +
                            std::to_string(det.cell_y) + ")";
          cv::putText(bgr, txt,
                      cv::Point(det.bbox.x, std::max(0, det.bbox.y - 8)),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

          // чтобы не спамить лог
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                               "Red square at px=(%.1f,%.1f) cell=(%d,%d)",
                               det.center.x, det.center.y, det.cell_x,
                               det.cell_y);

          cv::rectangle(bgr, det.bbox, cv::Scalar(0, 255, 0), 2);

#if QT == true
          cv::imshow("camera", bgr);
#endif
          last_detection = det;
          last_image_stamp_ = rclcpp::Time(msg->header.stamp);
          return det;
        }
      } else {

        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);

#if QT == true
        cv::imshow("camera", cv_ptr->image);
#endif
        last_detection = defDetect;
        last_image_stamp_ = rclcpp::Time(msg->header.stamp);
        return defDetect;
      }
#if QT == true
      cv::waitKey(1);
#endif
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return defDetect;
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
      return defDetect;
    }
    return defDetect;
  }

  Detection detectRedSquareAndCell(const cv::Mat &bgr, int grid_cols,
                                   int grid_rows) {
    Detection d;
    if (bgr.empty())
      return d;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // Красный: два диапазона Hue
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(0, 80, 80), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 80, 80), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    // Убираем шум
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    double bestScore = 0.0;
    cv::Rect bestRect;
    std::vector<cv::Point> bestApprox;
    double maxAspect = 0.;
    for (const auto &c : contours) {
      double area = cv::contourArea(c);
      if (area < 400.0)
        continue; // фильтр по площади (подстрой)

      cv::Rect r = cv::boundingRect(c);
      double aspect = (double)r.width / (double)r.height;
      if (aspect > maxAspect) {
        maxAspect = aspect;
      }
      if (aspect < 0.5 || aspect > 1.5)
        continue; // близко к квадрату

      // Аппроксимация контура -> квадрат обычно даёт 4 вершины
      //*
      std::vector<cv::Point> approx;
      double peri = cv::arcLength(c, true);
      cv::approxPolyDP(c, approx, 0.02 * peri, true);
      if ((int)approx.size() >= 8)
        continue;
      if (!cv::isContourConvex(approx)) // выпуклость
        continue;
      //*/
      // Скор: площадь * “квадратность”
      double fill = area / (double)(r.area() + 1);
      double score = area * fill;

      if (score > bestScore) {
        bestScore = score;
        bestRect = r;
        bestApprox = approx;
      }
    }
    this->maxAspect = maxAspect;
    // if (bestScore <= 0.0)
    //   return d;

    d.found = true;
    d.bbox = bestRect;
    d.center = cv::Point2f(bestRect.x + bestRect.width * 0.5f,
                           bestRect.y + bestRect.height * 0.5f);

    // Определяем “клетку” сетки grid_cols x grid_rows
    const int W = bgr.cols;
    const int H = bgr.rows;

    int cellW = std::max(1, W / grid_cols);
    int cellH = std::max(1, H / grid_rows);

    d.cell_x = std::clamp((int)(d.center.x / cellW), 0, grid_cols - 1);
    d.cell_y = std::clamp((int)(d.center.y / cellH), 0, grid_rows - 1);

    return d;
  }

  std::vector<LidarPoint>
  onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    std::vector<LidarPoint> points;
    // Картинка 600x600, центр — робот
    //if (!last_detection) {
    //  return points;
    //}
#if QT == true
    const int W = 600, H = 600;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(15, 15, 15));

    const cv::Point center(W / 2, H / 2);
    // Масштаб: сколько пикселей на метр (подстрой под range_max)
    const float meters_span = 12.0f; // “радиус” в метрах, который хотим видеть
    const float px_per_m = (std::min(W, H) * 0.45f) / meters_span;

    // сетка (опционально)
    drawGrid(img, center, px_per_m);
#endif

    // Собираем точки контура
    std::vector<cv::Point> poly;
    poly.reserve(msg->ranges.size());
    points.reserve(msg->ranges.size());

    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size();
         ++i, angle += msg->angle_increment) {
      float r = msg->ranges[i];
      if (!std::isfinite(r))
        continue;
      if (r < msg->range_min || r > msg->range_max)
        continue;

      float x = r * std::cos(angle);
      float y = r * std::sin(angle);

#if QT == true
      // экранные координаты: +x вправо, +y вверх (поэтому y инвертируем)
      int u = static_cast<int>(center.x + x * px_per_m);
      int v = static_cast<int>(center.y - y * px_per_m);

      // отсекаем, если за пределами
      if (u < 0 || u >= W || v < 0 || v >= H)
        continue;

      poly.emplace_back(u, v);
#endif

      points.push_back({x, y, r, angle});
    }
#if QT == true
    // Рисуем контур: полилиния + точки
    if (poly.size() >= 2) {
      cv::polylines(img, poly, false, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    for (const auto &p : poly) {
      cv::circle(img, p, 1, cv::Scalar(0, 200, 255), -1, cv::LINE_AA);
    }

    // Робот в центре
    cv::circle(img, center, 4, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);

    // Показ + обновление
    cv::imshow(win_, img);
    cv::waitKey(1);
#endif
    const auto scan_t = rclcpp::Time(msg->header.stamp);
    const double dtscan = std::abs((scan_t - last_image_stamp_).seconds());

    // допустимое окно подберите, например 0.15–0.30 c
    if (dtscan > 0.25) {
      return points;
    }
    auto t0 = this->now();
    processTogether(points, last_detection);
    auto dtf = (this->now() - t0).seconds();
    RCLCPP_INFO(this->get_logger(), "processTogether took %.3f s", dtf);
    return points;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::string win_ = "Lidar 2D (contour)";
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  double maxAspect = 0.;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<LidarViewer>());
  auto node = std::make_shared<ImageViewer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}