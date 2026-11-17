// Underwater camera post-processing for the Gazebo rgbd_camera.
//
// Gazebo (OGRE 2) renders the scene as if in air, so the water looks perfectly
// clear. This node takes the RGB and depth images of the rgbd_camera and adds,
// per pixel, the effects of the water between the camera and the object:
//
//   1. Attenuation + backscatter (Beer-Lambert, per RGB channel):
//        I = I0 * t + B * (1 - t),   t = exp(-eta * d)
//      d - distance along the camera ray [m], eta - attenuation [1/m],
//      B - colour of the water itself. Red fades much faster than green/blue.
//   2. Forward scattering: blur that grows with distance.
//   3. Sensor noise.
//
// Input/output topics and all coefficients come from params/underwater_camera.yaml.

#include <cmath>
#include <iostream>
#include <mutex>
#include <string>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace {

struct Config {
  std::string rgb_topic, depth_topic, info_topic, output_topic;
  cv::Scalar eta;          // attenuation [1/m], RGB order
  cv::Scalar backscatter;  // water colour [0-255], RGB order
  bool blur_enabled;
  int blur_kernel;         // Gaussian kernel size [px], made odd
  float blur_blend;        // max blur weight, reached at blur_distance
  float blur_distance;     // [m]; the weight grows linearly from 0 at the camera
  bool noise_enabled;
  float noise_stddev;      // [0-255]
};

// Throws YAML::Exception if the file or a key is missing.
Config LoadConfig(const std::string &path) {
  const YAML::Node y = YAML::LoadFile(path);
  auto rgb = [](const YAML::Node &n) {
    return cv::Scalar(n["r"].as<double>(), n["g"].as<double>(), n["b"].as<double>());
  };
  Config c;
  c.rgb_topic = y["topics"]["input_rgb_topic"].as<std::string>();
  c.depth_topic = y["topics"]["input_depth_topic"].as<std::string>();
  c.info_topic = y["topics"]["input_camera_info_topic"].as<std::string>();
  c.output_topic = y["topics"]["output_image_topic"].as<std::string>();
  c.eta = rgb(y["attenuation"]);
  c.backscatter = rgb(y["backscatter"]);
  c.blur_enabled = y["blur"]["enabled"].as<bool>();
  c.blur_kernel = y["blur"]["max_kernel_size"].as<int>() | 1;
  c.blur_blend = y["blur"]["blend_factor"].as<float>();
  c.blur_distance = y["blur"]["full_blur_distance"].as<float>();
  c.noise_enabled = y["noise"]["enabled"].as<bool>();
  c.noise_stddev = y["noise"]["stddev"].as<float>();
  return c;
}

class UnderwaterCamera {
 public:
  explicit UnderwaterCamera(Config cfg) : cfg_(std::move(cfg)) {}

  bool Start() {
    pub_ = node_.Advertise<gz::msgs::Image>(cfg_.output_topic);
    return pub_ &&
           node_.Subscribe(cfg_.info_topic, &UnderwaterCamera::OnCameraInfo, this) &&
           node_.Subscribe(cfg_.depth_topic, &UnderwaterCamera::OnDepth, this) &&
           node_.Subscribe(cfg_.rgb_topic, &UnderwaterCamera::OnRgb, this);
  }

 private:
  // Precomputes, from the camera intrinsics, the factor that turns planar depth
  // into distance along each pixel's ray: sqrt(1 + x^2 + y^2) in normalised
  // image coordinates. Recomputed only if the intrinsics change.
  void OnCameraInfo(const gz::msgs::CameraInfo &msg) {
    if (msg.intrinsics().k_size() != 9) return;
    const auto &k = msg.intrinsics().k();
    const cv::Vec4d intrinsics(k[0], k[4], k[2], k[5]);  // fx, fy, cx, cy
    const cv::Size size(msg.width(), msg.height());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (intrinsics == intrinsics_ && size == ray_scale_.size()) return;
    }
    cv::Mat scale(size, CV_32FC1);
    for (int r = 0; r < size.height; ++r)
      for (int c = 0; c < size.width; ++c)
        scale.at<float>(r, c) = std::hypot(1.0, (c - intrinsics[2]) / intrinsics[0],
                                           (r - intrinsics[3]) / intrinsics[1]);
    std::lock_guard<std::mutex> lock(mutex_);
    intrinsics_ = intrinsics;
    ray_scale_ = scale;
  }

  // Depth arrives as float32 planar depth (distance along the optical axis) [m].
  void OnDepth(const gz::msgs::Image &msg) {
    const size_t bytes = size_t(msg.width()) * msg.height() * sizeof(float);
    if (msg.width() == 0 || msg.data().size() < bytes) return;
    cv::Mat depth(msg.height(), msg.width(), CV_32FC1,
                  const_cast<char *>(msg.data().data()));
    std::lock_guard<std::mutex> lock(mutex_);
    depth.copyTo(depth_);
  }

  void OnRgb(const gz::msgs::Image &msg) {
    const auto format = msg.pixel_format_type();
    if (format != gz::msgs::RGB_INT8 && format != gz::msgs::BGR_INT8) {
      std::cerr << "Unsupported pixel format: " << format << std::endl;
      return;
    }
    cv::Mat image(msg.height(), msg.width(), CV_8UC3,
                  const_cast<char *>(msg.data().data()), msg.step());
    if (image.empty()) return;

    cv::Mat depth, ray_scale;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      depth = depth_.clone();
      ray_scale = ray_scale_;  // never modified in place, sharing is safe
    }
    // Wait for the first depth and camera_info; rgbd_camera images all share one size.
    if (depth.size() != image.size() || ray_scale.size() != image.size()) return;

    // Work in float RGB; convert back (with saturation) only at the end.
    cv::Mat img;
    image.convertTo(img, CV_32FC3);
    if (format == gz::msgs::BGR_INT8) cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    const cv::Mat dist = RayDistance(depth, ray_scale);
    cv::Mat dist3;
    cv::merge(std::vector<cv::Mat>(3, dist), dist3);

    // 1. Attenuation + backscatter: I = B + (I0 - B) * t
    cv::Mat t;
    cv::multiply(dist3, -cfg_.eta, t);
    cv::exp(t, t);
    cv::subtract(img, cfg_.backscatter, img);
    img = img.mul(t);
    cv::add(img, cfg_.backscatter, img);

    // 2. Forward scattering: blend towards a blurred copy, more with distance.
    if (cfg_.blur_enabled && cfg_.blur_kernel > 1) {
      cv::Mat blurred, weight;
      cv::GaussianBlur(img, blurred, cv::Size(cfg_.blur_kernel, cfg_.blur_kernel), 0);
      cv::min(dist3 * (cfg_.blur_blend / cfg_.blur_distance), cfg_.blur_blend, weight);
      img += (blurred - img).mul(weight);
    }

    // 3. Sensor noise.
    if (cfg_.noise_enabled && cfg_.noise_stddev > 0.0f) {
      cv::Mat noise(img.size(), CV_32FC3);
      cv::randn(noise, 0.0, cfg_.noise_stddev);
      img += noise;
    }

    gz::msgs::Image out;
    *out.mutable_header() = msg.header();
    out.set_width(img.cols);
    out.set_height(img.rows);
    out.set_pixel_format_type(gz::msgs::RGB_INT8);
    out.set_step(img.cols * 3);
    cv::Mat out_img(img.size(), CV_8UC3);
    img.convertTo(out_img, CV_8UC3);
    out.set_data(out_img.data, out_img.total() * out_img.elemSize());
    pub_.Publish(out);
  }

  // Planar depth -> distance along each pixel's ray [m]. The camera's own <clip>
  // range (model.sdf) decides what is visible: Gazebo returns +inf (or NaN) when
  // nothing is within <far>, and -inf for surfaces closer than <near>.
  //  - +inf/NaN: only open water on that ray; a distance large enough that
  //    exp(-eta * d) == 0, so the output is the pure water colour.
  //  - -inf: object right in front of the lens, i.e. no water in between.
  static cv::Mat RayDistance(cv::Mat depth, const cv::Mat &ray_scale) {
    constexpr float kOpenWater = 1e4f;
    cv::patchNaNs(depth, kOpenWater);
    depth = cv::min(cv::max(depth, 0.0f), kOpenWater);
    return depth.mul(ray_scale);
  }

  Config cfg_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
  std::mutex mutex_;     // guards the three members below
  cv::Mat depth_;        // latest depth frame
  cv::Mat ray_scale_;    // see OnCameraInfo
  cv::Vec4d intrinsics_;
};

}  // namespace

int main(int argc, char **argv) {
  const std::string config_path = argc > 1 ? argv[1] : "params/underwater_camera.yaml";

  Config cfg;
  try {
    cfg = LoadConfig(config_path);
  } catch (const YAML::Exception &e) {
    std::cerr << "Cannot load " << config_path << ": " << e.what() << std::endl;
    return 1;
  }

  UnderwaterCamera camera(cfg);
  if (!camera.Start()) {
    std::cerr << "Failed to set up Gazebo topics" << std::endl;
    return 1;
  }
  std::cout << cfg.rgb_topic << " + " << cfg.depth_topic << " -> " << cfg.output_topic
            << std::endl;
  gz::transport::waitForShutdown();
  return 0;
}
