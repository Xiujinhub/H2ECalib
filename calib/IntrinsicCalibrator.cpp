#include "calib/IntrinsicCalibrator.h"
#include <iostream>

int IntrinsicCalibrator::detectCalibBoard(const cv::Mat &input_image,
                                       cv::Mat &output_corners_image,
                                       std::vector<cv::Point2f> &pixel_corners,
                                       std::vector<cv::Point3f> &world_corners,
                                       const CalibConfig &cfg, int calib_type) {
  pixel_corners.clear();
  world_corners.clear();

  if (input_image.empty() || cfg.cols <= 0 || cfg.rows <= 0 ||
      cfg.interval_mm <= 0) {
    return -1;
  }

  cv::Mat gray;
  if (input_image.channels() != 1) {
    cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = input_image;
  }

  std::vector<cv::Point2f> all_pixel_corners;
  bool found = false;

  if (cfg.pattern == CHESSBOARD) {
    found = cv::findChessboardCorners(
        gray, cv::Size(cfg.cols, cfg.rows), all_pixel_corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (found)
      cv::cornerSubPix(
          gray, all_pixel_corners, cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                           0.001));
  } else if (cfg.pattern == CIRCLES_SYM) {
    found = cv::findCirclesGrid(gray, cv::Size(cfg.cols, cfg.rows),
                                all_pixel_corners, cv::CALIB_CB_SYMMETRIC_GRID);
  } else {
    cv::SimpleBlobDetector::Params params;
    params.blobColor = 255;
    params.filterByColor = true;
    params.maxArea = 50000;
    params.minArea = 1200;
    params.filterByArea = true;

    params.minDistBetweenBlobs = 36;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5;
    params.minConvexity = 0.8;
    params.thresholdStep = 20;
    cv::Ptr<cv::SimpleBlobDetector> blobDetector =
        cv::SimpleBlobDetector::create(params);
    cv::CirclesGridFinderParameters paramss;
    paramss.gridType =
        cv::CirclesGridFinderParameters::GridType::ASYMMETRIC_GRID;
    paramss.minDistanceToAddKeypoint = 45;
    paramss.edgeGain = 10;
    paramss.edgePenalty = 10;

    found = cv::findCirclesGrid(
        gray, cv::Size(cfg.cols, cfg.rows), all_pixel_corners,
        cv::CALIB_CB_CLUSTERING | cv::CALIB_CB_ASYMMETRIC_GRID, blobDetector,
        paramss);
  }

  cv::cvtColor(gray, output_corners_image, cv::COLOR_GRAY2RGB);
  cv::drawChessboardCorners(output_corners_image, cv::Size(cfg.cols, cfg.rows),
                            all_pixel_corners, found);

  if (!found)
    return 0;

  std::vector<cv::Point3f> all_world_corners;
  if (cfg.pattern == CHESSBOARD || cfg.pattern == CIRCLES_SYM) {
    for (int r = 0; r < cfg.rows; ++r)
      for (int c = 0; c < cfg.cols; ++c)
        all_world_corners.emplace_back(c * cfg.interval_mm, r * cfg.interval_mm,
                                       0);
  } else {
    for (int i = 0; i < cfg.rows; ++i) {
      if (i % 2 == 0) {
        for (int j = 0; j < cfg.cols; ++j)
          all_world_corners.push_back(cv::Point3f(
              j * cfg.interval_mm, (i * cfg.interval_mm) / 2.0f, 0));
      } else {
        for (int j = 0; j < cfg.cols; ++j)
          all_world_corners.push_back(
              cv::Point3f(j * cfg.interval_mm + cfg.interval_mm / 2.0f,
                          (i * cfg.interval_mm) / 2.0f, 0));
      }
    }
  }

  if (calib_type == 0) {
    for (size_t i = 0; i < all_pixel_corners.size(); ++i) {
      auto last_num = i % 4;
      auto div_num = (i - last_num) / 4;
      if (div_num != 1 && div_num != 3) {
        pixel_corners.push_back(all_pixel_corners[i]);
        world_corners.push_back(all_world_corners[i]);
      }
    }
  } else if (calib_type == 1) {
    pixel_corners = all_pixel_corners;
    world_corners = all_world_corners;
  } else {
    return -1;
  }

  return 1;
}

/**
 * @brief 可视化棋盘格角点（支持检测成功/失败）
 * @param img 灰度图像（会转换为BGR显示）
 * @param corners 检测到的角点坐标（如果成功）
 * @param boardSize 标定板尺寸
 * @param success 是否检测成功
 * @param windowName 显示窗口名称
 * @return 按键返回值（用于交互）
 */
int IntrinsicCalibrator::visualizeChessboardCorners(const cv::Mat& img, const std::vector<cv::Point2f>& corners, cv::Size boardSize,
                               bool success, const std::string& windowName) {
    cv::Mat displayImg;
    cvtColor(img, displayImg, cv::COLOR_GRAY2BGR);
    if (success) {
        drawChessboardCorners(displayImg, boardSize, corners, true);
        putText(displayImg, "Corners: " + std::to_string(corners.size()) + " / " + std::to_string(boardSize.width * boardSize.height),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    } else {
        putText(displayImg, "Detection FAILED!",
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    imshow(windowName, displayImg);
    return cv::waitKey(0);
}

bool IntrinsicCalibrator::intrinsicCalibrate(const CalibConfig &cfg,
                                             const std::vector<std::string> imgs) {
    // 存储每张图片的预处理结果
    struct ImgResult {
        cv::Mat displayImg;                     // 带角点绘制的显示图像
        std::vector<cv::Point2f> pixel_corners; // 过滤后的像素角点
        std::vector<cv::Point3f> world_corners; // 对应的世界坐标
        bool success;                           // 是否检测成功
    };
    std::vector<ImgResult> results;
    cv::Size imgSize;
    int valid = 0;

    // 第一遍：读取所有图片并检测角点（不进行标定）
    for (size_t i = 0; i < imgs.size(); ++i) {
        cv::Mat raw = cv::imread(imgs[i]);
        if (raw.empty()) {
            std::cerr << "Warning: cannot read image " << imgs[i] << std::endl;
            continue;
        }
        if (imgSize == cv::Size())
            imgSize = raw.size();

        std::vector<cv::Point2f> pixel_corners;
        std::vector<cv::Point3f> world_corners;
        cv::Mat output_corners_image;
        int ret = detectCalibBoard(raw, output_corners_image, pixel_corners,
                                   world_corners, cfg, 0); // calib_type=0

        ImgResult res;
        res.displayImg = output_corners_image.clone();
        res.pixel_corners = pixel_corners;
        res.world_corners = world_corners;
        res.success = (ret == 1);
        results.push_back(res);

        if (res.success) valid++;
        std::cout << "\r预处理进度: " << i + 1 << "/" << imgs.size()
                  << "  有效: " << valid << std::flush;
    }
    std::cout << "\n预处理完成，有效图片数: " << valid << std::endl;

    if (results.empty()) {
        std::cerr << "没有可用的图片！" << std::endl;
        return false;
    }

    // 可视化浏览模式
    const std::string winName = "Visualize";
    cv::namedWindow(winName, cv::WINDOW_NORMAL);
    int currentIdx = 0;
    bool browseMode = true;

    while (browseMode) {
        // 准备显示图像（添加文字信息）
        cv::Mat display = results[currentIdx].displayImg.clone();
        cv::imshow(winName, display);

        int key = cv::waitKey(0);
        if (key == 'a' || key == 'A') {
            currentIdx = (currentIdx - 1 + results.size()) % results.size();
        } else if (key == 'd' || key == 'D') {
            currentIdx = (currentIdx + 1) % results.size();
        } else if (key == ' ') {   // 空格键
            browseMode = false;
        } else if (key == 27) {    // ESC 可选退出
            //browseMode = false;
            std::cout << "用户取消标定。" << std::endl;
            cv::destroyWindow(winName);
            return false;
        }
    }
    cv::destroyWindow(winName);

    // 收集所有成功检测的数据用于标定
    std::vector<std::vector<cv::Point3f>> worlds_list_first;
    std::vector<std::vector<cv::Point2f>> pixels_list_first;
    valid = 0;
    for (const auto& res : results) {
        if (res.success) {
            worlds_list_first.push_back(res.world_corners);
            pixels_list_first.push_back(res.pixel_corners);
            valid++;
        }
    }

    if (valid < 5) {
        std::cerr << "至少需要 5 张有效图片，当前只有 " << valid << " 张！" << std::endl;
        return false;
    }

    // 执行标定
    cv::Mat K_first, dis_first;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(worlds_list_first, pixels_list_first, imgSize,
                                     K_first, dis_first, rvecs, tvecs, 0);
    std::cout << "第一次标定成功！RMS = " << rms << "\n";
    std::cout << "内参: \n" << K_first << "\n";
    std::cout << "畸变: \n" << dis_first << "\n";

    cv::FileStorage fs(cfg.xml_intrinsic_1st, cv::FileStorage::WRITE);
    fs << "K" << K_first << "distortion" << dis_first << "pattern"
       << (int)cfg.pattern;
    fs.release();
    return true;
}
