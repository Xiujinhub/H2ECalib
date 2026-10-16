#include "calib/IntrinsicCalibrator.h"
#include "calib/BoardImageCalibrator.h"
#include "calib/CoordinateTransformer.h"
#include <fstream>
#include <iomanip>
#include <vector>
#include <filesystem>
#ifdef _WIN32
#include <windows.h>
#endif

#define RUN_INTRINSIC 1  //相机内参标定
#define RUN_HAND_EYE_3D_BOARD_IMG 1 //手眼标定
namespace fs = std::filesystem;
struct Logger {
    static void info(const std::string &msg) {
        std::cout << "\033[1;36m[INFO] " << msg << "\033[0m" << std::endl;
    }

    static void success(const std::string &msg) {
        std::cout << "\033[1;32m[SUCCESS] " << msg << "\033[0m" << std::endl;
    }

    static void warn(const std::string &msg) {
        std::cout << "\033[1;33m[WARN] " << msg << "\033[0m" << std::endl;
    }

    static void error(const std::string &msg) {
        std::cerr << "\033[1;31m[ERROR] " << msg << "\033[0m" << std::endl;
    }

    static void section(const std::string &msg) {
        std::cout << "\n\033[1;35m"
                  << "============================================================="
                     "========"
                  << "\n   " << msg << "\n"
                  << "============================================================="
                     "========"
                  << "\033[0m" << std::endl;
    }
};

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    std::cout << std::fixed << std::setprecision(4);
    Logger::section("Industrial Camera + Robot Hand-Eye Calibration Program");
    std::cout << "Included: 2.5D Hand-Eye | 2D Four-Point | 2D Twelve-Point High "
                 "Precision Calibration\n"
              << std::endl;

#if RUN_INTRINSIC
    CalibConfig cfg;
    cfg.pattern = CHESSBOARD;
    cfg.cols = 8;
    cfg.rows = 11;
    cfg.interval_mm = 20; // Standard A4 print spacing
    cfg.calib_type = CalibrationType3D::EIH; // 标定类型，眼在手上：EIH，眼在手外：ETH

    std::string imgDir = "./assert/Checker-0910";
    if (!fs::exists(imgDir) || !fs::is_directory(imgDir)) {
        std::cerr << "错误: 目录 " << imgDir << " 不存在或不可访问！" << std::endl;
        std::cerr << "请在当前项目目录下创建 images/Checker 文件夹并放入标定图像。" << std::endl;
        return -1;
    }
    std::vector<std::string> intrinsic_img_files;
    for (const auto& entry : fs::directory_iterator(imgDir)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
                intrinsic_img_files.push_back(entry.path().string());
            }
        }
    }

    if (intrinsic_img_files.empty()) {
        std::cerr << "错误: images 目录下没有找到图像文件 (jpg/png/bmp)！" << std::endl;
        return -1;
    }

    sort(intrinsic_img_files.begin(), intrinsic_img_files.end());
    std::cout << "找到 " << intrinsic_img_files.size() << " 张图像。" << std::endl << std::endl;

    std::vector<std::string> extrinsic_img_files = intrinsic_img_files;

    Logger::section("Category 1: Intrinsic Calibration");

    IntrinsicCalibrator calib_obj;
    std::ifstream f_intrinsic(cfg.xml_intrinsic_1st);
    if (!calib_obj.intrinsicCalibrate(cfg, intrinsic_img_files)) {
        Logger::error("Intrinsic calibration failed!");
        return -1;
    }
    Logger::success("Intrinsic calibration success!");
#endif

#if RUN_HAND_EYE_3D_BOARD_IMG
    Logger::section(
        "Category 2: 3D Camera Hand-Eye Calibration - Based on Board Image");
    // CalibConfig cfg;
    // cfg.pattern = CHESSBOARD;
    // cfg.calib_type = CalibrationType3D::ETH;
    // cfg.cols = 7;
    // cfg.rows = 10;
    // cfg.interval_mm = 20; // Standard A4 print spacing
    // std::ifstream f_extrinsic(cfg.xml_extrinsic);
    // std::string imgDir = "./assert/Checker";
    // if (!fs::exists(imgDir) || !fs::is_directory(imgDir)) {
    //     std::cerr << "错误: 目录 " << imgDir << " 不存在或不可访问！" << std::endl;
    //     std::cerr << "请在当前项目目录下创建 images/Checker 文件夹并放入标定图像。" << std::endl;
    //     return -1;
    // }
    // std::vector<std::string> intrinsic_img_files;
    // for (const auto& entry : fs::directory_iterator(imgDir)) {
    //     if (entry.is_regular_file()) {
    //         std::string ext = entry.path().extension().string();
    //         transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    //         if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
    //             intrinsic_img_files.push_back(entry.path().string());
    //         }
    //     }
    // }

    // if (intrinsic_img_files.empty()) {
    //     std::cerr << "错误: images 目录下没有找到图像文件 (jpg/png/bmp)！" << std::endl;
    //     return -1;
    // }
    // std::vector<std::string> extrinsic_img_files = intrinsic_img_files;
    const std::vector<Pose3D> robot_poses = {

             {115.238, -340.780, 410.980, 190.96, -1.54, 115.45},
             {110.657, -175.770, 415.417, 198.36, 23.99, 108.80},
             {118.951, -294.655, 489.080, 191.35, 11.69, 107.74},
             {119.269, -343.982, 360.365, 188.83, -0.52, 115.61},
             {152.674, -154.429, 522.197, 191.50, 26.41, 109.23},
             {91.329, 76.653, 486.585, 210.01, 28.48, 140.53},
             {160.720, -67.945, 493.285, 175.20, 14.13, 111.87},
             {146.137, -221.958, 579.616, 172.94, 24.31, 104.13},
             {103.775, 80.734, 484.626, 202.21, 38.05, 106.40},
             {106.339, -72.747, 400.234, 193.65, 29.20, 96.21},
             {108.272, -153.359, 366.623, 193.81, 10.39, 109.05},
             {1.652, -268.889, 381.481, 172.02, 10.70, 86.21},
             {46.278, -190.686, 414.306, 179.89, 13.47, 104.49},
             {149.677, -103.968, 416.683, 195.50, 18.47, 139.38},
             {122.419, 16.217, 423.238, 199.89, 32.19, 114.15}
                                             /*{252.926, -401.932, -536.669, -83.512, 38.762, 28.218},
                                             {251.213, -386.262, -542.757, -82.352, 48.152, 39.276},
                                             {229.352, -374.528, -536.877, -86.592, 38.116, 17.081}*/};
    Logger::info("Starting hand-eye calibration...");
    BoardImageCalibrator board_calib_obj;
    board_calib_obj.setConfig(cfg);
    board_calib_obj.setImageFiles(extrinsic_img_files);
    board_calib_obj.setRobotPoses(robot_poses);
    board_calib_obj.setCalibrationType(cfg.calib_type);
    // 设置欧拉角类型 (示例: XYZ 外旋)
    board_calib_obj.setRPYType(RPY::RPYType::XYZ, RPY::ReferenceType::EXTRINSIC);

    if (!board_calib_obj.runCalibration()) {
        Logger::error("3D Hand-Eye Calibration failed!");
        return -1;
    }
    Logger::success("3D Hand-Eye Calibration success!");
    std::cout << "\n";
    Pose3D hand_eye_pose = board_calib_obj.getResultPose();
    auto errors = board_calib_obj.getErrors();
    auto markers = board_calib_obj.getMarkerPoints();
    auto marker_success = board_calib_obj.getMarkerSuccess();

    std::cout << "[INFO] Index | Robot(x,y,z,rx,ry,rz) | Marker(x,y,z) | Error\n";
    std::cout
        << "--------------------------------------------------------------\n";
    for (size_t i = 0; i < robot_poses.size(); ++i) {
        Pose3D p = robot_poses[i];
        std::cout << std::setw(3) << i << " | " << std::setw(10) << p.x() << ", "
                  << std::setw(10) << p.y() << ", " << std::setw(10) << p.z()
                  << ", " << std::setw(10) << p.rx() << ", " << std::setw(10)
                  << p.ry() << ", " << std::setw(10) << p.rz() << " | ";

        if (i < marker_success.size() && marker_success[i]) {
            std::cout << std::setw(10) << markers[i].x() << ", " << std::setw(10)
            << markers[i].y() << ", " << std::setw(10) << markers[i].z();
        } else {
            std::cout << std::setw(10) << "N/A" << ", " << std::setw(10) << "N/A"
                      << ", " << std::setw(10) << "N/A";
        }

        std::cout << " | ";
        if (i < errors.size() && errors[i] >= 0)
            std::cout << std::setw(10) << errors[i];
        else
            std::cout << std::setw(10) << "N/A";
        std::cout << "\n";
    }

    double errorSum = 0.0;
    int errCount = 0;
    for (auto error : errors) {
        if (error >= 0) {
            errorSum += error;
            errCount++;
        }
    }
    double avg = errCount == 0 ? 0 : errorSum / errCount;

    double maxE = 0, minE = 0;
    bool first = true;
    for (auto error : errors) {
        if (error < 0)
            continue;
        if (first) {
            maxE = minE = error;
            first = false;
        } else {
            maxE = (std::max)(maxE, error);
            minE = (std::min)(minE, error);
        }
    }

    Logger::success(
        "3D Hand-Eye Calibration successful! Avg Error: " + std::to_string(avg) +
        ", Max: " + std::to_string(maxE) + ", Min: " + std::to_string(minE));

    std::cout << "[INFO] hand_eyes Homogeneous Matrix (Eigen format):\n"
              << hand_eye_pose.toTransform3D().eigen() << "\n"
              << std::endl;

    std::cout << "[RESULT] hand_eyes (Pose3D format): \n"
              << hand_eye_pose.x() << ", " << hand_eye_pose.y() << ", "
              << hand_eye_pose.z() << ", " << hand_eye_pose.rx() << ", "
              << hand_eye_pose.ry() << ", " << hand_eye_pose.rz() << std::endl;

    cv::Mat H_hand_eyes =
        CoordinateTransformer::toHomogeneousMatrix(hand_eye_pose);
    cv::Mat H_cam_to_gripper_R = H_hand_eyes(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat H_cam_to_gripper_t = H_hand_eyes(cv::Rect(3, 0, 1, 3)).clone();

    Logger::info("Hand-eye calibration results (R | t) saved to: " +
                 cfg.xml_extrinsic);

    cv::FileStorage fs_hand_eye(cfg.xml_extrinsic, cv::FileStorage::WRITE);
    fs_hand_eye << "R" << H_cam_to_gripper_R << "t" << H_cam_to_gripper_t;
    fs_hand_eye.release();
    Logger::success("3D Hand-Eye Calibration complete!");
#endif
    Logger::section("All Calibrations Complete!");

    return 0;
}
