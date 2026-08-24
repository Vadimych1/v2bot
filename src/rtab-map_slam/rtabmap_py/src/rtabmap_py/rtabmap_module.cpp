#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Signature.h>

#include <opencv2/core.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

namespace py = pybind11;

cv::Mat convertGrid(cv::Mat grid, uint8_t unknown = 127, uint8_t free = 255, uint8_t occupied = 0)
{
    cv::Mat image(grid.size(), CV_8UC1);

    for (int y = 0; y < grid.rows; ++y)
    {
        for (int x = 0; x < grid.cols; ++x)
        {
            int value = grid.at<int8_t>(y, x);

            if (value < 0)
            {
                image.at<uint8_t>(y, x) = unknown;
            }
            else if (value == 0)
            {
                image.at<uint8_t>(y, x) = free;
            }
            else
            {
                image.at<uint8_t>(y, x) = occupied;
            }
        }
    }

    return image;
}

class RtabmapSLAM
{
public:
    RtabmapSLAM(float lidarMinRange, float lidarMaxRange, float resolution)
    {
        parameters.insert(
            rtabmap::ParametersPair(
                "Grid/Sensor",
                "0"));

        parameters.insert(
            rtabmap::ParametersPair(
                "Grid/RangeMax",
                std::to_string(lidarMaxRange)));

        parameters.insert(
            rtabmap::ParametersPair(
                "Grid/RangeMin",
                std::to_string(lidarMinRange)));

        parameters.insert(
            rtabmap::ParametersPair(
                "Grid/CellSize",
                std::to_string(resolution)));

        parameters.insert(
            rtabmap::ParametersPair(
                "GridGlobal/OccupancyThr",
                "0.5"));

        parameters.insert(
            rtabmap::ParametersPair(
                "Grid/3D",
                "false"));

        parameters.insert(
            rtabmap::ParametersPair(
                "RGBD/CreateOccupancyGrid",
                "true"));

        rtabmap_.init(parameters);

        _lidarMaxRange = lidarMinRange;
        _lidarMaxRange = lidarMaxRange;
    }

    py::tuple processScan(
        py::array_t<float, py::array::c_style | py::array::forcecast> distances,
        py::array_t<float, py::array::c_style | py::array::forcecast> angles,
        float odomX,
        float odomY,
        float odomTheta,
        double timestamp)
    {
        if (distances.size() != angles.size())
        {
            throw std::runtime_error(
                "distances and angles must be the same size");
        }

        const int count = distances.size();
        const float *dist = distances.data();
        const float *ang = angles.data();

        cv::Mat scan(1, count, CV_32FC2);
        int validCount = 0;
        float angleMin = INFINITY, angleMax = -INFINITY;

        for (int i = 0; i < count; i++)
        {
            const float r = dist[i];
            const float a = ang[i];

            if (a < angleMin)
            {
                angleMin = a;
            }

            if (a > angleMax)
            {
                angleMax = a;
            }

            if (!std::isfinite(r) || !std::isfinite(a) || r <= 0.05f || r > 12.0f)
            {
                continue;
            }

            scan.at<cv::Vec2f>(0, validCount) = cv::Vec2f(r * std::cos(a), r * std::sin(a));

            ++validCount;
        }

        scan = scan.colRange(0, validCount);

        // kUnknown=0,			/**< Unknown format. */
        // kXY=1,				/**< 2D points with X and Y coordinates. */
        // kXYI=2,				/**< 2D points with X, Y and intensity. */
        // kXYNormal=3,		    /**< 2D points with X, Y and normal vectors. */
        // kXYINormal=4,		/**< 2D points with X, Y, intensity and normal vectors. */
        rtabmap::LaserScan laserScan(scan,
                                     rtabmap::LaserScan::Format::kXY, // format
                                     _lidarMinRange,                  // min range
                                     _lidarMaxRange,                  // max range
                                     angleMin,                        // min angle
                                     angleMax,                        // max angle
                                     (angleMax - angleMin) / count    // increment
        );

        rtabmap::SensorData data(
            laserScan,
            cv::Mat(),
            cv::Mat(),
            rtabmap::CameraModel(),
            scanId_++,
            timestamp);

        rtabmap::Transform odom(
            odomX,
            odomY,
            0.0f,
            0.0f,
            0.0f,
            odomTheta);

        if (rtabmap_.process(data, odom))
        {
            const rtabmap::Statistics &stats = rtabmap_.getStatistics();
            const rtabmap::Signature &node = stats.getLastSignatureData();

            if (node.sensorData().gridCellSize() > 0.0f)
            {
                if (grid.addedNodes().find(node.id()) == grid.addedNodes().end())
                {
                    cv::Mat ground, obstacles, empty;
                    node.sensorData().uncompressDataConst(0, 0, 0, 0, &ground, &obstacles, &empty);
                    localGrids.add(node.id(), ground, obstacles, empty, node.sensorData().gridCellSize(), node.sensorData().gridViewPoint());
                }
            }

            grid.update(stats.poses());
        }

        rtabmap::Transform pose = getPose();

        float px = pose.x();
        float py = pose.y();
        float ptheta = pose.theta();

        return py::make_tuple(
            px,
            py,
            ptheta);
    }

    py::tuple getOccupancyGrid()
    {
        cv::Mat gridImage = getGrid();
        py::array_t<uint8_t> gridArray(
            {gridImage.rows,
             gridImage.cols});

        std::memcpy(
            gridArray.mutable_data(),
            gridImage.data,
            gridImage.total() * gridImage.elemSize());

        return py::make_tuple(xMin, yMin, gridArray);
    }

private:
    rtabmap::ParametersMap parameters;
    rtabmap::Rtabmap rtabmap_;
    int scanId_ = 0;

    float _lidarMinRange;
    float _lidarMaxRange;

    rtabmap::LocalGridCache localGrids;
    rtabmap::OccupancyGrid grid = rtabmap::OccupancyGrid(&localGrids, parameters);

    float xMin, yMin;

    cv::Mat getGrid()
    {
        cv::Mat map = grid.getMap(xMin, yMin); // -1 unknown, 0 free, 100 occupied
        return convertGrid(map);
    }

    rtabmap::Transform getPose() const
    {
        return rtabmap_.getPose(scanId_);
    }
};

PYBIND11_MODULE(rtabmap_py, m)
{
    m.doc() = "RTAB-Map Python wrapper for 2D LiDAR SLAM";

    py::class_<RtabmapSLAM>(m, "RtabmapSLAM")
        .def(
            py::init<float, float, float>(),
            py::arg("lidarMinRange"),
            py::arg("lidarMaxRange"),
            py::arg("resolution"))
        .def(
            "process",
            &RtabmapSLAM::processScan,
            py::arg("distances"),
            py::arg("angles"),
            py::arg("odomX"),
            py::arg("odomY"),
            py::arg("odomTheta"),
            py::arg("timestamp"))
        .def(
            "getOccupancyGrid",
            &RtabmapSLAM::getOccupancyGrid);
}