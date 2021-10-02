
// #include "matplotlibcpp.h"
#include "read_files.h"
#include "least_squares.h"
#include "tools.h"

#include "pbplot/pbPlots.hpp"
#include "pbplot/supportLib.hpp"

void plot_trajectory(const Eigen::MatrixXd &odom, const Eigen::MatrixXd &scan, const Eigen::MatrixXd &corrected_odom)
{
  RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

  Eigen::MatrixXd odom_trajectory;
  LeastSquares::compute_trajectory(odom, odom_trajectory);

  Eigen::MatrixXd scan_trajectory;
  LeastSquares::compute_trajectory(scan, scan_trajectory);

  Eigen::MatrixXd corrected_trajectory;
  LeastSquares::compute_trajectory(corrected_odom, corrected_trajectory);

  size_t length = odom_trajectory.rows();
  std::vector<double> x_odom(length, 0), y_odom(length, 0), x_scan(length, 0), x_corrected(length, 0), y_scan(length, 0), y_corrected(length, 0);

  for (size_t i=0; i<length; i++)
  {
    x_odom[i] = odom_trajectory(i, 0);
    y_odom[i] = odom_trajectory(i, 1);

    x_scan[i] = scan_trajectory(i, 0);
    y_scan[i] = scan_trajectory(i, 1);

    x_corrected[i] = corrected_trajectory(i, 0);
    y_corrected[i] = corrected_trajectory(i, 1);
  }

  ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
  series->xs = &x_odom;
  series->ys = &y_odom;
  series->linearInterpolation = false;
  series->pointType = toVector(L"solid");
  series->color = CreateRGBColor(1, 0, 0);

  ScatterPlotSeries *series1 = GetDefaultScatterPlotSeriesSettings();
  series->xs = &x_scan;
  series->ys = &y_scan;
  series->linearInterpolation = false;
  series->pointType = toVector(L"solid");
  series->color = CreateRGBColor(0, 1, 0);

  ScatterPlotSeries *series2 = GetDefaultScatterPlotSeriesSettings();
  series->xs = &x_corrected;
  series->ys = &y_corrected;
  series->linearInterpolation = false;
  series->pointType = toVector(L"solid");
  series->color = CreateRGBColor(0, 0, 1);  

  ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
  settings->width = 700;
  settings->height = 550;
  settings->autoBoundaries = true;
  settings->autoPadding = true;
  settings->title = toVector(L"Trajectory Plots [Red - Odom; Blue - Scan; Green - Corrected]");
  settings->xLabel = toVector(L"");
  settings->yLabel = toVector(L"");
  settings->scatterPlotSeries->push_back(series);
  settings->scatterPlotSeries->push_back(series1);
  settings->scatterPlotSeries->push_back(series2);

  DrawScatterPlotFromSettings(imageReference, settings);

  std::vector<double> *pngData = ConvertToPNG(imageReference->image);
  WriteToFile(pngData, "trajectory.png");
  DeleteImage(imageReference->image);

}

int main() 
{
  LeastSquares::FileReader odom_motions("../resources/odom_motions.txt");
  Eigen::MatrixXd odom = odom_motions.get_data();

  LeastSquares::FileReader scanmatched_motions("../resources/scanmatched_motions.txt");
  Eigen::MatrixXd scan = scanmatched_motions.get_data();

  if (!scanmatched_motions.check_dimension(odom_motions))
  {
    std::cerr << "The dimeensions doesn't match";
  }

  LeastSquares::Matrix3X3 parameters;

  LeastSquares::least_squares_calibrate_odometry(scan, odom,  parameters);

  Eigen::MatrixXd corrected_odom;

  LeastSquares::apply_odometry_correction(parameters, odom, corrected_odom);

  plot_trajectory(odom, scan, corrected_odom);

  return 0;
}