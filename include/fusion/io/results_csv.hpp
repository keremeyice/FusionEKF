#pragma once
// ---------------------------------------------------------------------------
// fusion/io/results_csv.hpp -- writes filter results to a CSV file, so they can
// be plotted (tools/plot_car_2d.py) or opened in a spreadsheet.
// ---------------------------------------------------------------------------
#include <string>
#include <vector>

namespace fusion {

/// One row of results: truth, the GNSS fix (if any) and the filter estimate.
struct ResultRow {
  double t = 0.0;
  double true_x = 0.0;
  double true_y = 0.0;
  bool has_gnss = false;
  double gnss_x = 0.0;  ///< only meaningful when has_gnss is true
  double gnss_y = 0.0;
  double est_x = 0.0;
  double est_y = 0.0;
  double est_vx = 0.0;
  double est_vy = 0.0;
  double sigma_x = 0.0;  ///< estimated standard deviation of x [m]
  double sigma_y = 0.0;
};

/// The first line of the file.
inline const std::string kResultsCsvHeader =
    "t,true_x,true_y,has_gnss,gnss_x,gnss_y,est_x,est_y,est_vx,est_vy,sigma_x,sigma_y";

/// Writes the header line, then one line per row, values separated by commas,
/// in the same order as the header. has_gnss is written as 1 or 0.
/// Returns false if the file could not be opened.
bool write_results_csv(const std::string& path, const std::vector<ResultRow>& rows);

}  // namespace fusion
