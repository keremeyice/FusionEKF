#include "fusion/io/results_csv.hpp"

#include <fstream>

namespace fusion {

bool write_results_csv(const std::string& path,
                       const std::vector<ResultRow>& rows) {
std::ofstream file(path);       // open (create or overwrite) the file
  if (!file) {                    // could it be opened? (e.g. the folder doesn't exist)
    return false;
  }
  file << kResultsCsvHeader << '\n';
  for(const ResultRow& row : rows)
  {
    file << row.t << ',' << row.true_x << ',' << row.true_y << ',' << (row.has_gnss ? 1 : 0) << ','
    << row.gnss_x << ',' << row.gnss_y << ',' << row.est_x << ',' << row.est_y << ','
    << row.est_vx << ',' << row.est_vy << ',' << row.sigma_x << ',' << row.sigma_y << '\n';
  }
  return true;
}

}  // namespace fusion
