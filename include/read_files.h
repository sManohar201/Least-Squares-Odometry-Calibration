#ifndef LEAST_SQUARES_READ_FILES_H
#define LEAST_SQUARES_READ_FILES_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <vector>
#include <filesystem>

namespace LeastSquares
{
  class FileReader
  {

    public:
      /**
       * @brief Construct a new File Reader object
       * 
       * @param[in] file_path - input file path
       */
      FileReader(std::string &file_path);
      /**
       * @brief Destroy the File Reader object
       */
      ~FileReader();
      /**
       * @brief - reads the file and counts the no of columns and rows in the file,
       * and sets rows and cols data variable.
       */
      void count_columns_rows();
      /**
       * @brief - reads through the input file and loads the data in a matrix.
       */
      void load_data();

    private:
      /**
       * @brief - path of the file
       */
      std::string file_name;
      /**
       * @brief - stream object to read the input file
       */
      std::ifstream file;
      /**
       * @brief - final data loaded as matrix
       */
      Eigen::MatrixXd data;
      /**
       * @brief - no of rows in the data
       */
      size_t rows;
      /**
       * @brief - no of columns in the data
       */
      size_t cols;
  };

} // namespace LeastSquares



#endif // LEAST_SQUARES_READ_FILES_H