#ifndef LEAST_SQUARES_READ_FILES_H
#define LEAST_SQUARES_READ_FILES_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <vector>
#include <iostream>


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
      FileReader(const std::string &file_path);
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
      /**
       * @brief Get the data object
       * 
       * @return const Eigen::MatrixXd - data matrix
       */
      const Eigen::MatrixXd get_data() const;
      
      /**
       * @brief - check if the dimension of other data matches with the current
       * one
       * @param[in] other - filereader object
       */
      bool check_dimension (const FileReader &other) const;

    private:
      /**
       * @brief Get the rows object
       * @return const size_t 
       */
      const size_t get_rows() const;
      /**
       * @brief Get the columns object
       * @return const size_t 
       */
      const size_t get_columns() const; 
      /**
       * @brief - opens the file in the given file path.
       */
      void open_file();
      /**
       * @brief - path of the file
       */
      std::string file_name_;
      /**
       * @brief - stream object to read the input file
       */
      std::ifstream file_;
      /**
       * @brief - final data loaded as matrix
       */
      Eigen::MatrixXd data_;
      /**
       * @brief - no of rows in the data
       */
      size_t rows_;
      /**
       * @brief - no of columns in the data
       */
      size_t cols_;
  };

} // namespace LeastSquares



#endif // LEAST_SQUARES_READ_FILES_H