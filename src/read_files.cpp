#include "../include/read_files.h"


namespace LeastSquares
{
  FileReader::FileReader(const std::string &file_path) : 
                         file_name_(file_path),
                         rows_(0), cols_(0)
  {
    
    open_file();
    // count the no of rows and cols to initialize matrix
    count_columns_rows();
    data_.resize(rows_, cols_);
    load_data();
  }

  FileReader::~FileReader() 
  {
    file_.close();
  }

  void FileReader::open_file()
  {
    file_.open(file_name_, std::ios_base::in);
    if (!file_.is_open())
    {
      std::cerr << " The file is not open!";
    }
  }

  void FileReader::count_columns_rows()
  {
    std::string line;
    while (std::getline(file_, line))
    {
      rows_++;
    }
    size_t commas = line.find(' ');
    while (commas != std::string::npos)
    {
      cols_++;
      commas = line.find(' ', commas+1);
    }
    file_.close();
  }

  void FileReader::load_data()
  {
    open_file();
    size_t index = 0;
    double a, b, c;
    while (file_ >> a >> b >> c)
    {
      data_.row(index) << a, b, c;
      index++;
    }
    file_.close();
  }

  bool FileReader::check_dimension (const FileReader &other) const
  {
    bool one = this->get_rows() == other.get_rows();
    bool two = this->get_columns() == other.get_columns();
    return (one && two);
  }

  const Eigen::MatrixXd FileReader::get_data() const 
  {
    return data_;
  }

  const size_t FileReader::get_rows() const 
  {
    return rows_;
  }

  const size_t FileReader::get_columns() const
  {
    return cols_;
  }

} // namespace LeastSquares
