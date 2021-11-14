#ifndef FILE_PARSER_H
#define FILE_PARSER_H

#include <string>
#include <sstream>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iterator>
#include <iostream>
#include <vector>
#include <string>

Eigen::MatrixXd txt2mat(const std::string& file_name);


#endif // FILE_PARSER_H