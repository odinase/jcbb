#include <string>
#include <sstream>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iterator>
#include <iostream>
#include <vector>
#include <string>
#include <array>
#include "file_parser.h"

namespace
{
    // Alot of code stolen from here: https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c

    class CSVRow
    {
    public:
        std::string_view operator[](std::size_t index) const
        {
            return std::string_view(&m_line[m_data[index] + 1], m_data[index + 1] - (m_data[index] + 1));
        }
        std::size_t size() const
        {
            return m_data.size() - 1;
        }
        void readNextRow(std::istream &str)
        {
            std::getline(str, m_line);

            m_data.clear();
            m_data.emplace_back(-1);
            std::string::size_type pos = 0;
            while ((pos = m_line.find(',', pos)) != std::string::npos)
            {
                m_data.emplace_back(pos);
                ++pos;
            }
            // This checks for a trailing comma with no data after it.
            pos = m_line.size();
            m_data.emplace_back(pos);
        }

    private:
        std::string m_line;
        std::vector<int> m_data;
    };

    std::istream &operator>>(std::istream &str, CSVRow &data)
    {
        data.readNextRow(str);
        return str;
    }

    class CSVIterator
    {
    public:
        typedef std::input_iterator_tag iterator_category;
        typedef CSVRow value_type;
        typedef std::size_t difference_type;
        typedef CSVRow *pointer;
        typedef CSVRow &reference;

        CSVIterator(std::istream &str) : m_str(str.good() ? &str : NULL) { ++(*this); }
        CSVIterator() : m_str(NULL) {}

        // Pre Increment
        CSVIterator &operator++()
        {
            if (m_str)
            {
                if (!((*m_str) >> m_row))
                {
                    m_str = NULL;
                }
            }
            return *this;
        }
        // Post increment
        CSVIterator operator++(int)
        {
            CSVIterator tmp(*this);
            ++(*this);
            return tmp;
        }
        CSVRow const &operator*() const { return m_row; }
        CSVRow const *operator->() const { return &m_row; }

        bool operator==(CSVIterator const &rhs) { return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL))); }
        bool operator!=(CSVIterator const &rhs) { return !((*this) == rhs); }

    private:
        std::istream *m_str;
        CSVRow m_row;
    };
}

Eigen::MatrixXd txt2mat(const std::string& file_name)
{
    std::ifstream file(file_name);

    int rows = 0, cols;
    std::vector<double> buffer;
    std::stringstream ss;
    for (CSVIterator loop(file); loop != CSVIterator(); ++loop)
    {
        cols = loop->size();
        for (int j = 0; j < loop->size(); j++)
        {
            ss.clear();
            ss << (*loop)[j];
            double t;
            ss >> t;
            buffer.push_back(t);
        }
        rows++;
    }

    Eigen::MatrixXd mat(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            mat(i, j) = buffer[i * cols + j];
        }
    }

    return mat;
}