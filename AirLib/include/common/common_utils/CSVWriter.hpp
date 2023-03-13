// Weibin Gu
// A class to create and write data in a csv file.
// https://thispointer.com/how-to-write-data-in-a-csv-file-in-c/

#ifndef commn_utils_CSVWriter_hpp
#define commn_utils_CSVWriter_hpp

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <iterator>
#include <string>
#include <algorithm>

class CSVWriter
{
    private:
        std::string fileName;
        std::string delimeter;
        int linesCount;

    public:
        CSVWriter(std::string filename, std::string delm = ",") :
                fileName(filename), delimeter(delm), linesCount(0)
        {}

        /*
        * Member function to store a range as comma seperated value
        */
        template<typename T>
        void addDatainRow(T first, T last)
        {
            std::fstream file;
            // Open the file in truncate mode if first line else in Append Mode
            file.open(fileName, std::ios::out | (linesCount ? std::ios::app : std::ios::trunc));
            // Iterate over the range and add each lement to file seperated by delimeter.
            for (; first != last; )
            {
                file << *first;
                if (++first != last)
                    file << delimeter;
            }
            file << "\n";
            linesCount++;
            // Close the file
            file.close();
        }
};

#endif