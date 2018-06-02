#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <fstream>

using namespace std;

class Logger {
 public:
 Logger() {
     nis_file_.open("nis.csv");
 }
 virtual ~Logger(){
     nis_file_.close();
 }

void log(string log, double nis){
    nis_file_ << log << "," << nis << endl;
}

 private:
 ofstream nis_file_;
};

#endif