#ifndef LOGGER_H_
#define LOGGER_H_

#include <fstream>
#include <iostream>

using namespace std;

class Logger {
 public:
  Logger(string filename) { file_.open(filename); }

  Logger() : Logger("nis.csv") {}

  virtual ~Logger() { file_.close(); }

  void log(string log, double nis) { file_ << log << "," << nis << endl; }

  void log(VectorXd v) {
    for (auto i = 0; i < v.size(); i++) {
      file_ << v(i) << ",";
    }
    file_ << endl;
  }

 private:
  ofstream file_;
};

#endif