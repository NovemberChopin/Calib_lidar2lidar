#ifndef __TOOLS_H__
#define __TOOLS_H__

#pragma once

#include <QTime>
#include <QString>

#include <fstream>
#include <iostream>
#include <json/json.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <Eigen/Core>

using std::string;
using std::vector;

class Tools
{
public:
  Tools();
  ~Tools();
};

string getFileName(string file_name);
void timeStart();
QString timeOff();

// string to QString
QString toQString(const string& s);
// QString to string
string fromQString(const QString& qs);

string joinStrVec(const vector<string> v, string splitor = " ");

void LoadExtrinsicJson(const std::string &filename, Eigen::Matrix4d &extrinsic);

#endif

