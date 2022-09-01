#include "Tools.h"

QTime myTime;

Tools::Tools()
{
}

Tools::~Tools()
{
}

string getFileName(string file_name)
{
  string subname;
  for (auto i = file_name.end() - 1; *i != '/'; i--)
  {
    subname.insert(subname.begin(), *i);
  }
  return subname;
}

void timeStart()
{
  myTime.start();
}

QString timeOff()
{
  int timediff = myTime.elapsed();
  float f = timediff / 1000.0;
  QString tr_timediff = QString("%1").arg(f);  // float->QString
  return tr_timediff;
}

QString toQString(const string& s) {
  QString qs(s.c_str());
  return qs;
}

string fromQString(const QString& qs) {
  string s = qs.toUtf8().data();
  return s;
}

string joinStrVec(const vector<string> v, string splitor) {
  string s = "";
  if (v.size() == 0) return s;
  for (int i = 0; i != v.size()  - 1; ++i) {
    s += (v[i] + splitor);
  }
  s += v[v.size() - 1];
  return s;
}


void LoadExtrinsicJson(const std::string &filename, Eigen::Matrix4d &extrinsic) {
    Json::Reader reader;
    Json::Value root;
    std::ifstream in(filename, std::ios::binary);
    // std::ifstream in;
    // in.open(filename);
    if (!in.is_open()) {
    std::cout << "Error Opening " << filename << std::endl;
    return;
    }
    if (reader.parse(in, root, false)) {
    auto name = root.getMemberNames();
    std::string id = *(name.begin());
    std::cout << id << std::endl;
    Json::Value data = root[id]["param"]["sensor_calib"]["data"];
    extrinsic << data[0][0].asDouble(), data[0][1].asDouble(), data[0][2].asDouble(), data[0][3].asDouble(),
                data[1][0].asDouble(), data[1][1].asDouble(), data[1][2].asDouble(), data[1][3].asDouble(),
                data[2][0].asDouble(), data[2][1].asDouble(), data[2][2].asDouble(), data[2][3].asDouble(),
                data[3][0].asDouble(), data[3][1].asDouble(), data[3][2].asDouble(), data[3][3].asDouble();
    }
    in.close();
    return;
}

