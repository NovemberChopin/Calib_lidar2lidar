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


void LoadExtrinsicYaml(const std::string &filename, Eigen::Matrix4d &extrinsic) {
  YAML::Node config = YAML::LoadFile(filename);
  std::vector<double> vec;
  for (YAML::const_iterator it=config["extrinsicMat"].begin(); it != config["extrinsicMat"].end(); ++it) {
    vec.push_back(it->as<double>());
  }
  // 将vector 数据写入 Eigen 矩阵
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      extrinsic(i, j) = vec[j + i*4];
    }
  }
}


void SaveExtrinsicYaml(const std::string &filename, Eigen::Matrix4d &extrinsic) {
  
  std::ofstream fout(filename);
  std::vector<double> vec;
  YAML::Node config;
  // std::cout << extrinsic << std::endl;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      vec.push_back(extrinsic(i, j));
    }
  }
  config["extrinsicMat"] = vec;

  Eigen::Vector3d angles = getRotation(extrinsic);
  config["rotation"]["pitch"] = angles(0);
  config["rotation"]["yaw"] = angles(1);
  config["rotation"]["roll"] = angles(2);

  Eigen::Vector3d trans = extrinsic.block<3, 1>(0, 3);
  config["trans"]["x"] = angles(0);
  config["trans"]["y"] = angles(1);
  config["trans"]["z"] = angles(2);

  fout << config;
  fout.close();
}


Eigen::Vector3d getRotation(Eigen::Matrix4d &extrinsic) {
  Eigen::Matrix3d rot_matrix = 
        extrinsic.block<3, 3>(0, 0);
  
  Eigen::Vector3d euler_angles = rot_matrix.eulerAngles(0, 1, 2);
  Eigen::Vector3d angles = euler_angles * 180 / M_PI;
  return angles;
}