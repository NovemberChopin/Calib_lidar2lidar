#ifndef CLOUDVIEWER_H
#define CLOUDVIEWER_H

// for solving error: no override found for 'vtkRenderWindow'
#include <vtkAutoInit.h>
// VTK_MODULE_INIT(vtkRenderingOpenGL2);
// VTK_MODULE_INIT(vtkInteractionStyle);

#include "MyCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>  // loadPolygonFileOBJ

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PolygonMesh.h>
#include <json/json.h>

#include <QtWidgets/QMainWindow>
#include "ui_CloudViewer.h"
#include "Tools.h"
#include "FileIO.h"
#include "Lidar2Lidar.h"

#include <vector>
#include <map>
#include <algorithm>
#include <QtWidgets/QMainWindow>
#include <QString>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QColorDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "QVTKWidget.h"
#include <vtkRenderWindow.h>
#include <QTextEdit>
#include <QTime>
#include <QMouseEvent>
#include <QDesktopServices>
#include <QUrl>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const int CLOUDVIEWER_MODE_POINT = 0;
const int CLOUDVIEWER_MODE_MESH = 1;
const int CLOUDVIEWER_MODE_POINT_MESH = 2;

using std::vector;
using std::string;

class CloudViewer : public QMainWindow
{
  Q_OBJECT

public:
  CloudViewer(QWidget *parent = 0);
  ~CloudViewer();

private:
  Ui::CloudViewerClass ui;

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud;
  MyCloud mycloud;      // 暂时先不动
  MyCloud sourceCloud;  // 需要进行变换的源点云
  std::vector<MyCloud> mycloud_vec;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  Lidar2Lidar lidarCalib;

  FileIO fileIO;

  QString save_filename;
  long total_points = 0; //Total amount of points in the viewer

  unsigned int red = 255;
  unsigned int green = 255;
  unsigned int blue = 255;
  unsigned int p = 2;
  std::vector<int> pointcolor;
  std::vector<int> bgcolor;

  QVBoxLayout *layout;

  int theme_id = 1; // 0: Windows theme, 1: Darcula theme
  bool enable_console = true; // console 的可用状态
  QString timeCostSecond = "0";  // 记录某个动作执行的时间

  /***** Slots of QMenuBar and QToolBar *****/
  // File menu slots
  void open();
  void add();
  void doOpen(const QStringList& filePathList);
  void clear();

  void savemulti(const QFileInfo& fileInfo, bool isSaveBinary);
  void exit();
  // Display menu slots
  void pointcolorChanged();
  void bgcolorChanged();
  void mainview();
  void leftview();
  void topview();

  /***** Utils Methods ***/
  void initial();
  void showPointcloud();  //显示点云
  void showPointcloudAdd();  //添加给viewer，显示点云

  void setCloudColor(unsigned int r, unsigned int g, unsigned int b);

  void setConsoleTable();

  void consoleLog(QString operation, QString subname, QString filename, QString note);

  void processSourceFrame(MyCloud &sourceCloud, const Eigen::Matrix4d &extrinsic);

  void showPopInfo();   // 弹窗提示
public slots:

  void loadParams();
  void showCalibParams();

  void save();

  void colorBtnPressed();
  void RGBsliderReleased();
  void psliderReleased();
  void pSliderChanged(int value);
  void rSliderChanged(int value);
  void gSliderChanged(int value);
  void bSliderChanged(int value);
  // Slots of checkBox
  void cooCbxChecked(int value);
  void bgcCbxChecked(int value);

  /***** Slots of dataTree(QTreeWidget) widget *****/
  // Item in dataTree is left-clicked
  void itemSelected(QTreeWidgetItem*, int);
  // Item in dataTree is right-clicked
  void popMenu(const QPoint&);
  void hideItem();
  void showItem();
  void deleteItem();

  // set show mode
  void setRenderingMode();

  void popMenuInConsole(const QPoint&);
  void clearConsole();
  void enableConsole();
  void disableConsole();

  void debug(const string& s);
};
#endif // CLOUDVIEWER_H

