#include "CloudViewer.h"
#include "Tools.h"

#include <fstream>
#include <iostream>
#include <string>


double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
static Eigen::Matrix4d calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix4d orign_calibration_matrix_ = Eigen::Matrix4d::Identity();
std::vector<Eigen::Matrix4d> modification_list_;

void CalibrationInit(Eigen::Matrix4d json_param) {
  calibration_matrix_ = json_param;
  orign_calibration_matrix_ = json_param;
  modification_list_.reserve(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale Init!\n";
}


void CloudViewer::testEigen() {
    QString fileName = QFileDialog::getOpenFileName(this, "Open Json", "/home/js/Documents", "Open json files(*.json)");
    Eigen::Matrix4d json_param;
    LoadExtrinsicJson(fileName.toStdString(), json_param);
    std::cout << "lidar to lidar extrinsic:\n" << std::endl;
    std::cout << json_param << std::endl;
    CalibrationInit(json_param);
}


CloudViewer::CloudViewer(QWidget *parent)
  : QMainWindow(parent) {
  ui.setupUi(this);

  /***** Slots connection of QMenuBar and QToolBar *****/
  // File (connect)
  QObject::connect(ui.openAction, &QAction::triggered, this, &CloudViewer::open);
  QObject::connect(ui.addAction, &QAction::triggered, this, &CloudViewer::add);
  QObject::connect(ui.clearAction, &QAction::triggered, this, &CloudViewer::clear);

  ui.saveAction->setData(QVariant(false));       // isSaveBinary = false
  ui.saveBinaryAction->setData(QVariant(true));  // isSaveBinary = true
  connect(ui.saveAction, SIGNAL(triggered()), this, SLOT(save()));
  connect(ui.saveBinaryAction, SIGNAL(triggered()), this, SLOT(save()));
  QObject::connect(ui.exitAction, &QAction::triggered, this, &CloudViewer::exit);
  // Display (connect)
  QObject::connect(ui.pointcolorAction, &QAction::triggered, this, &CloudViewer::pointcolorChanged);
  QObject::connect(ui.bgcolorAction, &QAction::triggered, this, &CloudViewer::bgcolorChanged);
  QObject::connect(ui.mainviewAction, &QAction::triggered, this, &CloudViewer::mainview);
  QObject::connect(ui.leftviewAction, &QAction::triggered, this, &CloudViewer::leftview);
  QObject::connect(ui.topviewAction, &QAction::triggered, this, &CloudViewer::topview);


  /***** Slots connection of RGB widget *****/
  // Random color (connect)
  connect(ui.colorBtn, SIGNAL(clicked()), this, SLOT(colorBtnPressed()));
  // Connection between RGB slider and RGB value (connect)
  connect(ui.rSlider, SIGNAL(valueChanged(int)), this, SLOT(rSliderChanged(int)));
  connect(ui.gSlider, SIGNAL(valueChanged(int)), this, SLOT(gSliderChanged(int)));
  connect(ui.bSlider, SIGNAL(valueChanged(int)), this, SLOT(bSliderChanged(int)));
  // RGB slider released (connect)
  connect(ui.rSlider, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
  connect(ui.gSlider, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
  connect(ui.bSlider, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
  // Change size of cloud (connect)
  connect(ui.pSlider, SIGNAL(valueChanged(int)), this, SLOT(pSliderChanged(int)));
  connect(ui.pSlider, SIGNAL(sliderReleased()), this, SLOT(psliderReleased()));
  // Checkbox for coordinate and background color (connect)
  connect(ui.cooCbx, SIGNAL(stateChanged(int)), this, SLOT(cooCbxChecked(int)));
  connect(ui.bgcCbx, SIGNAL(stateChanged(int)), this, SLOT(bgcCbxChecked(int)));

  /***** Slots connection of dataTree(QTreeWidget) widget *****/
  // Item in dataTree is left-clicked (connect)
  connect(ui.dataTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(itemSelected(QTreeWidgetItem*, int)));
  // Item in dataTree is right-clicked
  connect(ui.dataTree, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(popMenu(const QPoint&)));

  connect(ui.consoleTable, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(popMenuInConsole(const QPoint&)));
  // Initialization
  initial();
}

CloudViewer::~CloudViewer() {

}

void CloudViewer::doOpen(const QStringList& filePathList) {
  // Open point cloud file one by one
  for (int i = 0; i != filePathList.size(); i++) {
    timeStart(); // time start
    mycloud.cloud.reset(new PointCloudT); // Reset cloud
    QFileInfo fileInfo(filePathList[i]);
    std::string filePath = fromQString(fileInfo.filePath());
    std::string fileName = fromQString(fileInfo.fileName());

    // begin loading
    ui.statusBar->showMessage(
      fileInfo.fileName() + ": " + QString::number(i) + "/" + QString::number(filePathList.size())
      + " point cloud loading..."
    );

    mycloud = fileIO.load(fileInfo);
    if (!mycloud.isValid) {
      // TODO: deal with the error, print error info in console?
      debug("invalid cloud.");
      continue;
    }
    mycloud.viewer = viewer;
    mycloud_vec.push_back(mycloud);

    timeCostSecond = timeOff(); // time off

    consoleLog(
      "Open",
      toQString(mycloud.fileName),
      toQString(mycloud.filePath),
      "Time cost: " + timeCostSecond + " s, Points: " + QString::number(mycloud.cloud->points.size())
    );

    // update tree widget
    QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList()
      << toQString(mycloud.fileName));
    cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
    ui.dataTree->addTopLevelItem(cloudName);

    total_points += mycloud.cloud->points.size();
  }
  ui.statusBar->showMessage("");
  showPointcloudAdd();
}

// Open point cloud
void CloudViewer::open() {
  QStringList filePathList = QFileDialog::getOpenFileNames(
    this,
    tr("Open point cloud file"),
    toQString(mycloud.fileDir),
    toQString(fileIO.getInputFormatsStr())
  );
  if (filePathList.isEmpty()) return;

  // Clear cache
  // TODO: abstract a function
  mycloud_vec.clear();
  total_points = 0;
  ui.dataTree->clear();
  viewer->removeAllPointClouds();

  doOpen(filePathList);
}

// Add Point Cloud
void CloudViewer::add() {
  QStringList filePathList = QFileDialog::getOpenFileNames(
    this,
    tr("Add point cloud file"),
    toQString(mycloud.fileDir),
    toQString(fileIO.getInputFormatsStr())
  );
  if (filePathList.isEmpty()) return;

  doOpen(filePathList);
}

// Clear all point clouds
void CloudViewer::clear() {
  mycloud_vec.clear();  // 从点云容器中移除所有点云
  viewer->removeAllPointClouds();  // 从viewer中移除所有点云
  viewer->removeAllShapes(); // 这个remove更彻底
  ui.dataTree->clear();  // 将dataTree清空

  // 输出窗口
  consoleLog("Clear", "All point clouds", "", "");

  setWindowTitle("CloudViewer");  // 更新窗口标题
  showPointcloud();  // 更新显示
}

// Save point cloud
void CloudViewer::save() {
  if (!mycloud.isValid) {
    QMessageBox::critical(this, tr("Saving file error"),
      tr("There is no point cloud to save"));
    return;
  }

  // get binary flag from sender()
  QAction *action = qobject_cast<QAction *>(sender());
  QVariant v = action->data();
  bool isSaveBinary = (bool)v.value<bool>();

  QString selectedFilter = toQString(fileIO.outputFiltersMap.at(mycloud.fileSuffix));
  QString saveFilePath = QFileDialog::getSaveFileName(
    this,                                    // parent
    toQString("Save point cloud" + string(isSaveBinary ? " (binary)": "")), // caption
    toQString(mycloud.filePath),             // dir
    toQString(fileIO.getOutputFormatsStr()), // filter
    &selectedFilter                          // selected filter
  );
  if (saveFilePath.isEmpty()) return;

  QFileInfo fileInfo(saveFilePath);
  QString saveFileName = fileInfo.fileName();
  string saveFilePathStd = fromQString(saveFilePath);
  string saveFileNameStd = fromQString(saveFileName);

  if (mycloud_vec.size() > 1) {
    savemulti(fileInfo, isSaveBinary);
    return;
  }

  bool saveStatus = fileIO.save(mycloud, fileInfo, isSaveBinary);
  if (!saveStatus) {
    QMessageBox::critical(this, tr("Saving file error"),
      tr("We can not save the file"));
    return;
  }

  consoleLog("Save", saveFileName, saveFilePath, "Single save");

  setWindowTitle(saveFilePath + " - CloudViewer");
  QMessageBox::information(this, tr("save point cloud file"),
    toQString("Save " + saveFileNameStd + " successfully!"));
}

// Save multi point cloud
void CloudViewer::savemulti(const QFileInfo& fileInfo, bool isSaveBinary) {
  string subname = fromQString(fileInfo.fileName());
  QString saveFilePath = fileInfo.filePath();
  PointCloudT::Ptr multi_cloud;
  multi_cloud.reset(new PointCloudT);
  multi_cloud->height = 1;
  int sum = 0;
  for (auto c : mycloud_vec) {
    sum += c.cloud->points.size();
  }
  multi_cloud->width = sum;
  multi_cloud->resize(multi_cloud->height * multi_cloud->width);
  int k = 0;
  for (int i = 0; i != mycloud_vec.size(); ++i) {
    // 注意cloudvec[i]->points.size()和cloudvec[i]->size()的区别
    for (int j = 0; j != mycloud_vec[i].cloud->points.size(); ++j) {
      multi_cloud->points[k].x = mycloud_vec[i].cloud->points[j].x;
      multi_cloud->points[k].y = mycloud_vec[i].cloud->points[j].y;
      multi_cloud->points[k].z = mycloud_vec[i].cloud->points[j].z;
      multi_cloud->points[k].r = mycloud_vec[i].cloud->points[j].r;
      multi_cloud->points[k].g = mycloud_vec[i].cloud->points[j].g;
      multi_cloud->points[k].b = mycloud_vec[i].cloud->points[j].b;
      k++;
    }
  }

  MyCloud multiMyCloud;
  multiMyCloud.cloud = multi_cloud;
  multiMyCloud.isValid = true;

  // save multi_cloud
  bool saveStatus = fileIO.save(multiMyCloud, fileInfo, isSaveBinary);
  if (!saveStatus) {
    QMessageBox::critical(this, tr("Saving file error"),
      tr("We can not save the file"));
    return;
  }

  if (isSaveBinary) {
    consoleLog("Save as binary", QString::fromLocal8Bit(subname.c_str()), saveFilePath, "Multi save (binary)");
  } else {
    consoleLog("Save", QString::fromLocal8Bit(subname.c_str()), saveFilePath, "Multi save");
  }

  // 将保存后的 multi_cloud 设置为当前 mycloud,以便保存之后直接进行操作
  mycloud.cloud = multi_cloud;
  mycloud.filePath = fromQString(saveFilePath);
  mycloud.fileName = subname;

  setWindowTitle(saveFilePath + " - CloudViewer");
  QMessageBox::information(this, tr("save point cloud file"), toQString("Save " + subname + " successfully!"));
}

// 退出程序
void CloudViewer::exit() {
  this->close();
}


// 初始化
void CloudViewer::initial() {
  // 界面初始化
  setWindowIcon(QIcon(tr(":/Resources/images/icon.png")));
  setWindowTitle(tr("CloudViewer"));

  // 点云初始化
  mycloud.cloud.reset(new PointCloudT);
  mycloud.cloud->resize(1);
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  // viewer->addPointCloud(cloud, "cloud");

  ui.screen->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui.screen->GetInteractor(), ui.screen->GetRenderWindow());
  ui.screen->update();

  ui.consoleTable->setSelectionMode(QAbstractItemView::NoSelection);  // 禁止点击输出窗口的 item
  ui.dataTree->setSelectionMode(QAbstractItemView::ExtendedSelection); // 允许 dataTree 进行多选

  setConsoleTable();

  // 输出窗口
  consoleLog("Software start", "CloudViewer", "Welcome to use CloudViewer", "Nightn");

  // 设置背景颜色为 dark
  viewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);

}

// 显示点云，不重置相机角度
void CloudViewer::showPointcloud() {
  for (int i = 0; i != mycloud_vec.size(); i++)
  {
    viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
  }
//   viewer->resetCamera();
  ui.screen->update();
}

// 添加点云到viewer,并显示点云
void CloudViewer::showPointcloudAdd() {
  for (int i = 0; i != mycloud_vec.size(); i++) {
    viewer->addPointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
    viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
  }
  viewer->resetCamera();
  ui.screen->update();
}

void CloudViewer::setCloudColor(unsigned int r, unsigned int g, unsigned int b) {
  // Set the new color
  for (size_t i = 0; i < mycloud.cloud->size(); i++) {
    mycloud.cloud->points[i].r = r;
    mycloud.cloud->points[i].g = g;
    mycloud.cloud->points[i].b = b;
    mycloud.cloud->points[i].a = 255;
  }
}


/*********************************************/
/*****************界面槽函数*****************/
/********************************************/
void CloudViewer::colorBtnPressed() {
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  int selected_item_count = ui.dataTree->selectedItems().size();
  // 如果未选中任何点云，则对视图窗口中的所有点云进行着色
  if (selected_item_count == 0){
    for (int i = 0; i != mycloud_vec.size(); i++){
      for (int j = 0; j != mycloud_vec[i].cloud->points.size(); j++){
        mycloud_vec[i].cloud->points[j].r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
        mycloud_vec[i].cloud->points[j].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
        mycloud_vec[i].cloud->points[j].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
      }
    }

    // 输出窗口
    consoleLog("Random color", "All point clous", "", "");

  }
  else{
    for (int i = 0; i != selected_item_count; i++){
      int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
      for (int j = 0; j != mycloud_vec[cloud_id].cloud->size(); j++){
        mycloud_vec[cloud_id].cloud->points[j].r = red;
        mycloud_vec[cloud_id].cloud->points[j].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
        mycloud_vec[cloud_id].cloud->points[j].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
      }
    }

    // 输出窗口
    consoleLog("Random color", "Point clouds selected", "", "");
  }
  showPointcloud();
}

void CloudViewer::RGBsliderReleased() {
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  int selected_item_count = ui.dataTree->selectedItems().size();
  // 如果未选中任何点云，则对视图窗口中的所有点云进行着色
  if (selected_item_count == 0) {
    for (int i = 0; i != mycloud_vec.size(); i++) {
      mycloud_vec[i].setPointColor(red, green, blue);
    }

    // 输出窗口
    consoleLog("Change cloud color", "All point clouds", QString::number(red) + " " + QString::number(green) + " " + QString::number(blue), "");
  }
  else{
    for (int i = 0; i != selected_item_count; i++) {
      int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
      mycloud_vec[cloud_id].setPointColor(red, green, blue);
    }
    // 输出窗口
    consoleLog("Change cloud color", "Point clouds selected", QString::number(red) + " " + QString::number(green) + " " + QString::number(blue), "");
  }
  showPointcloud();
}

// 设置所有点云的尺寸
void CloudViewer::psliderReleased() {
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  int selected_item_count = ui.dataTree->selectedItems().size();
  if (selected_item_count == 0){
    for (int i = 0; i != mycloud_vec.size(); i++){
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        p, mycloud_vec[i].cloudId);
    }
    // 输出窗口
    consoleLog("Change cloud size", "All point clouds", "Size: " + QString::number(p), "");
  }
  else{
    for (int i = 0; i != selected_item_count; i++){
      int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        p, mycloud_vec[i].cloudId);
    }
    // 输出窗口
    consoleLog("Change cloud size", "Point clouds selected", "Size: " + QString::number(p), "");
  }
  ui.screen->update();
}

void CloudViewer::pSliderChanged(int value) {
  p = value;
  ui.sizeLCD->display(value);

}

void CloudViewer::rSliderChanged(int value) {
  red = value;
  ui.rLCD->display(value);
}

void CloudViewer::gSliderChanged(int value) {
  green = value;
  ui.gLCD->display(value);
}

void CloudViewer::bSliderChanged(int value) {
  blue = value;
  ui.bLCD->display(value);
}

void CloudViewer::cooCbxChecked(int value) {
  switch (value) {
    case 0: {
      viewer->removeCoordinateSystem();
      consoleLog("Remove coordinate system", "Remove", "", "");
      break;
    }
    case 2: {
      viewer->addCoordinateSystem();
      consoleLog("Add coordinate system", "Add", "", "");
      break;
    }
  }
  ui.screen->update();
}

void CloudViewer::bgcCbxChecked(int value) {
  switch (value) {
    case 0: {
      viewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);
      consoleLog("Change bg color", "Background", "30 30 30", "");
      break;
    }
    case 2: {
      // ！注意：setBackgroundColor()接收的是0-1的double型参数
      viewer->setBackgroundColor(240 / 255.0, 240 / 255.0, 240 / 255.0);
      consoleLog("Change bg color", "Background", "240 240 240", "");
      break;
    }
  }
  ui.screen->update();
}

// 通过颜色对话框改变点云颜色
void CloudViewer::pointcolorChanged() {
  QColor color = QColorDialog::getColor(Qt::white, this, "Select color for point cloud");

  if (color.isValid()) {
    // QAction* action = dynamic_cast<QAction*>(sender());
    // if (action != ui.pointcolorAction) // 改变颜色的信号来自于 dataTree
    QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
    int selected_item_count = ui.dataTree->selectedItems().size();
    if (selected_item_count == 0) {
      for (int i = 0; i != mycloud_vec.size(); ++i) {
        mycloud_vec[i].setPointColor(color.red(), color.green(), color.blue());
      }
      // 输出窗口
      consoleLog("Change cloud color", "All point clouds", QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()), "");
    }
    else {
      for (int i = 0; i != selected_item_count; i++) {
        int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
        mycloud_vec[cloud_id].setPointColor(color.red(), color.green(), color.blue());
      }
      // 输出窗口
      consoleLog("Change cloud color", "Point clouds selected", QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()), "");
    }
    // 颜色的改变同步至RGB停靠窗口
    ui.rSlider->setValue(color.red());
    ui.gSlider->setValue(color.green());
    ui.bSlider->setValue(color.blue());

    showPointcloud();
  }
}

// 通过颜色对话框改变背景颜色
void CloudViewer::bgcolorChanged() {
  QColor color = QColorDialog::getColor(Qt::white, this,
    "Select color for point cloud");
  if (color.isValid()) {
    viewer->setBackgroundColor(color.red() / 255.0,
      color.green() / 255.0, color.blue() / 255.0);
    // 输出窗口
    consoleLog("Change bg color", "Background", QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()), "");
    showPointcloud();
  }
}

// 三视图
void CloudViewer::mainview() {
  viewer->setCameraPosition(0, -1, 0, 0.5, 0.5, 0.5, 0, 0, 1);
  ui.screen->update();
}

void CloudViewer::leftview() {
  viewer->setCameraPosition(-1, 0, 0, 0, 0, 0, 0, 0, 1);
  ui.screen->update();
}

void CloudViewer::topview() {
  viewer->setCameraPosition(0, 0, 1, 0, 0, 0, 0, 1, 0);
  ui.screen->update();
}


void CloudViewer::setConsoleTable() {
  // 设置输出窗口
  QStringList header2;
  header2 << "Time" << "Operation" << "Operation object" << "Details" << "Note";
  ui.consoleTable->setHorizontalHeaderLabels(header2);
  ui.consoleTable->setColumnWidth(0, 150);
  ui.consoleTable->setColumnWidth(1, 200);
  ui.consoleTable->setColumnWidth(2, 200);
  ui.consoleTable->setColumnWidth(3, 300);

  // ui.consoleTable->setEditTriggers(QAbstractItemView::NoEditTriggers); // 设置不可编辑
  ui.consoleTable->verticalHeader()->setDefaultSectionSize(22); // 设置行距

  ui.consoleTable->setContextMenuPolicy(Qt::CustomContextMenu);

}

void CloudViewer::consoleLog(QString operation, QString subname, QString filename, QString note) {
  if (enable_console == false) {
    return;
  }
  int rows = ui.consoleTable->rowCount();
  ui.consoleTable->setRowCount(++rows);
  QDateTime time = QDateTime::currentDateTime();  // 获取系统现在的时间
  QString time_str = time.toString("MM-dd hh:mm:ss"); // 设置显示格式
  ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(time_str));
  ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
  ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(subname));
  ui.consoleTable->setItem(rows - 1, 3, new QTableWidgetItem(filename));
  ui.consoleTable->setItem(rows - 1, 4, new QTableWidgetItem(note));

  ui.consoleTable->scrollToBottom(); // 滑动自动滚到最底部
}

// QTreeWidget的item的点击相应函数
void CloudViewer::itemSelected(QTreeWidgetItem* item, int count) {
  count = ui.dataTree->indexOfTopLevelItem(item);  // 获取item的行号

  for (int i = 0; i != mycloud_vec.size(); i++)
  {
    viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, mycloud_vec[i].cloudId);
  }

  // 提取当前点云的RGB,点云数量等信息
  int cloud_size = mycloud_vec[count].cloud->points.size();
  unsigned int cloud_r = mycloud_vec[count].cloud->points[0].r;
  unsigned int cloud_g = mycloud_vec[count].cloud->points[0].g;
  unsigned int cloud_b = mycloud_vec[count].cloud->points[0].b;
  bool multi_color = true;
  if (mycloud_vec[count].cloud->points.begin()->r == (mycloud_vec[count].cloud->points.end() - 1)->r) // 判断点云单色多色的条件（不是很严谨）
    multi_color = false;

  // 选中item所对应的点云尺寸变大
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  int selected_item_count = ui.dataTree->selectedItems().size();
  for (int i = 0; i != selected_item_count; i++){
    int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      2, mycloud_vec[i].cloudId);
  }
  // mycloud = mycloud_vec[count];
  ui.screen->update();
}

// consoleTable 右击响应事件
void CloudViewer::popMenuInConsole(const QPoint&) {
  QAction clearConsoleAction("Clear console", this);
  QAction enableConsoleAction("Enable console", this);
  QAction disableConsoleAction("Disable console", this);

  connect(&clearConsoleAction, &QAction::triggered, this, &CloudViewer::clearConsole);
  connect(&enableConsoleAction, &QAction::triggered, this, &CloudViewer::enableConsole);
  connect(&disableConsoleAction, &QAction::triggered, this, &CloudViewer::disableConsole);

  QPoint pos;
  QMenu menu(ui.dataTree);
  menu.addAction(&clearConsoleAction);
  menu.addAction(&enableConsoleAction);
  menu.addAction(&disableConsoleAction);

  if (enable_console == true){
    menu.actions()[1]->setVisible(false);
    menu.actions()[2]->setVisible(true);
  }
  else{
    menu.actions()[1]->setVisible(true);
    menu.actions()[2]->setVisible(false);
  }

  menu.exec(QCursor::pos()); // 在当前鼠标位置显示
}

// 清空 consoleTable
void CloudViewer::clearConsole() {
  ui.consoleTable->clearContents();
  ui.consoleTable->setRowCount(0);
}

// 允许使用 consoleTable
void CloudViewer::enableConsole() {
  enable_console = true;
}

// 禁用 consoleTable
void CloudViewer::disableConsole() {
  clearConsole();
  enable_console = false;

}

// QTreeWidget的item的右击响应函数
void CloudViewer::popMenu(const QPoint&) {
  QTreeWidgetItem* curItem = ui.dataTree->currentItem(); // 获取当前被点击的节点
  if (curItem == NULL)return;           // 这种情况是右键的位置不在treeItem的范围内，即在空白位置右击
  QString name = curItem->text(0);
  int id = ui.dataTree->indexOfTopLevelItem(curItem);
  MyCloud& myCloud = mycloud_vec[id];

  QAction hideItemAction("Hide", this);
  QAction showItemAction("Show", this);
  QAction deleteItemAction("Delete", this);
  QAction changeColorAction("Change color", this);
  // show mode
  QAction pointModeAction("Set point mode", this);
  QAction meshModeAction("Set mesh mode", this);
  QAction pointMeshModeAction("Set point+mesh mode", this);
  pointModeAction.setData(QVariant(CLOUDVIEWER_MODE_POINT));
  meshModeAction.setData(QVariant(CLOUDVIEWER_MODE_MESH));
  pointMeshModeAction.setData(QVariant(CLOUDVIEWER_MODE_POINT_MESH));

  pointModeAction.setCheckable(true);
  meshModeAction.setCheckable(true);
  pointMeshModeAction.setCheckable(true);

  if (myCloud.curMode == "point") {
    pointModeAction.setChecked(true);
  }
  else if (myCloud.curMode == "mesh") {
    meshModeAction.setChecked(true);
  }
  else if (myCloud.curMode == "point+mesh") {
    pointMeshModeAction.setChecked(true);
  }

  connect(&hideItemAction, &QAction::triggered, this, &CloudViewer::hideItem);
  connect(&showItemAction, &QAction::triggered, this, &CloudViewer::showItem);
  connect(&deleteItemAction, &QAction::triggered, this, &CloudViewer::deleteItem);
  connect(&changeColorAction, &QAction::triggered, this, &CloudViewer::pointcolorChanged);

  connect(&pointModeAction, SIGNAL(triggered()), this, SLOT(setRenderingMode()));
  connect(&meshModeAction, SIGNAL(triggered()), this, SLOT(setRenderingMode()));
  connect(&pointMeshModeAction, SIGNAL(triggered()), this, SLOT(setRenderingMode()));

  QPoint pos;
  QMenu menu(ui.dataTree);
  menu.addAction(&hideItemAction);
  menu.addAction(&showItemAction);
  menu.addAction(&deleteItemAction);
  menu.addAction(&changeColorAction);

  menu.addAction(&pointModeAction);
  menu.addAction(&meshModeAction);
  menu.addAction(&pointMeshModeAction);

  if (mycloud_vec[id].visible == true){
    menu.actions()[1]->setVisible(false);
    menu.actions()[0]->setVisible(true);
  }
  else{
    menu.actions()[1]->setVisible(true);
    menu.actions()[0]->setVisible(false);
  }

  const vector<string> modes = myCloud.supportedModes;
  if (std::find(modes.begin(), modes.end(), "point") == modes.end()) {
    menu.actions()[4]->setVisible(false);
  }
  if (std::find(modes.begin(), modes.end(), "mesh") == modes.end()) {
    menu.actions()[5]->setVisible(false);
  }
  if (std::find(modes.begin(), modes.end(), "point+mesh") == modes.end()) {
    menu.actions()[6]->setVisible(false);
  }

  menu.exec(QCursor::pos()); // 在当前鼠标位置显示
}
void CloudViewer::hideItem() {
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  for (int i = 0; i != ui.dataTree->selectedItems().size(); i++){
    // TODO hide之后，item变成灰色，再次右击item时，“hideItem” 选项变成 “showItem”
    // QTreeWidgetItem* curItem = ui.dataTree->currentItem();
    QTreeWidgetItem* curItem = itemList[i];
    QString name = curItem->text(0);
    int id = ui.dataTree->indexOfTopLevelItem(curItem);
    mycloud_vec[id].hide();
    // QMessageBox::information(this, "cloud_id", QString::fromLocal8Bit(cloud_id.c_str()));

    QColor item_color = QColor(112, 122, 132, 255);
    curItem->setTextColor(0, item_color);
    mycloud_vec[id].visible = false;
  }

  // 输出窗口
  consoleLog("Hide point clouds", "Point clouds selected", "", "");

  ui.screen->update(); // 刷新视图窗口，不能省略
}

void CloudViewer::showItem() {
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  for (int i = 0; i != ui.dataTree->selectedItems().size(); i++){
    // QTreeWidgetItem* curItem = ui.dataTree->currentItem();
    QTreeWidgetItem* curItem = itemList[i];
    QString name = curItem->text(0);
    int id = ui.dataTree->indexOfTopLevelItem(curItem);
    // 将cloud_id所对应的点云设置成透明
    mycloud_vec[id].show();
    QColor item_color;
    if (theme_id == 0){
      item_color = QColor(0, 0, 0, 255);
    }
    else{
      item_color = QColor(241, 241, 241, 255);
    }
    curItem->setTextColor(0, item_color);
    mycloud_vec[id].visible = true;
  }

  // 输出窗口
  consoleLog("Show point clouds", "Point clouds selected", "", "");

  ui.screen->update(); // 刷新视图窗口，不能省略

}

void CloudViewer::deleteItem() {
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  // ui.dataTree->selectedItems().size() 随着迭代次数而改变，因此循环条件要设置为固定大小的 selected_item_count
  int selected_item_count = ui.dataTree->selectedItems().size();
  for (int i = 0; i != selected_item_count; i++){
    // QTreeWidgetItem* curItem = ui.dataTree->currentItem();
    // QMessageBox::information(this, "itemList's size", QString::number(ui.dataTree->selectedItems().size()));
    QTreeWidgetItem* curItem = itemList[i];
    QString name = curItem->text(0);
    int id = ui.dataTree->indexOfTopLevelItem(curItem);
    // QMessageBox::information(this, "information", "curItem: " + name + " " + QString::number(id));
    auto it = mycloud_vec.begin() + ui.dataTree->indexOfTopLevelItem(curItem);
    // 删除点云之前，将其点的数目保存
    int delete_points = (*it).cloud->points.size();
    it = mycloud_vec.erase(it);
    // QMessageBox::information(this, "information", QString::number(delete_points) + " " + QString::number(mycloud_vec.size()));

    total_points -= delete_points;

    ui.dataTree->takeTopLevelItem(ui.dataTree->indexOfTopLevelItem(curItem));
  }

  // 移除之后再添加，避免 id 和资源管理树行号不一致的情况
  viewer->removeAllPointClouds();
  for (int i = 0; i != mycloud_vec.size(); i++)
  {
    viewer->addPointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
    viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
  }

  // 输出窗口
  consoleLog("Delete point clouds", "Point clouds selected", "", "");

  ui.screen->update();

}

void CloudViewer::setRenderingMode() {
  QAction *action = qobject_cast<QAction *>(sender());
  QVariant v = action->data();
  int mode = (int)v.value<int>();
  string modeStr;

  switch (mode) {
    case CLOUDVIEWER_MODE_POINT: {
      modeStr = "point";
      consoleLog("Point Mode", "Point clouds selected", "", "");
      break;
    }
    case CLOUDVIEWER_MODE_MESH: {
      modeStr = "mesh";
      consoleLog("Mesh Mode", "Point clouds selected", "", "");
      break;
    }
    case CLOUDVIEWER_MODE_POINT_MESH: {
      modeStr = "point+mesh";
      consoleLog("Point+Mesh Mode", "Point clouds selected", "", "");
      break;
    }
  }

  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  for (int i = 0; i != ui.dataTree->selectedItems().size(); i++) {
    QTreeWidgetItem* curItem = itemList[i];
    QString name = curItem->text(0);
    int id = ui.dataTree->indexOfTopLevelItem(curItem);
    MyCloud& myCloud = mycloud_vec[id];
    myCloud.setShowMode(modeStr);
  }
  ui.screen->update();
}

void CloudViewer::debug(const string& s) {
  QMessageBox::information(this, tr("Debug"), QString::fromLocal8Bit(s.c_str()));
}

