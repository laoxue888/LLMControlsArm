# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwindow.ui'
##
## Created by: Qt User Interface Compiler version 6.8.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QFrame, QMainWindow,
    QMenu, QMenuBar, QPushButton, QRadioButton,
    QScrollArea, QSizePolicy, QSplitter, QStackedWidget,
    QToolBar, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 622)
        self.actionCheck_for_updates = QAction(MainWindow)
        self.actionCheck_for_updates.setObjectName(u"actionCheck_for_updates")
        self.actionUser_Guide = QAction(MainWindow)
        self.actionUser_Guide.setObjectName(u"actionUser_Guide")
        self.actionOpen_log_dir = QAction(MainWindow)
        self.actionOpen_log_dir.setObjectName(u"actionOpen_log_dir")
        font = QFont()
        self.actionOpen_log_dir.setFont(font)
        self.actionUpdate_log = QAction(MainWindow)
        self.actionUpdate_log.setObjectName(u"actionUpdate_log")
        self.actionUpdate_log.setFont(font)
        self.actionUserGuide = QAction(MainWindow)
        self.actionUserGuide.setObjectName(u"actionUserGuide")
        self.actionWindow1 = QAction(MainWindow)
        self.actionWindow1.setObjectName(u"actionWindow1")
        self.action = QAction(MainWindow)
        self.action.setObjectName(u"action")
        self.actionexecute_all_node = QAction(MainWindow)
        self.actionexecute_all_node.setObjectName(u"actionexecute_all_node")
        self.actionexecute_graph = QAction(MainWindow)
        self.actionexecute_graph.setObjectName(u"actionexecute_graph")
        self.actionWindow2 = QAction(MainWindow)
        self.actionWindow2.setObjectName(u"actionWindow2")
        self.actionexetute_from_goal_node = QAction(MainWindow)
        self.actionexetute_from_goal_node.setObjectName(u"actionexetute_from_goal_node")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout_3 = QVBoxLayout(self.centralwidget)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.splitter = QSplitter(self.centralwidget)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setOrientation(Qt.Orientation.Vertical)
        self.stackedWidget = QStackedWidget(self.splitter)
        self.stackedWidget.setObjectName(u"stackedWidget")
        self.page_window1 = QWidget()
        self.page_window1.setObjectName(u"page_window1")
        self.verticalLayout_2 = QVBoxLayout(self.page_window1)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.scrollArea = QScrollArea(self.page_window1)
        self.scrollArea.setObjectName(u"scrollArea")
        self.scrollArea.setWidgetResizable(True)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setObjectName(u"scrollAreaWidgetContents")
        self.scrollAreaWidgetContents.setGeometry(QRect(0, 0, 98, 28))
        self.pushButton = QPushButton(self.scrollAreaWidgetContents)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(240, 110, 151, 31))
        self.checkBox = QCheckBox(self.scrollAreaWidgetContents)
        self.checkBox.setObjectName(u"checkBox")
        self.checkBox.setGeometry(QRect(420, 170, 82, 20))
        self.radioButton = QRadioButton(self.scrollAreaWidgetContents)
        self.radioButton.setObjectName(u"radioButton")
        self.radioButton.setGeometry(QRect(160, 170, 98, 20))
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)

        self.verticalLayout_2.addWidget(self.scrollArea)

        self.stackedWidget.addWidget(self.page_window1)
        self.page_window2 = QWidget()
        self.page_window2.setObjectName(u"page_window2")
        self.verticalLayout_5 = QVBoxLayout(self.page_window2)
        self.verticalLayout_5.setSpacing(0)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_graph = QVBoxLayout()
        self.verticalLayout_graph.setObjectName(u"verticalLayout_graph")

        self.verticalLayout_5.addLayout(self.verticalLayout_graph)

        self.stackedWidget.addWidget(self.page_window2)
        self.splitter.addWidget(self.stackedWidget)
        self.frame = QFrame(self.splitter)
        self.frame.setObjectName(u"frame")
        self.frame.setMinimumSize(QSize(0, 200))
        self.frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout = QVBoxLayout(self.frame)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_msg = QVBoxLayout()
        self.verticalLayout_msg.setSpacing(0)
        self.verticalLayout_msg.setObjectName(u"verticalLayout_msg")

        self.verticalLayout.addLayout(self.verticalLayout_msg)

        self.splitter.addWidget(self.frame)

        self.verticalLayout_3.addWidget(self.splitter)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menuBar = QMenuBar(MainWindow)
        self.menuBar.setObjectName(u"menuBar")
        self.menuBar.setGeometry(QRect(0, 0, 800, 33))
        font1 = QFont()
        font1.setBold(False)
        self.menuBar.setFont(font1)
        self.menuHelp = QMenu(self.menuBar)
        self.menuHelp.setObjectName(u"menuHelp")
        self.menuHelp.setFont(font1)
        self.menuWindow = QMenu(self.menuBar)
        self.menuWindow.setObjectName(u"menuWindow")
        self.menuTools = QMenu(self.menuBar)
        self.menuTools.setObjectName(u"menuTools")
        MainWindow.setMenuBar(self.menuBar)
        self.toolBar = QToolBar(MainWindow)
        self.toolBar.setObjectName(u"toolBar")
        MainWindow.addToolBar(Qt.ToolBarArea.TopToolBarArea, self.toolBar)

        self.menuBar.addAction(self.menuTools.menuAction())
        self.menuBar.addAction(self.menuWindow.menuAction())
        self.menuBar.addAction(self.menuHelp.menuAction())
        self.menuHelp.addAction(self.actionOpen_log_dir)
        self.menuHelp.addAction(self.actionUserGuide)
        self.menuHelp.addAction(self.actionUpdate_log)
        self.menuWindow.addAction(self.actionWindow1)
        self.menuWindow.addAction(self.actionWindow2)
        self.menuTools.addAction(self.actionexetute_from_goal_node)
        self.menuTools.addAction(self.actionexecute_graph)
        self.toolBar.addAction(self.actionexetute_from_goal_node)
        self.toolBar.addAction(self.actionexecute_graph)

        self.retranslateUi(MainWindow)

        self.stackedWidget.setCurrentIndex(1)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"GraphExecuter", None))
        self.actionCheck_for_updates.setText(QCoreApplication.translate("MainWindow", u"Check for updates", None))
        self.actionUser_Guide.setText(QCoreApplication.translate("MainWindow", u"User Guide", None))
        self.actionOpen_log_dir.setText(QCoreApplication.translate("MainWindow", u"\u6253\u5f00\u65e5\u5fd7\u6587\u4ef6\u5939", None))
        self.actionUpdate_log.setText(QCoreApplication.translate("MainWindow", u"\u66f4\u65b0\u8bb0\u5f55", None))
        self.actionUserGuide.setText(QCoreApplication.translate("MainWindow", u"\u7528\u6237\u6559\u7a0b", None))
        self.actionWindow1.setText(QCoreApplication.translate("MainWindow", u"\u7a97\u53e31", None))
        self.action.setText(QCoreApplication.translate("MainWindow", u"\u4ece\u9009\u4e2d\u8282\u70b9\u6267\u884c", None))
        self.actionexecute_all_node.setText(QCoreApplication.translate("MainWindow", u"\u6267\u884c\u6240\u6709\u8282\u70b9", None))
        self.actionexecute_graph.setText(QCoreApplication.translate("MainWindow", u"\u8fd0\u884c\u6574\u56fe", None))
        self.actionexecute_graph.setIconText(QCoreApplication.translate("MainWindow", u"\u4ece\u6240\u6709\u7684\u76ee\u6807\u8282\u70b9\u5f00\u59cb\u521b\u5efa\u591a\u7ebf\u7a0b\u8fd0\u884c\u6574\u4e2a\u56fe", None))
#if QT_CONFIG(tooltip)
        self.actionexecute_graph.setToolTip(QCoreApplication.translate("MainWindow", u"\u4ece\u6240\u6709\u7684\u76ee\u6807\u8282\u70b9\u5f00\u59cb\u521b\u5efa\u591a\u7ebf\u7a0b\u8fd0\u884c\u6574\u4e2a\u56fe", None))
#endif // QT_CONFIG(tooltip)
        self.actionWindow2.setText(QCoreApplication.translate("MainWindow", u"\u7a97\u53e32", None))
        self.actionexetute_from_goal_node.setText(QCoreApplication.translate("MainWindow", u"\u9009\u4e2d\u76ee\u6807\u8282\u70b9\u8fd0\u884c", None))
        self.actionexetute_from_goal_node.setIconText(QCoreApplication.translate("MainWindow", u"\u4ece\u9009\u4e2d\u7684\u76ee\u6807\u8282\u70b9\u5f00\u59cb\u8fd0\u884c", None))
#if QT_CONFIG(tooltip)
        self.actionexetute_from_goal_node.setToolTip(QCoreApplication.translate("MainWindow", u"\u4ece\u9009\u4e2d\u7684\u76ee\u6807\u8282\u70b9\u5f00\u59cb\u8fd0\u884c", None))
#endif // QT_CONFIG(tooltip)
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
        self.checkBox.setText(QCoreApplication.translate("MainWindow", u"CheckBox", None))
        self.radioButton.setText(QCoreApplication.translate("MainWindow", u"RadioButton", None))
        self.menuHelp.setTitle(QCoreApplication.translate("MainWindow", u"\u5e2e\u52a9", None))
        self.menuWindow.setTitle(QCoreApplication.translate("MainWindow", u"\u7a97\u53e3", None))
        self.menuTools.setTitle(QCoreApplication.translate("MainWindow", u"\u5de5\u5177", None))
        self.toolBar.setWindowTitle(QCoreApplication.translate("MainWindow", u"toolBar", None))
    # retranslateUi

