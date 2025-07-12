# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'read_data.ui'
##
## Created by: Qt User Interface Compiler version 6.8.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QGridLayout, QPushButton, QSizePolicy,
    QWidget)

class Ui_ReadDataForm(object):
    def setupUi(self, ReadDataForm):
        if not ReadDataForm.objectName():
            ReadDataForm.setObjectName(u"ReadDataForm")
        ReadDataForm.resize(228, 24)
        self.gridLayout = QGridLayout(ReadDataForm)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.pushButton_open_folder = QPushButton(ReadDataForm)
        self.pushButton_open_folder.setObjectName(u"pushButton_open_folder")

        self.gridLayout.addWidget(self.pushButton_open_folder, 0, 0, 1, 1)


        self.retranslateUi(ReadDataForm)

        QMetaObject.connectSlotsByName(ReadDataForm)
    # setupUi

    def retranslateUi(self, ReadDataForm):
        ReadDataForm.setWindowTitle(QCoreApplication.translate("ReadDataForm", u"Form", None))
        self.pushButton_open_folder.setText(QCoreApplication.translate("ReadDataForm", u"Open folder", None))
    # retranslateUi

