# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'messageconsole.ui'
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
from PySide6.QtWidgets import (QApplication, QSizePolicy, QTextBrowser, QVBoxLayout,
    QWidget)

class Ui_MessageConsole(object):
    def setupUi(self, MessageConsole):
        if not MessageConsole.objectName():
            MessageConsole.setObjectName(u"MessageConsole")
        MessageConsole.resize(509, 192)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MessageConsole.sizePolicy().hasHeightForWidth())
        MessageConsole.setSizePolicy(sizePolicy)
        MessageConsole.setMinimumSize(QSize(0, 0))
        MessageConsole.setMaximumSize(QSize(16777215, 16777215))
        self.verticalLayout = QVBoxLayout(MessageConsole)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.textBrowser = QTextBrowser(MessageConsole)
        self.textBrowser.setObjectName(u"textBrowser")
        sizePolicy.setHeightForWidth(self.textBrowser.sizePolicy().hasHeightForWidth())
        self.textBrowser.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.textBrowser)


        self.retranslateUi(MessageConsole)

        QMetaObject.connectSlotsByName(MessageConsole)
    # setupUi

    def retranslateUi(self, MessageConsole):
        MessageConsole.setWindowTitle(QCoreApplication.translate("MessageConsole", u"Form", None))
    # retranslateUi

