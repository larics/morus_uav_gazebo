from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from score_tracker import ScoreTracker

class HighscoreDialog(QDialog):
    """
    This class Implements a highscore dialog for tracking user score.
    """

    def __init__(self, track_obj):
        super(self.__class__, self).__init__()

        self.sorted_scores = track_obj.get_sorted_score()
        self.setupUI()

    def setupUI(self):

        self.info_label = QLabel("Highscores")
        newfont = QtGui.QFont("Aerial", 16, QtGui.QFont.Bold) 
        self.info_label.setFont(newfont)
        self.info_label.setAlignment(Qt.AlignCenter)

        scroll_area = QScrollArea()
        self.scroll_layout = QVBoxLayout()
        scroll_area.setLayout(self.scroll_layout)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFixedWidth(400)
        scroll_area.setFixedHeight(600)

        start_item = {
            ScoreTracker.NICK_KEY   : "NAME",
            ScoreTracker.FNAME_KEY  : "-",
            ScoreTracker.LNAME_KEY  : "-",
            ScoreTracker.SCORE_KEY  : "SCORE",
            ScoreTracker.TIME_KEY   : "TIME"
        }

        self.scroll_layout.addLayout(
            self.makeHorizontalLayout("", start_item))
        for i, item in enumerate(self.sorted_scores):
            h_layout = self.makeHorizontalLayout(i+1, item)
            self.scroll_layout.addLayout(h_layout)

        v_layout = QVBoxLayout()
        v_layout.addWidget(self.info_label)
        v_layout.addWidget(scroll_area)

        self.setLayout(v_layout)
        self.setWindowTitle("Highscores")
        self.show()
        self.adjustSize()

    def makeHorizontalLayout(self, index, item):
        """
        Return a horizontal layout containing give score.
        """
        print(item)

        left_label = QLabel()
        left_label.setText("{0}. {1}".format(
            index, item[ScoreTracker.NICK_KEY]))
        left_label.setFixedWidth(100)

        right_label = QLabel()
        right_label.setText("{0}".format(
            item[ScoreTracker.SCORE_KEY]))
        right_label.setFixedWidth(100)
        
        time_label = QLabel()
        time_label.setText("{0}".format(
            item[ScoreTracker.TIME_KEY]))
        time_label.setFixedWidth(100)

        layout = QHBoxLayout()
        layout.addWidget(left_label)
        layout.addWidget(right_label)
        layout.addWidget(time_label)
        layout.setSpacing(120)
        layout.setAlignment(Qt.AlignTop)

        return layout