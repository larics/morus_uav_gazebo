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

        for i, item in enumerate(self.sorted_scores):
            print(item)
            h_layout = self.makeHorizontalLayout(i, item)
            self.scroll_layout.addLayout(h_layout)

        v_layout = QVBoxLayout()
        v_layout.addWidget(self.info_label)
        v_layout.addWidget(scroll_area)

        self.setLayout(v_layout)
        self.setGeometry(300, 300, 250, 500)
        self.setWindowTitle("Highscores")

        self.show()

    def makeHorizontalLayout(self, index, item):
        """
        Return a horizontal layout containing give score.
        """

        left_label = QLabel()
        left_label.setText("{0}. {1}".format(
            index+1, item[ScoreTracker.NICK_KEY]))
        right_label = QLabel()
        right_label.setText("{0}".format(
            item[ScoreTracker.SCORE_KEY]))
        
        layout = QHBoxLayout()
        layout.addWidget(left_label)
        layout.addWidget(right_label)
        layout.setSpacing(120)
        layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        return layout