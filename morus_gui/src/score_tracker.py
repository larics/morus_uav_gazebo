import yaml
import rospkg
import datetime
import os

class ScoreTracker():
    """
    This class implemets score tracking functionality.
    """

    NICK_KEY = "nick"
    FNAME_KEY = "fname"
    LNAME_KEY = "lname"
    SCORE_KEY = "score"

    def __init__(self):
        """
        Initialize Score tracker. Make a new score sheet with format date_score.yaml. 
        If it already exists read and save its contents.
        """

        # Get morus_gui path as ROS package
        rospack = rospkg.RosPack()
        path = rospack.get_path("morus_gui")

        # Create a new scoresheet filename
        resource_path = path + "/resources/"
        filename = datetime.date.today().strftime("%B_%d_%Y_score.yaml")
        self.file_path = resource_path + filename     
        print("Saving score to file: {}".format(self.file_path))

        if os.path.isfile(self.file_path):  

            # Read YAML contents
            with open(self.file_path) as f:
                
                self.scoring_list = yaml.load(f)
                if self.scoring_list is None:
                    self.scoring_list = []

        else:

            # Create a file if it does not exists   
            self.scoring_list = []
            open(self.file_path, 'a').close()

        print("Current score sheet: \n")
        for item in self.scoring_list:
            print(item)


    def add_initial_entry(self, nick, fname, lname):
        """
        Return true if name is added, otherwise false.
        """

        for item in self.scoring_list:
            
            # Nickname already registered
            if item[ScoreTracker.NICK_KEY] == nick:
                return False

        self.scoring_list.append({
            ScoreTracker.NICK_KEY : nick,
            ScoreTracker.FNAME_KEY : fname,
            ScoreTracker.LNAME_KEY : lname,
            ScoreTracker.SCORE_KEY : -1
            })

        
        self.write_yaml()
        return True

    def write_yaml(self):
        """
        Write current scoring list to the yaml file.
        """

        # Write to YAML
        print("Writing to yaml.")
        with open(self.file_path, "w") as f:
            yaml.dump(self.scoring_list, f)

    def update_score(self, nick, score):
        """
        Update score for the given nickname.
        """
        
        updated = False
        for i in range(len(self.scoring_list)):

            if self.scoring_list[i][ScoreTracker.NICK_KEY] == nick:
                self.scoring_list[i][ScoreTracker.SCORE_KEY] = score
                updated = True
                break

        if not updated:
            raise Exception("Given score and nickname pair is not found in the scoring list.")

        self.write_yaml()

    def get_sorted_score(self):
        """
        Return a sorted list of dictionary entries.
        """

        return sorted(self.scoring_list, key= lambda k : k[ScoreTracker.SCORE_KEY], reverse=True)

    def get_user_list(self):
        """
        Get list of available users.
        """

        usr_list = []
        for item in self.scoring_list:
            usr_list.append(item[ScoreTracker.NICK_KEY])

        return usr_list