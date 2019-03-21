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
    TIME_KEY = "time"

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
            ScoreTracker.SCORE_KEY : -1,
            ScoreTracker.TIME_KEY  : -1
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

    def update_score(self, nick, score, time):
        """
        Update score for the given nickname.
        """
        
        if score > 579:
            score = 579

        updated = False
        for i in range(len(self.scoring_list)):

            if self.scoring_list[i][ScoreTracker.NICK_KEY] == nick:
                print("Found nick: Updating score")
                current_value = self.scoring_list[i][ScoreTracker.SCORE_KEY]
                current_time = self.scoring_list[i][ScoreTracker.TIME_KEY]
                # User is found - check if score needs to be updated
                
                if current_value < score:
                    # Score needs to be updated
                    print("ScoreTracker: Saving score and time for user {}".format(nick))
                    self.scoring_list[i][ScoreTracker.SCORE_KEY] = score
                    self.scoring_list[i][ScoreTracker.TIME_KEY] = time

                elif current_value == score and current_time < time:
                    # Time needs to be updated
                    print("ScoreTracker: Saving only time for user {}".format(nick))
                    self.scoring_list[i][ScoreTracker.TIME_KEY] = time

                updated = True
                break

        if not updated:
            raise Exception("Given score and nickname pair is not found in the scoring list.")

        self.write_yaml()

    def my_key(self, x):
        return x[ScoreTracker.SCORE_KEY], x[ScoreTracker.TIME_KEY]

    def get_sorted_score(self):
        """
        Return a sorted list of dictionary entries.
        """

        return sorted(self.scoring_list, key=self.my_key, reverse=True)[0:20]

    def get_user_list(self):
        """
        Get list of available users.
        """

        usr_list = []
        for item in self.scoring_list:
            usr_list.append(item[ScoreTracker.NICK_KEY])

        return usr_list