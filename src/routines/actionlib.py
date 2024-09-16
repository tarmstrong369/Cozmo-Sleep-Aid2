#!/usr/bin/env python3
from routines import emotions
import random
from enum import Enum 

class ActionType(Enum):
    Inactive = 0
    Pleasant = 1
    PleasantInactive = 2
    Unpleasant = 3
    UnpleasantInactive = 4
    Active = 5
    PleasantActive = 6
    UnpleasantActive = 7

class ActionLibrary:
    def __init__(self):  # , robot: cozmo.robot.Robot):
        self.animation = []
        self.chosen_animation = []

        # get all emotions in lists by category
        self.list_p = ['emotions.pleasant.' + func for func in dir(emotions.pleasant)
                       if callable(getattr(emotions.pleasant, func)) and not func.startswith("__")]
        self.list_unp = ['emotions.unpleasant.' + func for func in dir(emotions.unpleasant)
                         if callable(getattr(emotions.unpleasant, func)) and not func.startswith("__")]
        self.list_act = ['emotions.active.' + func for func in dir(emotions.active)
                         if callable(getattr(emotions.active, func)) and not func.startswith("__")]
        self.list_inact = ['emotions.inactive.' + func for func in dir(emotions.inactive)
                           if callable(getattr(emotions.inactive, func)) and not func.startswith("__")]
        self.list_act_unp = ['emotions.active_unpleasant.' + func for func in dir(emotions.active_unpleasant)
                             if callable(getattr(emotions.active_unpleasant, func)) and not func.startswith("__")]
        self.list_inact_unp = ['emotions.inactive_unpleasant.' + func for func in dir(emotions.inactive_unpleasant)
                               if callable(getattr(emotions.inactive_unpleasant, func)) and not func.startswith("__")]
        self.list_act_p = ['emotions.active_pleasant.' + func for func in dir(emotions.active_pleasant)
                           if callable(getattr(emotions.active_pleasant, func)) and not func.startswith("__")]
        self.list_inact_p = ['emotions.inactive_pleasant.' + func for func in dir(emotions.inactive_pleasant)
                             if callable(getattr(emotions.inactive_pleasant, func)) and not func.startswith("__")]
        self.emotion_list = self.list_act_p + self.list_act_unp + self.list_inact_p + self.list_inact_unp

    def random_short(self):
        """
        gets 1 random animation from all emotions
        :return: list with output of specific animation in Emotion class
        """
        rand = random.randint(0, len(self.emotion_list) - 1)
        self.animation = eval(self.emotion_list[rand] + '()')
        self.chosen_animation = self.emotion_list[rand]
        return self.animation

    def random_long(self, num_anims=3):
        """
        gets a variable set of animations from all emotions
        :param num_anims: define number of random animations to grab. 3 by default
        :return: 1 list with output of specific sets of animations in Emotion class
        """
        rand = random.sample(range(len(self.emotion_list) - 1), num_anims)
        for val in rand:
            self.animation += eval(self.emotion_list[val] + '()')
            self.chosen_animation.append(self.emotion_list[val])
        return self.animation

    def get_attention_getting_routine(self):
        """
        attention routine. Moves from least invasive to most invasive actions
        start with inactive or pleasant or inactive pleasant
        inactive unpleasant or unpleasant next
        active or active pleasant
        active unpleasant
        :return: list with individual robot commands as strings in index
        """
        # Initialize behaviors and repeats lists
        # get off the charger first
        #self.animation = ['robot.drive_straight(distance_mm(100), speed_mmps(40))', 'robot.move_head(1)',]
        #self.animation = ['cli.drive_wheels(lwheel_speed: 40, rwheel_speed: 40, duration: 2.5)', 'cli.move_head(1)',]
        self.animatin = []
        repeats = []
        # Fill with desired subroutines
        # Waking up, drive off contacts

        # initial routine
        init_category = random.randint(0, 2)
        if init_category == 0:
            # inactive
            index = random.randint(0, len(self.list_inact) - 1)
            self.animation += eval(self.list_inact[index] + '()')
            repeats.append(0)
        elif init_category == 1:
            # pleasant
            index = random.randint(0, len(self.list_p) - 1)
            self.animation += eval(self.list_p[index] + '()')
            repeats.append(0)
        else:
            # inactive pleasant
            index = random.randint(0, len(self.list_inact_p) - 1)
            self.animation += eval(self.list_inact_p[index] + '()')
            repeats.append(0)

        # second routine
        second_category = random.randint(0, 1)
        if second_category == 0:
            # unpleasant
            index = random.randint(0, len(self.list_unp) - 1)
            self.animation += eval(self.list_unp[index] + '()')
            repeats.append(0)
        else:
            # unpleasant inactive
            index = random.randint(0, len(self.list_inact_unp) - 1)
            self.animation += eval(self.list_inact_unp[index] + '()')
            repeats.append(0)

        # third routine
        third_category = random.randint(0, 1)
        if third_category == 0:
            # active
            index = random.randint(0, len(self.list_act) - 1)
            self.animation += eval(self.list_act[index] + '()')
            repeats.append(0)
        else:
            # active pleasant
            index = random.randint(0, len(self.list_act_p) - 1)
            self.animation += eval(self.list_act_p[index] + '()')
            repeats.append(0)

        # final routine
        # active unpleasant
        index = random.randint(0, len(self.list_act_unp) - 1)
        self.animation += eval(self.list_act_unp[index] + '()')
        repeats.append(0)

        return self.animation, repeats

    def get_act_type(self):
        act_type_list=[]
        for i in range(random.randint(1,4)):
            act_type_list.append(random.randint(0,7))
        return act_type_list

    def get_attention_getting_walk_route(self,act_type_list):
        """
        attention routine. Takes a lit of action type commands and moves through
        each action in the list
        :return: list with individual robot commands as strings in index
        """
        # Initialize behaviors and repeats lists
        # get off the charger first
        #self.animation = ['cli.drive_straight(distance_mm(100), speed_mmps(40))', 'cli.move_head(1)',]
        self.animation = []
        repeats = []
        # Fill with desired subroutines
        # Waking up, drive off contacts

        for act_type in act_type_list:
            # fromt act_type
            if act_type == ActionType.Inactive.value:
                # inactive
                index = random.randint(0, len(self.list_inact) - 1)
                self.animation += eval(self.list_inact[index] + '()')
                repeats.append(0)
            elif act_type == ActionType.Pleasant.value:
                # pleasant
                index = random.randint(0, len(self.list_p) - 1)
                self.animation += eval(self.list_p[index] + '()')
                repeats.append(0)
            elif act_type == ActionType.PleasantInactive.value:
                # inactive pleasant
                index = random.randint(0, len(self.list_inact_p) - 1)
                self.animation += eval(self.list_inact_p[index] + '()')
                repeats.append(0)
            elif act_type == ActionType.Unpleasant.value:
                # unpleasant
                index = random.randint(0, len(self.list_unp) - 1)
                self.animation += eval(self.list_unp[index] + '()')
                repeats.append(0)
            elif act_type == ActionType.UnpleasantInactive.value:
                # unpleasant inactive
                index = random.randint(0, len(self.list_inact_unp) - 1)
                self.animation += eval(self.list_inact_unp[index] + '()')
                repeats.append(0)
            elif act_type == ActionType.Active.value:
                # active
                index = random.randint(0, len(self.list_act) - 1)
                self.animation += eval(self.list_act[index] + '()')
                repeats.append(0)
            elif act_type == ActionType.PleasantActive.value:
                # active pleasant
                index = random.randint(0, len(self.list_act_p) - 1)
                self.animation += eval(self.list_act_p[index] + '()')
                repeats.append(0)
            elif act_type == ActionType.UnpleasantActive.value:
                # active unpleasant
                index = random.randint(0, len(self.list_act_unp) - 1)
                self.animation += eval(self.list_act_unp[index] + '()')
                repeats.append(0)

        return self.animation, repeats

    # Version of attention getting where we do num_actions behaviors from a random behavior group/type
    def get_basic_attention_routine(self, num_actions = 3):
        # Pick behavior group/type
        act_type = random.randint(0,7)
        # Names of each chosen behavior
        behavior_names = []
        # The actions for each chosen behavior
        self.animation = []
        # Determine which group was picked, then get num_actions random behaviors from that group,
        # Store the behavior names, and get the behavior cozmo calls/actions, and add a 1 second delay
        # between behaviors 
        if act_type == ActionType.Inactive.value:
            # inactive
            for i in range(num_actions):
                index = random.randint(0, len(self.list_inact) - 1)
                behavior_names.append(self.list_inact[index] + '()')
                self.animation += eval(self.list_inact[index] + '()')
                #self.animation += ['wait(1)']
        elif act_type == ActionType.Pleasant.value:
            # pleasant
            for i in range(num_actions):
                index = random.randint(0, len(self.list_p) - 1)
                behavior_names.append(self.list_p[index] + '()')
                self.animation += eval(self.list_p[index] + '()')
                #self.animation += ['wait(1)']
        elif act_type == ActionType.PleasantInactive.value:
            # inactive pleasant
            for i in range(num_actions):
                index = random.randint(0, len(self.list_inact_p) - 1)
                behavior_names.append(self.list_inact_p[index] + '()')
                self.animation += eval(self.list_inact_p[index] + '()')
                #self.animation += ['wait(1)']
        elif act_type == ActionType.Unpleasant.value:
            # unpleasant
            for i in range(num_actions):
                index = random.randint(0, len(self.list_unp) - 1)
                behavior_names.append(self.list_unp[index] + '()')
                self.animation += eval(self.list_unp[index] + '()')
                #self.animation += ['wait(1)']
        elif act_type == ActionType.UnpleasantInactive.value:
            # unpleasant inactive
            for i in range(num_actions):
                index = random.randint(0, len(self.list_inact_unp) - 1)
                behavior_names.append(self.list_inact_unp[index] + '()')
                self.animation += eval(self.list_inact_unp[index] + '()')
                #self.animation += ['wait(1)']
        elif act_type == ActionType.Active.value:
            # active
            for i in range(num_actions):
                index = random.randint(0, len(self.list_act) - 1)
                behavior_names.append(self.list_act[index] + '()')
                self.animation += eval(self.list_act[index] + '()')
                #self.animation += ['wait(1)']
        elif act_type == ActionType.PleasantActive.value:
            # active pleasant
            for i in range(num_actions):
                index = random.randint(0, len(self.list_act_p) - 1)
                behavior_names.append(self.list_act_p[index] + '()')
                self.animation += eval(self.list_act_p[index] + '()')
                #self.animation += ['wait(1)']
        elif act_type == ActionType.UnpleasantActive.value:
            # active unpleasant
            for i in range(num_actions):
                index = random.randint(0, len(self.list_act_unp) - 1)
                behavior_names.append(self.list_act_unp[index] + '()')
                self.animation += eval(self.list_act_unp[index] + '()')
                #self.animation += ['wait(1)']
        
        # Return our list of actions to perform, the chosen group, and the names of the behaviors
        return self.animation, act_type, behavior_names

    def get_success_routine(self):
        # Initialize behaviors and repeats lists
        self.animation = []
        repeats = []
        self.animation = eval(str(emotions.done.success()))
        repeats.append(0)
        return self.animation, repeats

    def get_done_routine(self):
        # Initialize behaviors and repeats lists
        self.animation = []
        repeats = []
        self.animation = eval(str(emotions.done.poop()))
        repeats.append(0)
        return self.animation, repeats


library = ActionLibrary()
