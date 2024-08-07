#!/usr/bin/env python3
import math


class Active:
    def __init__(self):
        self.animation = []

    def spin(self):
        """
        Active spin
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.drive_wheels(-1000, 1000, lwheel_acc=999, rwheel_acc=999, duration=5)',
            'wait(1)',
            'cli.drive_wheels(1000, -1000, lwheel_acc=999, rwheel_acc=999, duration=5)',
        ]
        return self.animation

    def race(self):
        """
        Active race
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.move_lift(-20)',
            'cli.drive_straight(145, 999)',
            'cli.drive_straight(-145, 999)',
            'cli.drive_straight(145, 999)',
            'cli.drive_straight(-145, 999)',
            'cli.drive_straight(145, 999)',
            'cli.drive_straight(-145, 999)',
        ]
        return self.animation

    def twitch(self):
        """
        Active twitch
        :return: animation list
        """
        self.animation = [
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.drive_wheels(-100, 100, lwheel_acc=999, rwheel_acc=999, duration=0.6)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.turn_in_place(math.radians(180))',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.drive_wheels(-100, 100, lwheel_acc=999, rwheel_acc=999, duration=0.6)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.turn_in_place(math.radians(180))',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.drive_wheels(-100, 100, lwheel_acc=999, rwheel_acc=999, duration=0.6)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.turn_in_place(math.radians(180))',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.drive_wheels(-100, 100, lwheel_acc=999, rwheel_acc=999, duration=0.6)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.turn_in_place(math.radians(180))',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.drive_wheels(-100, 100, lwheel_acc=999, rwheel_acc=999, duration=0.6)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.turn_in_place(math.radians(180))',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.drive_wheels(-100, 100, lwheel_acc=999, rwheel_acc=999, duration=0.6)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.turn_in_place(math.radians(180))',
        ]
        return self.animation

    #def square(self):
    #    """
    #    Active move in a square
    #    :return: animation list
    #    """
    #    self.animation = [
    #        'wait(2)',
    #        'cli.turn_in_place(math.radians(90))',
    #        'cli.drive_straight(120, 200)',
    #        'cli.turn_in_place(math.radians(90))',
    #        'cli.drive_straight(120, 200)',
    #        'cli.turn_in_place(math.radians(90))',
    #        'cli.drive_straight(120, 200)',
    #        'cli.turn_in_place(math.radians(90))',
    #        'cli.drive_straight(120, 200)',
    #    ]
    #    return self.animation


class Inactive:
    def __init__(self):
        self.animation = []

    def lift_head(self):
        """
        Inactive head lift
        :return: animation list
        """
        self.animation = [
            'wait(1)',
            'cli.move_head(.5)',
            'wait(5)',
        ]
        return self.animation

    def blink(self):
        """
        Inactive blink
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("CodeLabBlink")',
            'wait(3.5)',
            'cli.play_anim_group("CodeLabBlink")',
            'wait(3.5)',
            'cli.play_anim_group("CodeLabBlink")',
            'wait(3.5)',
            'cli.play_anim_group("CodeLabBlink")',
        ]
        return self.animation

    def looking(self):
        """
        Inactive looking around
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.move_head(.5)',
            'wait(3)',
            'cli.move_head(-.5)',
            'wait(6)',
            'cli.move_head(.5)',
            'wait(4)',
        ]
        return self.animation

    # def rainbow(self):
    #     """
    #     Inactive backpack rainbow
    #     :return: animation list
    #     """
    #     self.animation = [
    #         'wait(2)',
    #         'cli.move_lift(-5)',
    #         'cli.move_head(-5)',
    #         'cli.set_center_backpack_lights(pycozmo.protocol_encoder.LightState(on_color=pycozmo.lights.Color(rgb=(255, 0, 0)).to_int16()))',
    #         'wait(2)',
    #         'cli.set_center_backpack_lights(pycozmo.protocol_encoder.LightState(on_color=pycozmo.lights.Color(rgb=(255, 127, 0)).to_int16()))',
    #         'wait(2)',
    #         'cli.set_center_backpack_lights(pycozmo.protocol_encoder.LightState(on_color=pycozmo.lights.Color(rgb=(255, 255, 0)).to_int16()))',
    #         'wait(2)',
    #         'cli.set_center_backpack_lights(pycozmo.protocol_encoder.LightState(on_color=pycozmo.lights.Color(rgb=(0, 255, 0)).to_int16()))',
    #         'wait(2)',
    #         'cli.set_center_backpack_lights(pycozmo.protocol_encoder.LightState(on_color=pycozmo.lights.Color(rgb=(0, 0, 255)).to_int16()))',
    #         'wait(2)',
    #         'cli.set_center_backpack_lights(pycozmo.protocol_encoder.LightState(on_color=pycozmo.lights.Color(rgb=(39, 0, 51)).to_int16()))',
    #         'wait(2)',
    #         'cli.set_center_backpack_lights(pycozmo.protocol_encoder.LightState(on_color=pycozmo.lights.Color(rgb=(139, 0, 255)).to_int16()))',
    #         'wait(2)',
    #     ]
    #     return self.animation


class Pleasant:
    def __init__(self):
        self.animation = []

    def attention(self):
        """
        Pleasant attention
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.drive_wheels(-100, 100, lwheel_acc=999, rwheel_acc=999, duration=0.6)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.4)',
            'cli.move_head(1)',
            'wait(1)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.3)',
            'cli.drive_wheels(-100, 100, lwheel_acc=999, rwheel_acc=999, duration=0.6)',
            'cli.drive_wheels(100, -100, lwheel_acc=999, rwheel_acc=999, duration=0.4)',
            'wait(1)',
            'cli.play_anim_group("CodeLabChatty")',
            'cli.play_anim_group("VC_Alrighty")',
            'wait(2)',
        ]
        return self.animation

    def hunting(self):
        """
        Pleasant pounce
        :return: animation list
        """
        self.animation = [
            'wait(1)',
            'cli.drive_straight(10, 10)',
            'wait(1)',
            'cli.drive_straight(10, 10)',
            'wait(1)',
            'cli.play_anim_group("PouncePounce")',
            'cli.play_anim_group("CodeLabYes")',
        ]
        return self.animation

    def happy_realization(self):
        """
        Pleasant happy realization
        :return: animation list
        """
        self.animation = [
            'wait(1)',
            'cli.move_lift(-2)',
            'wait(2)',
            'cli.play_anim_group("SoftSparkUpgradeLift")',
            'cli.move_head(5)',
            'cli.play_anim_group("ComeHere_AlreadyHere")',
            'wait(1)',
        ]
        return self.animation

    def impressed(self):
        """
        Pleasant impressed
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.drive_straight(50, 50)',
            'wait(1)',
            'cli.move_head(1)',
            'cli.move_head(1)',
            'cli.play_anim_group("CodeLabAmazed")',
        ]
        return self.animation

    def agree(self):
        """
        Pleasant agree
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("CodeLabYes")',
            'cli.move_head(3)',
            'cli.move_head(3)',
            'cli.move_head(3)',
            'wait(.6)',
            'cli.move_head(-3)',
            'wait(.6)',
            'cli.move_head(3)',
            'wait(1)',
            'cli.play_anim_group("CodeLabHappy")',
        ]
        return self.animation


class Unpleasant:
    def __init__(self):
        self.animation = []

    def disappointment(self):
        """
        Unpleasant disappointment
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("CodeLabStaring")',
            'cli.move_lift(1)',
            'wait(2)',
            'cli.move_lift(-100)',
            'wait(1)',
            'cli.move_head(3)',
            'wait(2)',
            'cli.move_lift(0.25)',
            'wait(2)',
            'cli.move_lift(-100)',
            'wait(1)',
        ]
        return self.animation

    def frustrated(self):
        """
        Unpleasant frustrated
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.move_lift(-5)',
            'cli.move_head(5)',
            'cli.play_anim_group("CodeLabSquint1")',
            'wait(5)',
            'cli.play_anim_group("FrustratedByFailureMajor")',
            'cli.move_lift(-5)',
            'cli.move_head(-5)',
            'cli.turn_in_place(math.radians(180))',
        ]
        return self.animation

    def frustrated_02(self):
        """
        Second Unpleasant frustrated
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("CodeLabStaring")',
            'cli.turn_in_place(math.radians(180))',
            'cli.move_lift(100)',
            'cli.move_lift(-100)',
            'wait(1)',
            'cli.move_lift(100)',
            'wait(1)',
            'cli.move_lift(-100)',
            'wait(1)',
        ]
        return self.animation

    def confused(self):
        """
        Unpleasant confused
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.turn_in_place(math.radians(30))',
            'wait(2.5)',
            'cli.turn_in_place(math.radians(-60))',
            'wait(2.5)',
            'cli.turn_in_place(math.radians(30))',
            'cli.play_anim_group("CodeLabIDK")',

        ]
        return self.animation

    def sad(self):
        """
        Unpleasant sad
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("CodeLabUnhappy")',
            'wait(.5)',
            'cli.play_anim_group("CodeLabDejected")',
            'wait(2)',
        ]
        return self.animation


class ActivePleasant:
    def __init__(self):
        self.animation = []

    def amazed(self):
        """
        Active and Pleasant amazed
        :return: animation list
        """
        self.animation = [
            'cli.move_head(10)',
            'cli.turn_in_place(math.radians(10))',
            'cli.play_anim_group("PopAWheelieInitial")',
            'cli.turn_in_place(math.radians(-20))',
            'cli.play_anim_group("PopAWheelieInitial")',
            'cli.turn_in_place(math.radians(10))',
            'cli.play_anim_group("SuccessfulWheelie")',
        ]
        return self.animation

    def swaying(self):
        """
        Active and Pleasant swaying
        :return: animation list
        """
        self.animation = [
            'cli.play_anim_group("CodeLabIdle")',
            'cli.play_anim_group("CodeLabHappy")',
        ]
        return self.animation

    def firetruck(self):
        """
        Active and Pleasant firetruck
        :return: animation list
        """
        self.animation = [
            'cli.play_anim_group("CodeLabFireTruck")',
        ]
        return self.animation

    def duck(self):
        """
        Active and Pleasant ducking
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("CodeLabChatty")',
            'cli.play_anim_group("CodeLabDuck")',
            'cli.play_anim_group("CodeLabChatty")',
        ]
        return self.animation


class ActiveUnpleasant:
    def __init__(self):
        self.animation = []

    def scared(self):
        """
        Active and Unpleasant scared
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.drive_straight(140, 50)',
            'cli.move_head(100)',
            'wait(1)',
            'cli.play_anim_group("Shiver")',
            'cli.play_anim_group("CodeLabScaredCozmo")',
            'cli.drive_straight(-140, 999)',
            'cli.play_anim_group("Shiver")',
            'cli.play_anim_group("Shiver")',
            'cli.play_anim_group("Shiver")',
        ]
        return self.animation

    def grossed_out(self):
        """
        Active and Unpleasant grossed out
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.move_lift(50)',
            'cli.drive_straight(-50, 150)',
            'cli.play_anim_group("CodeLabVampire")',
            'cli.play_anim_group("CodeLabSquint1")',
            'cli.move_lift(-50)',
            'cli.drive_wheels(-1000, 1000, lwheel_acc=250, rwheel_acc=250, duration=2)',
            'cli.drive_straight(200, 150)',
        ]
        return self.animation

    def frustrated(self):
        """
        Active and Unpleasant frustrated
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("CodeLabUnhappy")',
            'cli.play_anim_group("CodeLabFrustrated")',
            'cli.move_head(-100)',
            'cli.turn_in_place(math.radians(180))',
            'cli.move_lift(100)',
            'wait(.25)',
            'cli.move_lift(-100)',
            'wait(.25)',
            'cli.move_lift(100)',
            'wait(.25)',
            'cli.move_lift(-100)',
            'wait(.25)',
            'cli.move_lift(100)',
            'wait(.25)',
            'cli.move_lift(-100)',
            'wait(.25)',
            'cli.move_lift(100)',
            'wait(.25)',
            'cli.move_lift(-100)',
            'wait(.25)',
            'cli.move_lift(100)',
            'wait(.25)',
            'cli.move_lift(-100)',
            'wait(.25)',
        ]
        return self.animation

    def arguing(self):
        """
        Active and Unpleasant arguing
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.move_head(-50)',
            'cli.move_lift(50)',
            'cli.move_lift(50)',
            'wait(1)',
            'cli.move_lift(-50)',
            'wait(1)',
            'cli.move_head(50)',
            'cli.play_anim_group("GuardDogBusted")',
        ]
        return self.animation


class InactivePleasant:
    def __init__(self):
        self.animation = []



    def yes(self):
        """
        Inactive and Pleasant agreement
        :return: animation list
        """
        self.animation = [
            'cli.move_head(10)',
            'wait(2)',
            'cli.play_anim_group("CodeLabTakaTaka")',
            'wait(4)',
            'cli.play_anim_group("CodeLabYes")',
            'wait(4)',
            'cli.play_anim_group("CodeLabYes")',
        ]
        return self.animation

    def surprise(self):
        """
        Inactive and Pleasant surprise
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'wait(2)',
            'cli.play_anim_group("ReactToPokeReaction")',
            'cli.move_head(1)',
            'wait(1)',
            'cli.play_anim_group("PetDetectionShort")',
            'cli.play_anim_group("VC_Alrighty")',
        ]
        return self.animation

    def thinking(self):
        """
        Inactive and Pleasant thinking
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("CodeLabThinking")',
            'wait(2)',
            'cli.play_anim_group("VC_Alrighty")',
        ]
        return self.animation


class InactiveUnpleasant:
    def __init__(self):
        self.animation = []

    def grumpy(self):
        """
        Inactive and Unpleasant grumpy
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("NothingToDoBoredIntro")',
            'cli.play_anim_group("CodeLabSquint2")',
            'wait(1)',
            'cli.move_head(.5)',
            'wait(3)',
            'cli.move_head(-50)',
            'wait(1)',
        ]
        return self.animation

    def bored(self):
        """
        Inactive and Unpleasant bored
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.play_anim_group("CodeLabBored")',
            'wait(5)',
            'cli.play_anim_group("CodeLabSquint2")',
            'wait(5)',
        ]
        return self.animation

    #def stare(self):
    #    """
    #    Inactive and Unpleasant stare
    #    :return: animation list
    #    """
    #    self.animation = [
    #        'wait(2)',
    #        'cli.play_anim_group("CodeLabStaring")',
    #        'wait(5)',
    #        'cli.play_anim_group("CodeLabStaring")',
    #        'wait(5)',
    #    ]
    #    return self.animation

    def sad(self):
        """
        Inactive and Unpleasant sad
        :return: animation list
        """
        self.animation = [
            'wait(2)',
            'cli.move_lift(-5)',
            'cli.move_head(-5)',
            'wait(5)',
            'cli.play_anim_group("CodeLabSquint1")',
            'cli.play_anim_group("FeedingInterrupted_Severe")',
            'wait(5)',
        ]
        return self.animation


class Done:
    def __init__(self):
        self.animation = []

    def success(self):
        """
        user stood up routine with return to near charger
        :return: animation list
        """
        self.animation = [
            #'cli.play_anim_group("CodeLabHappy")',
            'cli.play_anim("anim_SHARELab_goal_achieved")',
            'wait(2)',
        ]
        return self.animation

    def poop(self):
        """
        user did not stand up routine, return to near charger
        :return: animation list
        """
        self.animation = [
            'cli.play_anim_group("CodeLabDejected")',
            'wait(2)',
        ]
        return self.animation


# initialize each category for use in other modules
active = Active()
inactive = Inactive()
pleasant = Pleasant()
unpleasant = Unpleasant()
active_pleasant = ActivePleasant()
active_unpleasant = ActiveUnpleasant()
inactive_pleasant = InactivePleasant()
inactive_unpleasant = InactiveUnpleasant()
done = Done()
