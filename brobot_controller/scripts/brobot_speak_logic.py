#!/usr/bin/python3


class brobot_speak_logic():
    def __init__(self):
        self.brobot_talkback = None

    def basic_speak_logic(self, message, username):
        if "hello" in message:
            brobot_talk_back = f"Hello, {username}, it's me Brobot, How are you"
        else:
            self.brobot_talkback = "Brobot Brobot Brobot Brobot"
        return brobot_talk_back

    
    def do_something():
        pass
