# casino.py
import random

class CasinoGame:
    def __init__(self):
        self.balance = 1000
        self.symbols = ['🍒', '🍋', '🍊', '🍇', '🔔', '💎', '7']
        self.current_reel = [random.choice(self.symbols) for _ in range(3)]
        self.spinning = False
        
    def spin(self):
        if self.balance >= 10 and not self.spinning:
            self.balance -= 10
            self.spinning = True
            self.current_reel = [random.choice(self.symbols) for _ in range(3)]
            win = self.check_win()
            if win:
                self.balance += 50
            self.spinning = False
            return self.current_reel, win
        return None, False
        
    def check_win(self):
        return len(set(self.current_reel)) == 1